#!/usr/bin/env python3
"""Bridge official EGO-Planner PositionCommand → our controller topics.

Supports optional per-drone namespace (uav0 → /uav0/planner/*).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from drone_msgs.msg import TrajectoryCommand

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'quadrotor_msgs not found — build ego_vendor first: '
        'colcon build --packages-up-to ego_planner'
    ) from exc


GOAL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class EgoCmdBridge(Node):
    def __init__(self) -> None:
        super().__init__('ego_cmd_bridge')
        self.declare_parameter('namespace', '')
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('cmd_topic', '')
        self.declare_parameter('optimal_topic', '')
        self.declare_parameter('goal_in_topic', '')
        self.declare_parameter('goal_out_topic', '')
        self.declare_parameter('path_maxlen', 2000)
        # Runaway-command guard: drop pos_cmds beyond this |x|,|y| bound or
        # farther than this from the current mission goal (see _cmd_is_sane).
        self.declare_parameter('cmd_bound', 25.0)
        self.declare_parameter('auto_goal_enable', False)
        self.declare_parameter('auto_goal_x', 15.0)
        self.declare_parameter('auto_goal_y', 0.0)
        self.declare_parameter('auto_goal_z', 1.0)
        self.declare_parameter('auto_goal_delay', 8.0)
        self.declare_parameter('auto_goal_repeats', 5)
        self.declare_parameter('auto_goal_period', 0.4)
        self.declare_parameter('cruise_height', 1.0)
        # Multi-drone: every EGO node also subscribes to /move_base_simple/goal —
        # fan-out would give ALL drones the same goal. Keep false for swarms.
        self.declare_parameter('publish_move_base_simple', True)

        ns = self.get_parameter('namespace').get_parameter_value().string_value.strip()
        drone_id = int(self.get_parameter('drone_id').value)
        prefix = f'/{ns}' if ns else ''

        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        opt_topic = self.get_parameter('optimal_topic').get_parameter_value().string_value
        goal_in = self.get_parameter('goal_in_topic').get_parameter_value().string_value
        goal_out = self.get_parameter('goal_out_topic').get_parameter_value().string_value

        if not cmd_topic:
            cmd_topic = f'/drone_{drone_id}_planning/pos_cmd'
        if not opt_topic:
            opt_topic = f'/drone_{drone_id}_plan_vis/optimal_list'
        if not goal_in:
            goal_in = f'{prefix}/drone/goal' if prefix else '/drone/goal'
        if not goal_out:
            # Fan-out destination for the FSM. Prefer remapped relative "goal"
            # when namespaced; for single-drone use move_base_simple once.
            goal_out = f'{prefix}/drone/goal' if prefix else '/move_base_simple/goal'

        traj_topic = f'{prefix}/planner/trajectory_cmd'
        local_topic = f'{prefix}/planner/local_goal'
        path_topic = f'{prefix}/planner/trajectory'

        self.traj_pub = self.create_publisher(TrajectoryCommand, traj_topic, 10)
        self.local_pub = self.create_publisher(PoseStamped, local_topic, 10)
        # Latch yellow path so late RViz (Path B/D) still sees /planner/trajectory.
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, path_topic, path_qos)
        self._goal_out_topic = goal_out
        self.goal_out = self.create_publisher(PoseStamped, goal_out, GOAL_QOS)
        # Optional second fan-out — NEVER the same topic twice (double Triggered
        # → planNextWaypoint spin_some while node already in rclcpp::spin → crash).
        self.goal_out_legacy = None
        legacy = '/move_base_simple/goal'
        fanout = bool(self.get_parameter('publish_move_base_simple').value)
        if fanout and goal_out.rstrip('/') != legacy:
            self.goal_out_legacy = self.create_publisher(PoseStamped, legacy, GOAL_QOS)

        self.create_subscription(PositionCommand, cmd_topic, self.on_cmd, 50)
        self.create_subscription(PoseStamped, goal_in, self.on_goal, GOAL_QOS)
        self.create_subscription(Marker, opt_topic, self.on_optimal, 10)

        self.path = Path()
        self.path.header.frame_id = 'map'
        self.have_optimal_path = False
        self._goal_xy = None
        self._auto_left = 0
        self._auto_started = False
        self._delay_timer = None
        self._auto_timer = None

        if self.get_parameter('auto_goal_enable').value:
            delay = float(self.get_parameter('auto_goal_delay').value)
            self._auto_left = int(self.get_parameter('auto_goal_repeats').value)
            self._delay_timer = self.create_timer(delay, self._start_auto_goal)

        self.get_logger().info(
            f'EGO bridge ready ns={ns or "-"}: {cmd_topic} → {local_topic}; '
            f'Path ← {opt_topic}; goals {goal_in} → {goal_out}')

    def _start_auto_goal(self) -> None:
        if self._auto_started:
            return
        self._auto_started = True
        if self._delay_timer is not None:
            self._delay_timer.cancel()
        period = float(self.get_parameter('auto_goal_period').value)
        self._auto_timer = self.create_timer(period, self._publish_auto_goal)
        self._publish_auto_goal()

    def _publish_auto_goal(self) -> None:
        if self._auto_left <= 0:
            if self._auto_timer is not None:
                self._auto_timer.cancel()
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self.get_parameter('auto_goal_x').value)
        msg.pose.position.y = float(self.get_parameter('auto_goal_y').value)
        msg.pose.position.z = float(self.get_parameter('auto_goal_z').value)
        msg.pose.orientation.w = 1.0
        self.goal_out.publish(msg)
        if self.goal_out_legacy is not None:
            self.goal_out_legacy.publish(msg)
        self._auto_left -= 1
        self.get_logger().info(
            f'Auto goal ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f}, '
            f'{msg.pose.position.z:.1f}) remaining={self._auto_left}')

    def on_goal(self, msg: PoseStamped) -> None:
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = 'map' if not msg.header.frame_id else msg.header.frame_id
        out.pose = msg.pose
        if out.pose.position.z < 0.5:
            out.pose.position.z = float(self.get_parameter('cruise_height').value)
        self.goal_out.publish(out)
        if self.goal_out_legacy is not None:
            self.goal_out_legacy.publish(out)
        # Track the active mission goal for the runaway-command guard below.
        self._goal_xy = (float(out.pose.position.x), float(out.pose.position.y))

    def _cmd_is_sane(self, p) -> bool:
        """Reject planner commands that ran away from the mission goal.

        Under CPU load EGO occasionally extrapolates a stale trajectory and
        streams position commands far outside the work area; forwarding them
        turns a recoverable stall into a cross-map runaway. Anything beyond
        the workspace bound or far from the current goal is dropped, letting
        the controller time out to an anchored hover instead.
        """
        bound = float(self.get_parameter('cmd_bound').value)
        if abs(p.x) > bound or abs(p.y) > bound:
            return False
        if self._goal_xy is not None:
            dx = p.x - self._goal_xy[0]
            dy = p.y - self._goal_xy[1]
            if dx * dx + dy * dy > bound * bound:
                return False
        return True

    def on_optimal(self, msg: Marker) -> None:
        if not msg.points:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for p in msg.points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(p.x)
            ps.pose.position.y = float(p.y)
            ps.pose.position.z = float(p.z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path = path
        self.have_optimal_path = True
        self.path_pub.publish(self.path)

    def on_cmd(self, msg: PositionCommand) -> None:
        now = self.get_clock().now().to_msg()

        if not self._cmd_is_sane(msg.position):
            self.get_logger().warn(
                f'dropping runaway pos_cmd ({msg.position.x:.1f}, {msg.position.y:.1f})',
                throttle_duration_sec=2.0)
            return

        lg = PoseStamped()
        lg.header.stamp = now
        lg.header.frame_id = 'map'
        lg.pose.position = msg.position
        lg.pose.orientation.w = 1.0
        self.local_pub.publish(lg)

        tc = TrajectoryCommand()
        tc.header.stamp = now
        tc.header.frame_id = 'map'
        tc.position = msg.position
        tc.velocity = msg.velocity
        tc.acceleration = msg.acceleration
        tc.yaw = float(msg.yaw)
        tc.yaw_dot = float(msg.yaw_dot)
        tc.trajectory_ready = True
        self.traj_pub.publish(tc)

        if not self.have_optimal_path:
            ps = PoseStamped()
            ps.header = lg.header
            ps.pose.position = msg.position
            ps.pose.orientation.w = 1.0
            self.path.header.stamp = now
            self.path.poses.append(ps)
            maxlen = int(self.get_parameter('path_maxlen').value)
            if len(self.path.poses) > maxlen:
                self.path.poses = self.path.poses[-maxlen:]
            self.path_pub.publish(self.path)


def main() -> None:
    rclpy.init()
    node = EgoCmdBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
