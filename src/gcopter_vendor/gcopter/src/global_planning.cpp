#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <drone_msgs/msg/trajectory_command.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    std::string odomTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;
    double cruiseHeight;

    Config(rclcpp::Node &node)
    {
        node.declare_parameter("MapTopic", std::string("/map_generator/global_cloud"));
        node.declare_parameter("TargetTopic", std::string("/drone/goal"));
        node.declare_parameter("OdomTopic", std::string("/drone/odom"));
        node.declare_parameter("DilateRadius", 0.35);
        node.declare_parameter("VoxelWidth", 0.25);
        node.declare_parameter("MapBound", std::vector<double>{-18.0, 18.0, -12.0, 12.0, 0.0, 4.0});
        node.declare_parameter("TimeoutRRT", 0.02);
        node.declare_parameter("MaxVelMag", 1.5);
        node.declare_parameter("MaxBdrMag", 2.1);
        node.declare_parameter("MaxTiltAngle", 1.05);
        node.declare_parameter("MinThrust", 2.0);
        node.declare_parameter("MaxThrust", 20.0);
        node.declare_parameter("VehicleMass", 1.0);
        node.declare_parameter("GravAcc", 9.8);
        node.declare_parameter("HorizDrag", 0.70);
        node.declare_parameter("VertDrag", 0.80);
        node.declare_parameter("ParasDrag", 0.01);
        node.declare_parameter("SpeedEps", 0.0001);
        node.declare_parameter("WeightT", 20.0);
        node.declare_parameter("ChiVec", std::vector<double>{1.0e4, 1.0e4, 1.0e4, 1.0e4, 1.0e5});
        node.declare_parameter("SmoothingEps", 1.0e-2);
        node.declare_parameter("IntegralIntervs", 16);
        node.declare_parameter("RelCostTol", 1.0e-5);
        node.declare_parameter("CruiseHeight", 1.0);

        node.get_parameter("MapTopic", mapTopic);
        node.get_parameter("TargetTopic", targetTopic);
        node.get_parameter("OdomTopic", odomTopic);
        node.get_parameter("DilateRadius", dilateRadius);
        node.get_parameter("VoxelWidth", voxelWidth);
        node.get_parameter("MapBound", mapBound);
        node.get_parameter("TimeoutRRT", timeoutRRT);
        node.get_parameter("MaxVelMag", maxVelMag);
        node.get_parameter("MaxBdrMag", maxBdrMag);
        node.get_parameter("MaxTiltAngle", maxTiltAngle);
        node.get_parameter("MinThrust", minThrust);
        node.get_parameter("MaxThrust", maxThrust);
        node.get_parameter("VehicleMass", vehicleMass);
        node.get_parameter("GravAcc", gravAcc);
        node.get_parameter("HorizDrag", horizDrag);
        node.get_parameter("VertDrag", vertDrag);
        node.get_parameter("ParasDrag", parasDrag);
        node.get_parameter("SpeedEps", speedEps);
        node.get_parameter("WeightT", weightT);
        node.get_parameter("ChiVec", chiVec);
        node.get_parameter("SmoothingEps", smoothingEps);
        node.get_parameter("IntegralIntervs", integralIntervs);
        node.get_parameter("RelCostTol", relCostTol);
        node.get_parameter("CruiseHeight", cruiseHeight);
    }
};

class GlobalPlanner : public rclcpp::Node
{
private:
    Config config;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr localGoalPub;
    rclcpp::Publisher<drone_msgs::msg::TrajectoryCommand>::SharedPtr trajCmdPub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub;
    Visualizer visualizer;
    voxel_map::VoxelMap voxelMap;
    Trajectory<5> traj;
    double trajStamp{0.0};
    bool mapInitialized{false};
    bool haveOdom{false};
    bool havePlanned{false};
    int mapIngestCount_{0};
    size_t lastMapPoints_{0};
    double lastMapIngestSec_{-1.0};
    double lastPathPubSec_{-1.0};
    Eigen::Vector3d odomPos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d odomVel{Eigen::Vector3d::Zero()};

public:
    GlobalPlanner() : Node("global_planning_node"), config(*this), visualizer(*this)
    {
        Eigen::Vector3i xyz(
            static_cast<int>((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth),
            static_cast<int>((config.mapBound[3] - config.mapBound[2]) / config.voxelWidth),
            static_cast<int>((config.mapBound[5] - config.mapBound[4]) / config.voxelWidth));
        Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        mapSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            config.mapTopic, cloud_qos,
            std::bind(&GlobalPlanner::mapCallBack, this, std::placeholders::_1));

        targetSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            config.targetTopic, rclcpp::QoS(10).reliable(),
            std::bind(&GlobalPlanner::targetCallBack, this, std::placeholders::_1));

        odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
            config.odomTopic, 50,
            std::bind(&GlobalPlanner::odomCallback, this, std::placeholders::_1));

        localGoalPub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planner/local_goal", 10);
        trajCmdPub = this->create_publisher<drone_msgs::msg::TrajectoryCommand>("/planner/trajectory_cmd", 10);
        // Latch so late RViz / dashboard subscribers still see the yellow path.
        auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        pathPub = this->create_publisher<nav_msgs::msg::Path>("/planner/trajectory", path_qos);

        RCLCPP_INFO(get_logger(),
                    "GCOPTER Path C ready: map=%s goal=%s → /planner/local_goal + trajectory_cmd",
                    config.mapTopic.c_str(), config.targetTopic.c_str());
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odomPos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        odomVel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
        haveOdom = true;
    }

    void mapCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Lock after a successful plan. Before that, allow a few debounced
        // reinjects so a stale TransientLocal leftover can be overwritten —
        // but never rebuild voxel maps at cloud publish rate (~10 Hz), which
        // starves process() and looks like a dead drone / missing blue path.
        if (havePlanned) {
            return;
        }
        if (mapInitialized) {
            if (mapIngestCount_ >= 3) {
                return;
            }
            const double now_sec = this->now().seconds();
            if (lastMapIngestSec_ >= 0.0 && (now_sec - lastMapIngestSec_) < 1.0) {
                return;
            }
        }
        if (msg->data.empty() || msg->point_step == 0) {
            return;
        }
        size_t total = msg->data.size() / msg->point_step;
        if (total == 0) {
            return;
        }

        Eigen::Vector3i xyz(
            static_cast<int>((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth),
            static_cast<int>((config.mapBound[3] - config.mapBound[2]) / config.voxelWidth),
            static_cast<int>((config.mapBound[5] - config.mapBound[4]) / config.voxelWidth));
        Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);
        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        float *fdata = reinterpret_cast<float *>(&msg->data[0]);
        for (size_t i = 0; i < total; ++i) {
            size_t cur = msg->point_step / sizeof(float) * i;
            if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2])) {
                continue;
            }
            voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], fdata[cur + 2]));
        }
        voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));
        mapInitialized = true;
        ++mapIngestCount_;
        lastMapPoints_ = total;
        lastMapIngestSec_ = this->now().seconds();
        RCLCPP_INFO(get_logger(), "GCOPTER map ingested (%zu points, n=%d)",
                    total, mapIngestCount_);
    }

    static bool nudgeToFree(voxel_map::VoxelMap & map, Eigen::Vector3d & pt,
                            double z_lo, double z_hi)
    {
        if (map.query(pt) == 0) {
            return true;
        }
        for (double r = 0.15; r <= 4.0; r += 0.15) {
            for (int iz = -2; iz <= 2; ++iz) {
                const double z = std::clamp(pt.z() + 0.25 * iz, z_lo, z_hi);
                for (int k = 0; k < 24; ++k) {
                    const double a = k * (2.0 * M_PI / 24.0);
                    Eigen::Vector3d c(pt.x() + r * std::cos(a),
                                     pt.y() + r * std::sin(a), z);
                    if (map.query(c) == 0) {
                        pt = c;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!mapInitialized) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for map…");
            return;
        }
        if (!haveOdom) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odom…");
            return;
        }
        // Sparse first cloud → front-end thinks air is free → "straight first goal".
        if (mapIngestCount_ < 2 || lastMapPoints_ < 400) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Map still warming up (n=%d pts=%zu)…",
                mapIngestCount_, lastMapPoints_);
            return;
        }

        double zGoal = msg->pose.position.z;
        if (zGoal < 0.5) {
            zGoal = config.cruiseHeight;
        }
        Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
        Eigen::Vector3d start = odomPos;
        if (start.z() < 0.3) {
            start.z() = zGoal;
        }

        const double z_lo = config.mapBound[4] + 0.2;
        const double z_hi = config.mapBound[5] - 0.2;
        Eigen::Vector3d goal0 = goal;
        if (!nudgeToFree(voxelMap, goal, z_lo, z_hi)) {
            RCLCPP_WARN(get_logger(), "Goal in obstacle (%.2f %.2f %.2f)",
                        goal0.x(), goal0.y(), goal0.z());
            return;
        }
        if ((goal - goal0).norm() > 1e-3) {
            RCLCPP_WARN(get_logger(),
                        "Goal near obstacle — nudged (%.2f,%.2f,%.2f) → (%.2f,%.2f,%.2f)",
                        goal0.x(), goal0.y(), goal0.z(), goal.x(), goal.y(), goal.z());
        }
        Eigen::Vector3d start0 = start;
        if (!nudgeToFree(voxelMap, start, z_lo, z_hi)) {
            RCLCPP_WARN(get_logger(), "Start in obstacle — cannot plan");
            return;
        }
        if ((start - start0).norm() > 1e-3) {
            RCLCPP_WARN(get_logger(), "Start nudged to free space");
        }

        visualizer.visualizeStartGoal(start, 0.5, 0);
        visualizer.visualizeStartGoal(goal, 0.5, 1);
        // Lock the map before planning so mapCallBack cannot rebuild voxelMap
        // under plan()'s feet.
        havePlanned = true;
        plan(start, goal);
    }

    void plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
    {
        std::vector<Eigen::Vector3d> route;
        sfc_gen::planPath<voxel_map::VoxelMap>(
            start, goal, voxelMap.getOrigin(), voxelMap.getCorner(),
            &voxelMap, 0.01, route);

        if (route.size() <= 1) {
            RCLCPP_ERROR(get_logger(),
                "Path front-end failed (no free 3D route; refusing straight-line fallback)");
            havePlanned = false;
            return;
        }

        std::vector<Eigen::MatrixX4d> hPolys;
        std::vector<Eigen::Vector3d> pc;
        voxelMap.getSurf(pc);
        sfc_gen::convexCover(route, pc, voxelMap.getOrigin(), voxelMap.getCorner(), 7.0, 3.0, hPolys);
        sfc_gen::shortCut(hPolys);

        visualizer.visualizePolytope(hPolys);

        Eigen::Matrix3d iniState, finState;
        iniState << route.front(), odomVel, Eigen::Vector3d::Zero();
        finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

        gcopter::GCOPTER_PolytopeSFC gcopter;

        Eigen::VectorXd magnitudeBounds(5), penaltyWeights(5), physicalParams(6);
        magnitudeBounds << config.maxVelMag, config.maxBdrMag, config.maxTiltAngle, config.minThrust, config.maxThrust;
        penaltyWeights << config.chiVec[0], config.chiVec[1], config.chiVec[2], config.chiVec[3], config.chiVec[4];
        physicalParams << config.vehicleMass, config.gravAcc, config.horizDrag, config.vertDrag, config.parasDrag, config.speedEps;

        traj.clear();
        if (!gcopter.setup(config.weightT, iniState, finState, hPolys, INFINITY,
                           config.smoothingEps, config.integralIntervs,
                           magnitudeBounds, penaltyWeights, physicalParams)) {
            RCLCPP_ERROR(get_logger(), "GCOPTER setup failed");
            havePlanned = false;
            return;
        }
        if (std::isinf(gcopter.optimize(traj, config.relCostTol))) {
            RCLCPP_ERROR(get_logger(), "GCOPTER optimize failed");
            havePlanned = false;
            return;
        }

        if (traj.getPieceNum() > 0) {
            trajStamp = this->now().seconds();
            visualizer.visualize(traj, route);
            publishPath();
            lastPathPubSec_ = trajStamp;
            RCLCPP_INFO(get_logger(), "GCOPTER traj pieces=%d duration=%.2fs",
                        traj.getPieceNum(), traj.getTotalDuration());
        } else {
            // Unlock so a later goal can try again with a fresh map if needed.
            havePlanned = false;
        }
    }

    void publishPath()
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = now();
        const double T = traj.getTotalDuration();
        for (double t = 0.0; t <= T; t += 0.05) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            const Eigen::Vector3d p = traj.getPos(t);
            ps.pose.position.x = p.x();
            ps.pose.position.y = p.y();
            ps.pose.position.z = p.z();
            ps.pose.orientation.w = 1.0;
            path.poses.push_back(ps);
        }
        pathPub->publish(path);
    }

    void process()
    {
        if (traj.getPieceNum() <= 0) {
            return;
        }

        const double now_sec = this->now().seconds();
        // Keep yellow path visible for late RViz subscribers / volatile displays.
        if (lastPathPubSec_ < 0.0 || (now_sec - lastPathPubSec_) > 1.0) {
            publishPath();
            lastPathPubSec_ = now_sec;
        }

        double delta = now_sec - trajStamp;
        if (delta < 0.0) {
            return;
        }
        if (delta > traj.getTotalDuration()) {
            delta = traj.getTotalDuration();
        }

        const Eigen::Vector3d pos = traj.getPos(delta);
        const Eigen::Vector3d vel = traj.getVel(delta);
        const Eigen::Vector3d acc = traj.getAcc(delta);

        double thr;
        Eigen::Vector4d quat;
        Eigen::Vector3d omg;
        Eigen::VectorXd physicalParams(6);
        physicalParams << config.vehicleMass, config.gravAcc, config.horizDrag,
            config.vertDrag, config.parasDrag, config.speedEps;
        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));
        flatmap.forward(vel, acc, traj.getJer(delta), 0.0, 0.0, thr, quat, omg);

        double yaw = std::atan2(
            2.0 * (quat(0) * quat(3) + quat(1) * quat(2)),
            1.0 - 2.0 * (quat(2) * quat(2) + quat(3) * quat(3)));
        // Prefer heading along velocity when moving.
        if (vel.head<2>().norm() > 0.15) {
            yaw = std::atan2(vel.y(), vel.x());
        }

        geometry_msgs::msg::PoseStamped lg;
        lg.header.stamp = this->now();
        lg.header.frame_id = "map";
        lg.pose.position.x = pos.x();
        lg.pose.position.y = pos.y();
        lg.pose.position.z = pos.z();
        lg.pose.orientation.z = std::sin(yaw * 0.5);
        lg.pose.orientation.w = std::cos(yaw * 0.5);
        localGoalPub->publish(lg);

        drone_msgs::msg::TrajectoryCommand tc;
        tc.header = lg.header;
        tc.position = lg.pose.position;
        tc.velocity.x = vel.x();
        tc.velocity.y = vel.y();
        tc.velocity.z = vel.z();
        tc.acceleration.x = acc.x();
        tc.acceleration.y = acc.y();
        tc.acceleration.z = acc.z();
        tc.yaw = yaw;
        tc.yaw_dot = 0.0;
        tc.trajectory_ready = true;
        trajCmdPub->publish(tc);

        std_msgs::msg::Float64 speedMsg, thrMsg;
        speedMsg.data = vel.norm();
        thrMsg.data = thr;
        visualizer.speedPub->publish(speedMsg);
        visualizer.thrPub->publish(thrMsg);
        visualizer.visualizeSphere(pos, config.dilateRadius);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlanner>();

    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
        node->process();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
