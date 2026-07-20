# Path B display (EGO style)

| Layer | Topic | Look |
|---|---|---|
| ForestCloud | `/map_generator/global_cloud` | Gray/white flat — full simulation obstacles |
| OccupancyGray | `/drone_0_grid/grid_map/occupancy` | Gray — local raw occupancy |
| InflatedOcc | `/drone_0_grid/grid_map/occupancy_inflate` | **Rainbow by height** (AxisColor Z) — paper/demo look |

Official warehouse ships two looks:
- `default.rviz`: inflate as solid blue FlatColor `(29;108;212)`
- Paper / README / demo GIFs: inflate (or map cloud) colored by **height** → rainbow “highland”

Path B uses the **demo/paper** rainbow-height inflate so RViz matches what you see in the warehouse videos.

Pipeline:
```
global cloud → local_sense (5 m) → EGO grid_map → rainbow inflate (Z)
```

Configs: `ego_avoidance.rviz`, `ego_swarm.rviz`.
