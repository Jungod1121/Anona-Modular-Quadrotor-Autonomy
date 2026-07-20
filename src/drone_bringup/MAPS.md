# Available obstacle maps

Catalog of portable scenes for Path A–H / Mode D. Selected with:

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=gcopter map:=official_forest
ros2 run drone_bringup dashboard   # map picker in the web UI
```

All maps are **procedural point clouds** (no PCD scene files). A `map_adapter`
node republishes onto both `/map/obstacles` and `/map_generator/global_cloud`,
publishes a coarse `/map/occupancy` slice at cruise height, and `/map/metadata`
(JSON bounds / difficulty / seed). Official maps get an AABB boundary wall cloud
when the generator omits outer walls. The legacy `cloud_bridge` entry point remains
available for minimal relay-only setups.

## Pose / AABB (why forest works, mockamap used to fail)

Forest places the drone **outside** the obstacle box (`init x=-15` vs cloud ±13)
and keeps an east–west corridor (`map/clear_y`). Mockamap fills its AABB — start
must sit outside (or on a free edge) and corridors must be wider than inflate/`dist0`.

| ID | Generator size (m) | Init → Goal | Notes |
|----|--------------------|-------------|--------|
| **official_forest** | 26×20×3 | (−15,0,1)→(15,0,1) | Outside cloud + `clear_y` |
| **official_perlin** | 50×26×5, fill 0.05 | (−22,0,1)→(22,0,1) | Start outside box |
| **official_posts** | 20×20×4, 12 pillars | (−8,0,1)→(8,0,1) | Thinned density |
| **official_maze2d** | 20×20, `road_width=1.2` | (−9,0,1)→(9,0,1) | Path B inflate 0.05 / dist0 0.25 |
| **official_maze3d** | 20×20×4, roadRad 8 | (−6,0,1)→(6,0,1) | Z in **[0,z]** (workspace patch) |

Path A expands its occupancy grid for any `official_*` map (`map_origin≈(-25,-15)`,
size 50×30×4). Path C lowers `DilateRadius` on mazes/posts and widens `MapBound`.

## Official EGO resources

| ID | Package | Obstacles | Notes |
|----|---------|-----------|--------|
| **official_forest** | `map_generator` / `random_forest` | Cylinders + yawed rings | Highest completeness; Path B/C default |
| **official_perlin** | `mockamap` type=1 | Fractal Perlin 3D blobs | Larger thinner fill than stock |
| **official_posts** | `mockamap` type=2 | Hollow rectangular pillars | Sparse for flyable gaps |
| **official_maze2d** | `mockamap` type=3 | Recursive-division walls | Wider roads than stock 0.5 m |
| **official_maze3d** | `mockamap` type=4 | Voronoi 3D maze | Z≥0; wider passages |

Sources: `src/ego_vendor/map_generator`, `src/ego_vendor/mockamap`.

## Homemade / early-project maps

| ID | Package | Obstacles |
|----|---------|-----------|
| **dense_field** | `drone_map` | Cylinders/spheres + boundary walls (Path A default) |
| **sparse** | `drone_map` | Few obstacles, open field |
| **narrow_corridor** | `drone_map` | S-bend: 3 staggered 1.2–1.8 m doors + side clutter (PLAN §5.3) |
| **ego_maze2d_port** | `drone_map` | Port of mockamap maze2D into homemade frame |
| **ego_forest_port** | `drone_map` | Port of forest cylinders+rings into homemade frame |

## Tier presets (review dataset)

Curated difficulty tiers — **additive** presets; legacy map IDs above are unchanged.

| ID | Tier | Based on | `difficulty` | Default `seed` |
|----|------|----------|--------------|----------------|
| **tier_simple_open** | simple | `sparse` | simple | 42 |
| **tier_medium_corridor** | medium | `narrow_corridor` | medium | 42 |
| **tier_complex_forest** | complex | `official_forest` | complex | 1 |
| **tier_extreme_maze** | extreme | `official_maze2d` | extreme | 510 |

Optional larger variants: **forest_wide** (denser `ego_dense_forest`), **dense_asymmetric**
(diagonal start–goal through `dense_field`).

### Per-map metadata fields

Every catalog entry includes: `difficulty` (`simple` \| `medium` \| `complex` \| `extreme`),
`seed`, `obstacle_family`, `safety_radius`, and `bounds` (`xmin`…`zmax`). Published on
`/map/metadata` when `map_adapter` is running.

## Defaults

| Planner | Default map |
|---------|-------------|
| homemade | `dense_field` |
| ego | `official_forest` |
| gcopter | `official_forest` |
| fuel_explore | `narrow_corridor` (homemade gate corridor; not FUEL upstream warehouse) |
| vfh | `dense_field` |
| sac | `dense_field` |

Path D is **FUEL-style**, not upstream FUEL: homemade maps + simplified fog/frontier
FSM + EGO trajectories. The original FUEL demo uses its own PCD warehouse (20×12 m)
and Fast-Planner hierarchy — we do **not** ship that scene or planner stack.

Aliases: `forest`→`official_forest`, `perlin`→`official_perlin`, `maze2d`→`official_maze2d`,
`narrow`→`narrow_corridor`, `auto`→planner default.

## Goals (RViz)

Publish with RViz **2D Goal Pose** → `/drone/goal` (swarm: `/uav0/drone/goal`).
Height is not meaningful from the 2D tool — Path B uses `fsm/cruise_height`
(default 1.0 m); Path A uses `cruise_z`; Path C uses `CruiseHeight`. Dashboard
does not send goals.
