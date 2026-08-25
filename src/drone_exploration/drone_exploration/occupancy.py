"""Shared fog-of-war occupancy grid for FUEL-style exploration."""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

UNK, FREE, OCC = 0, 1, 2


class OccupancyGrid3D:
    """Coarse voxel grid: UNKNOWN / FREE / OCC over an AABB."""

    def __init__(
        self,
        box_min: Sequence[float],
        box_max: Sequence[float],
        resolution: float,
    ) -> None:
        self.res = float(max(0.15, resolution))
        self.origin = np.asarray(box_min, dtype=np.float64)
        extent = np.asarray(box_max, dtype=np.float64) - self.origin
        self.dims = np.maximum(1, np.ceil(extent / self.res).astype(np.int32))
        self.grid = np.zeros(self.dims, dtype=np.uint8)  # UNK

    def world_to_idx(self, pts: np.ndarray) -> np.ndarray:
        idx = np.floor((pts - self.origin) / self.res).astype(np.int32)
        return idx

    def idx_to_world(self, idx: np.ndarray) -> np.ndarray:
        return self.origin + (idx.astype(np.float64) + 0.5) * self.res

    def in_bounds(self, idx: np.ndarray) -> np.ndarray:
        return (
            (idx[:, 0] >= 0) & (idx[:, 0] < self.dims[0])
            & (idx[:, 1] >= 0) & (idx[:, 1] < self.dims[1])
            & (idx[:, 2] >= 0) & (idx[:, 2] < self.dims[2])
        )

    def reveal(
        self,
        sensor_xyz: np.ndarray,
        cloud_xyz: Optional[np.ndarray],
        radius: float,
        z_half: float,
        inflate: float,
    ) -> None:
        """Mark sphere around sensor: OCC near cloud points, else FREE."""
        sx, sy, sz = sensor_xyz
        # Candidate voxel indices inside axis-aligned sensing box.
        r = float(radius)
        zh = float(z_half)
        lo = self.world_to_idx(np.array([[sx - r, sy - r, sz - zh]]))[0]
        hi = self.world_to_idx(np.array([[sx + r, sy + r, sz + zh]]))[0]
        lo = np.maximum(lo, 0)
        hi = np.minimum(hi, self.dims - 1)
        if np.any(hi < lo):
            return

        xs = np.arange(lo[0], hi[0] + 1)
        ys = np.arange(lo[1], hi[1] + 1)
        zs = np.arange(lo[2], hi[2] + 1)
        xx, yy, zz = np.meshgrid(xs, ys, zs, indexing='ij')
        idx = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)
        centers = self.idx_to_world(idx)
        dxy = np.hypot(centers[:, 0] - sx, centers[:, 1] - sy)
        dz = np.abs(centers[:, 2] - sz)
        mask = (dxy <= r) & (dz <= zh)
        if not np.any(mask):
            return
        idx = idx[mask]
        centers = centers[mask]

        # Default: free inside sensing volume.
        self.grid[idx[:, 0], idx[:, 1], idx[:, 2]] = FREE

        if cloud_xyz is None or cloud_xyz.size == 0:
            return

        # Obstacles inside sensing volume.
        dxy_c = np.hypot(cloud_xyz[:, 0] - sx, cloud_xyz[:, 1] - sy)
        dz_c = np.abs(cloud_xyz[:, 2] - sz)
        near = cloud_xyz[(dxy_c <= r) & (dz_c <= zh)]
        if near.size == 0:
            return

        # Inflate: mark voxels within inflate of any obstacle point.
        occ_idx = self.world_to_idx(near)
        valid = self.in_bounds(occ_idx)
        occ_idx = occ_idx[valid]
        if occ_idx.size == 0:
            return
        self.grid[occ_idx[:, 0], occ_idx[:, 1], occ_idx[:, 2]] = OCC

        if inflate > 1e-6:
            rad_v = max(1, int(np.ceil(inflate / self.res)))
            for ox, oy, oz in np.unique(occ_idx, axis=0):
                x0, x1 = max(0, ox - rad_v), min(self.dims[0], ox + rad_v + 1)
                y0, y1 = max(0, oy - rad_v), min(self.dims[1], oy + rad_v + 1)
                z0, z1 = max(0, oz - rad_v), min(self.dims[2], oz + rad_v + 1)
                block = self.grid[x0:x1, y0:y1, z0:z1]
                # Only inflate over free/unk inside the already revealed volume:
                # keep OCC; leave UNK outside sensing as-is by only painting FREE→OCC.
                free_mask = block == FREE
                block[free_mask] = OCC
                self.grid[x0:x1, y0:y1, z0:z1] = block

    def observed_obstacle_points(self) -> np.ndarray:
        occ = np.argwhere(self.grid == OCC)
        if occ.size == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return self.idx_to_world(occ)

    def frontier_clusters(
        self,
        cruise_z: float,
        min_size: int = 3,
        max_clusters: int = 16,
    ) -> List[Tuple[np.ndarray, int]]:
        """Return (centroid xyz, cluster cell count) for frontier free clusters."""
        free_mask = self.grid == FREE
        if not free_mask.any():
            return []

        # Frontier: FREE with at least one UNKNOWN 6-neighbour.
        # Vectorised via padded shifts — the pure-Python per-cell loop cost
        # O(cells) interpreter round-trips per publish tick.
        unk = self.grid == UNK
        pad = np.pad(unk, 1, mode='constant', constant_values=False)
        has_unk_nbr = (
            pad[0:-2, 1:-1, 1:-1] | pad[2:, 1:-1, 1:-1]
            | pad[1:-1, 0:-2, 1:-1] | pad[1:-1, 2:, 1:-1]
            | pad[1:-1, 1:-1, 0:-2] | pad[1:-1, 1:-1, 2:]
        )
        frontiers = np.argwhere(free_mask & has_unk_nbr)
        if frontiers.size == 0:
            return []

        pts = self.idx_to_world(frontiers.astype(np.int32))
        # Prefer slices near cruise height.
        band = np.abs(pts[:, 2] - cruise_z) < max(0.8, self.res * 2.5)
        if np.any(band):
            pts = pts[band]
            frontiers = frontiers[band]

        # Greedy clustering in XY. Note: membership is measured against the
        # SEED only (not chained), so each cluster is exactly "all frontier
        # cells within sep of its seed" and the previous nested rescan loop
        # was equivalent to a single masked pass — now done in numpy.
        remaining = np.arange(len(pts))
        clusters: List[Tuple[np.ndarray, int]] = []
        sep = max(1.0, self.res * 3.0)
        while remaining.size and len(clusters) < max_clusters:
            seed = remaining[0]
            d2 = (pts[remaining, 0] - pts[seed, 0]) ** 2 + \
                 (pts[remaining, 1] - pts[seed, 1]) ** 2
            memb = remaining[d2 < sep * sep]
            remaining = remaining[d2 >= sep * sep]
            if memb.size < min_size:
                continue
            c = pts[memb].mean(axis=0)
            c[2] = cruise_z
            clusters.append((c, int(memb.size)))
        return clusters
