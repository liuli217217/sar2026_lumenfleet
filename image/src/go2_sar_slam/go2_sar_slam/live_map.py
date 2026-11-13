"""Occupancy grid built online from the lidar (log-odds), plus camera coverage.

Reuses go2_sar_core.map_utils.OccupancyMap for coordinates, inflation, distance field
and A*; only the map content differs (no prior map file).
"""
import math

import numpy as np
from scipy.sparse.csgraph import dijkstra

from go2_sar_core.map_utils import OccupancyMap

L_HIT, L_MISS, L_MIN, L_MAX = 0.9, 0.4, -2.0, 4.0
L_OCCUPIED, L_FREE = 1.0, -0.5
L_CONFIDENT, L_MISS_CONFIDENT = 2.0, 0.1   # walls seen many times fade slowly (static scene;
                                           # rays grazing a 1-cell wall end would erase it)


class LiveMap(OccupancyMap):
    """Square grid centred on the start pose. Cell (row, col), row 0 = lowest y."""

    def __init__(self, size=20.0, resolution=0.05):
        self.resolution = resolution
        n = int(round(size / resolution))
        self.height = self.width = n
        self.origin = (-size / 2.0, -size / 2.0)
        self.logodds = np.zeros((n, n), np.float32)
        self.seen = np.zeros((n, n), bool)          # covered by the camera

    # ------------------------------------------------------------- state
    @property
    def occupied(self):
        return self.logodds > L_OCCUPIED

    @property
    def free(self):
        return self.logodds < L_FREE

    @property
    def unknown(self):
        return ~(self.occupied | self.free)

    def cells(self, xy):
        """(N, 2) world points -> unique in-map (rows, cols)."""
        if len(xy) == 0:
            return np.zeros(0, int), np.zeros(0, int)
        cols = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(np.int64)
        rows = np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(np.int64)
        ok = (rows >= 0) & (rows < self.height) & (cols >= 0) & (cols < self.width)
        keys = np.unique(rows[ok] * self.width + cols[ok])
        return keys // self.width, keys % self.width

    def cell_centres(self, rows, cols):
        return np.column_stack([self.origin[0] + (cols + 0.5) * self.resolution,
                                self.origin[1] + (rows + 0.5) * self.resolution])

    def known_bounds(self, margin=1.0):
        """(r0, r1, c0, c1) window around everything observed so far."""
        rows, cols = np.nonzero(~self.unknown)
        m = int(margin / self.resolution)
        if len(rows) == 0:
            return 0, self.height, 0, self.width
        return (max(rows.min() - m, 0), min(rows.max() + m + 1, self.height),
                max(cols.min() - m, 0), min(cols.max() + m + 1, self.width))

    # ------------------------------------------------------------ update
    def update(self, hit_xy, free_xy, soft_free_xy=None):
        """Lidar update: hit_xy are obstacle points, free_xy points on free rays (world).
        soft_free_xy: free unless the cell is occupied now (the robot footprint: with
        a localization error it may overlap the wall next to the body)."""
        hr, hc = self.cells(hit_xy)
        fr, fc = self.cells(free_xy)
        if soft_free_xy is not None and len(soft_free_xy):
            sr, sc = self.cells(soft_free_xy)
            ok = self.logodds[sr, sc] <= L_OCCUPIED
            fr, fc = np.concatenate([fr, sr[ok]]), np.concatenate([fc, sc[ok]])
        if len(fr):
            keep = ~np.isin(fr * self.width + fc, hr * self.width + hc)   # a hit wins over a miss
            fr, fc = fr[keep], fc[keep]
            lo = self.logodds[fr, fc]
            self.logodds[fr, fc] = np.maximum(lo - np.where(lo > L_CONFIDENT, L_MISS_CONFIDENT, L_MISS), L_MIN)
        if len(hr):
            self.logodds[hr, hc] = np.minimum(self.logodds[hr, hc] + L_HIT, L_MAX)

    def mark_seen(self, xy):
        r, c = self.cells(xy)
        self.seen[r, c] = True

    # -------------------------------------------------------- distances
    def geodesic(self, blocked, cost, start, window, step=2):
        """Dijkstra distance [m] from `start` over a coarse version (step x step cells)
        of the window (r0, r1, c0, c1). Returns (dist, r0, c0, step); dist is coarse."""
        r0, r1, c0, c1 = window
        b = blocked[r0:r1, c0:c1]
        k = cost[r0:r1, c0:c1]
        h, w = b.shape[0] // step, b.shape[1] // step
        # optimistic pooling: a coarse cell is open if any fine cell is (a 0.9 m door
        # leaves only a ~0.25 m strip after inflation); A* on the fine grid has the last word
        b = b[:h * step, :w * step].reshape(h, step, w, step).all(axis=(1, 3))
        k = k[:h * step, :w * step].reshape(h, step, w, step).mean(axis=(1, 3))
        srow, scol = self.to_cell(*start)
        s = ((srow - r0) // step, (scol - c0) // step)
        if not (0 <= s[0] < h and 0 <= s[1] < w):
            return np.full((h, w), np.inf), r0, c0, step
        b[s] = False
        dist = dijkstra(self._graph(b, k, self.resolution * step), indices=s[0] * w + s[1])
        return dist.reshape(h, w), r0, c0, step

    def plan(self, blocked, cost, start, goal, margin=1.0, fallback_radius=0.0):
        """Shortest path inside the observed window (plus start and goal), see
        OccupancyMap.plan."""
        s, g = self.to_cell(*start), self.to_cell(*goal)
        r0, r1, c0, c1 = self.known_bounds(margin)
        m = int(margin / self.resolution)
        window = (max(min(r0, s[0] - m, g[0] - m), 0), min(max(r1, s[0] + m, g[0] + m), self.height),
                  max(min(c0, s[1] - m, g[1] - m), 0), min(max(c1, s[1] + m, g[1] + m), self.width))
        return super().plan(blocked, cost, start, goal, fallback_radius=fallback_radius, window=window)

def angle_bins(points, n_bins):
    """Bin index of every (N, 2) point by its direction from the origin."""
    ang = np.arctan2(points[:, 1], points[:, 0])
    return (np.floor((ang + math.pi) / (2 * math.pi) * n_bins).astype(np.int64)) % n_bins


def bin_angles(n_bins):
    return -math.pi + (np.arange(n_bins) + 0.5) * 2 * math.pi / n_bins
