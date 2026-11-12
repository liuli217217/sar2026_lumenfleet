"""Prior occupancy map (ROS map_server png + yaml), inflation and shortest paths."""
import math
import os

import cv2
import numpy as np
import yaml
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


class OccupancyMap:
    """Static map: True = occupied. Cell (row, col), row 0 = lowest y."""

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        image = cv2.imread(os.path.join(os.path.dirname(yaml_path), meta['image']), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(meta['image'])
        occ = (255 - image.astype(np.float32)) / 255.0
        if meta.get('negate', 0):
            occ = 1.0 - occ
        self.occupied = np.flipud(occ > meta.get('occupied_thresh', 0.65))   # row 0 = origin row
        self.resolution = float(meta['resolution'])
        self.origin = (float(meta['origin'][0]), float(meta['origin'][1]))
        self.height, self.width = self.occupied.shape

    # ----------------------------------------------------------- coordinates
    def to_cell(self, x, y):
        return (int(math.floor((y - self.origin[1]) / self.resolution)),
                int(math.floor((x - self.origin[0]) / self.resolution)))

    def to_world(self, row, col):
        return (self.origin[0] + (col + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution)

    def inside(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def wall_points(self):
        """World coordinates of all occupied cells (for scan-to-map matching)."""
        rows, cols = np.nonzero(self.occupied)
        return np.column_stack([self.origin[0] + (cols + 0.5) * self.resolution,
                                self.origin[1] + (rows + 0.5) * self.resolution])

    def inflate(self, occupied, radius):
        """Grow obstacles by `radius` metres (robot footprint)."""
        r = int(math.ceil(radius / self.resolution))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        return cv2.dilate(occupied.astype(np.uint8), kernel) > 0

    def distance_field(self, occupied):
        """Distance [m] from every cell to the nearest occupied cell."""
        free = (~occupied).astype(np.uint8)
        return cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.resolution

    def nearest_free(self, blocked, x, y, max_radius=1.5):
        """Closest non-blocked cell centre to (x, y), or None."""
        row, col = self.to_cell(x, y)
        r = int(max_radius / self.resolution)
        best = None
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                rr, cc = row + dr, col + dc
                if self.inside(rr, cc) and not blocked[rr, cc]:
                    d = dr * dr + dc * dc
                    if best is None or d < best[0]:
                        best = (d, rr, cc)
        return None if best is None else self.to_world(best[1], best[2])

    # ------------------------------------------------------------------ A*
    @staticmethod
    def _graph(b, k, cell):
        """8-connected grid graph over the open cells of b (no corner cutting); moving
        into a cell costs length * (1 + k[cell])."""
        h, w = b.shape
        idx = np.arange(h * w).reshape(h, w)
        src, dst, wt = [], [], []
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
            ra, rb = slice(max(-dr, 0), h - max(dr, 0)), slice(max(dr, 0), h - max(-dr, 0) or None)
            ca, cb = slice(max(-dc, 0), w - max(dc, 0)), slice(max(dc, 0), w - max(-dc, 0) or None)
            ok = ~b[ra, ca] & ~b[rb, cb]
            if dr and dc:   # both side cells must be open as well
                ok &= ~b[rb, ca] & ~b[ra, cb]
            length = cell * math.hypot(dr, dc)
            a, z = idx[ra, ca][ok], idx[rb, cb][ok]
            src += [a, z]
            dst += [z, a]
            wt += [length * (1.0 + k[rb, cb][ok]), length * (1.0 + k[ra, ca][ok])]
        src, dst, wt = np.concatenate(src), np.concatenate(dst), np.concatenate(wt)
        return csr_matrix((wt, (src, dst)), shape=(h * w, h * w))

    def plan(self, blocked, cost, start, goal, fallback_radius=0.0, window=None):
        """Shortest path on the grid (Dijkstra in C, scipy.sparse.csgraph), inside
        `window` (r0, r1, c0, c1; default: the whole map). blocked: bool grid, cost:
        extra cost per cell (>= 0). fallback_radius: if the goal itself cannot be
        reached, go to the reachable cell closest to it within this radius (a victim
        against a wall). Returns (path [(x, y), ...], length [m]) or (None, inf)."""
        s, g = self.to_cell(*start), self.to_cell(*goal)
        if not (self.inside(*s) and self.inside(*g)):
            return None, math.inf
        r0, r1, c0, c1 = window or (0, self.height, 0, self.width)
        b = blocked[r0:r1, c0:c1].copy()
        h, w = b.shape
        ls, lg = (s[0] - r0) * w + (s[1] - c0), (g[0] - r0) * w + (g[1] - c0)
        b.flat[ls] = False          # the robot may start inside the inflation zone
        if b.flat[lg] and fallback_radius <= 0:
            return None, math.inf
        dist, pred = dijkstra(self._graph(b, cost[r0:r1, c0:c1], self.resolution), indices=ls,
                              return_predecessors=True)
        if not np.isfinite(dist[lg]):
            if fallback_radius <= 0:
                return None, math.inf
            rr, cc = np.divmod(np.arange(h * w), w)
            off = np.hypot(rr - (g[0] - r0), cc - (g[1] - c0)) * self.resolution
            ok = np.isfinite(dist) & (off <= fallback_radius)
            if not ok.any():
                return None, math.inf
            lg = int(np.flatnonzero(ok)[np.argmin(off[ok])])
        cells = [lg]
        while cells[-1] != ls:
            cells.append(pred[cells[-1]])
        cells = np.array(cells[::-1])
        rows, cols = cells // w + r0, cells % w + c0
        path = [self.to_world(r, c) for r, c in zip(rows, cols)]
        length = float(np.sum(np.hypot(np.diff(rows), np.diff(cols)))) * self.resolution
        return path, length

def load_victims(path):
    with open(path) as f:
        return [(v['name'], float(v['x']), float(v['y'])) for v in yaml.safe_load(f)['victims']]
