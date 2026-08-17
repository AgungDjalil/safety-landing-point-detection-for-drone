import os
import random
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass

import numpy as np


@dataclass
class FlatCluster:
    node_indices: np.ndarray
    normal:       np.ndarray
    planarity:    float
    centroid:     np.ndarray


@dataclass
class NodeNormal:
    node_idx:  int
    position:  np.ndarray
    normal:    np.ndarray
    planarity: float
    is_flat:   bool


class DBL_GNG_CPU:
    """DBL-GNG dengan numpy murni (tanpa torch/GPU) + multithreading."""

    def __init__(
        self,
        feature_number: int = 3,
        max_nodes: int = 500,
        alpha: float = 0.5,
        beta: float = 0.01,
        delta: float = 0.5,
        rho: float = 0.5,
        eps: float = 1e-4,
        planarity_threshold: float = 1.0,
        min_cluster_size:    int   = 5,
        normal_axis:         list | np.ndarray | None = None,
        max_normal_angle_deg: float = 30.0,
        node_normal_radius:  int   = 3,
        num_workers: int | None = None,
    ):
        self.feature_number      = feature_number
        self.M                   = max_nodes
        self.alpha               = alpha
        self.beta                = beta
        self.delta               = delta
        self.rho                 = rho
        self.eps                 = eps
        self.planarity_threshold = planarity_threshold
        self.min_cluster_size    = min_cluster_size
        self.node_normal_radius  = node_normal_radius

        self.num_workers = num_workers if num_workers else (os.cpu_count() or 1)
        self._executor   = ThreadPoolExecutor(max_workers=self.num_workers)

        _ax = np.array(normal_axis if normal_axis is not None
                       else [0.0, 0.0, 1.0], dtype=np.float32)
        _ax /= np.linalg.norm(_ax)
        self.normal_axis_np = _ax

        self.cos_threshold        = float(np.cos(np.deg2rad(max_normal_angle_deg)))
        self.max_normal_angle_deg = max_normal_angle_deg

        self.W:         np.ndarray | None = None
        self.C:         np.ndarray | None = None
        self.E:         np.ndarray | None = None
        self.S:         np.ndarray | None = None
        self.A_1:       np.ndarray | None = None
        self.A_2:       np.ndarray | None = None
        self.Delta_W_1: np.ndarray | None = None
        self.Delta_W_2: np.ndarray | None = None

        self.is_initialized = False
        self.node_point_map: dict[int, np.ndarray] = {}
        self._last_X: np.ndarray | None = None

        # Cache
        self._cache_adj:           np.ndarray | None = None
        self._cache_flat_clusters: list[FlatCluster] | None = None
        self._cache_node_normals:  list[NodeNormal]  | None = None
        self._cache_normals_np:    np.ndarray | None = None
        self._cache_planarity_np:  np.ndarray | None = None

    def get_outlier_points_numpy(self) -> np.ndarray:
        if self._last_X is None or not self.node_point_map:
            return np.empty((0, 3), dtype=np.float32)

        node_normals = self.get_node_normals()
        flat_node_set = {nn.node_idx for nn in node_normals if nn.is_flat}

        flat_point_indices = set()
        for nid in flat_node_set:
            if nid in self.node_point_map:
                flat_point_indices.update(self.node_point_map[nid].tolist())

        all_point_indices = set(range(len(self._last_X)))
        outlier_indices = all_point_indices - flat_point_indices

        if not outlier_indices:
            return np.empty((0, 3), dtype=np.float32)

        outlier_idx = np.array(sorted(outlier_indices), dtype=np.int64)
        return self._last_X[outlier_idx, :3].astype(np.float32)

    def _invalidate_cache(self):
        self._cache_adj           = None
        self._cache_flat_clusters = None
        self._cache_node_normals  = None
        self._cache_normals_np    = None
        self._cache_planarity_np  = None

    def _get_adj(self) -> np.ndarray:
        if self._cache_adj is not None:
            return self._cache_adj
        n   = len(self.W)
        adj = np.zeros((n, n), dtype=np.float32)
        if len(self.C) > 0:
            adj[self.C[:, 0], self.C[:, 1]] = 1.0
            adj[self.C[:, 1], self.C[:, 0]] = 1.0
        self._cache_adj = adj
        return adj

    def reset_batch(self):
        n = len(self.W)
        self.Delta_W_1  = np.zeros_like(self.W)
        self.Delta_W_2  = np.zeros_like(self.W)
        self.A_1        = np.zeros(n, dtype=np.float32)
        self.A_2        = np.zeros(n, dtype=np.float32)
        self.S          = np.zeros((n, n), dtype=np.float32)
        self.node_point_map = {}
        self._invalidate_cache()

    def initialize(self, data: np.ndarray, number_of_starting_points: int = 1):
        data = data[:, :self.feature_number].copy().astype(np.float32)
        np.random.shuffle(data)

        if len(data) < number_of_starting_points * 3:
            number_of_starting_points = max(1, len(data) // 3)

        node_list  = np.empty((0, self.feature_number), dtype=np.float32)
        edge_list  = np.empty((0, 2), dtype=np.int64)
        temp_data  = data.copy()
        batch_size = len(data) // number_of_starting_points

        for i in range(number_of_starting_points):
            if len(temp_data) < 3:
                break
            pool      = np.arange(len(temp_data), dtype=int)
            pool_s    = pool[-batch_size:] if batch_size <= len(pool) else pool
            sel_idx   = np.random.choice(pool_s)
            curr_node = temp_data[sel_idx]
            node_list = np.append(node_list, [curr_node], axis=0)

            y2  = np.sum(np.square(temp_data), axis=1)
            dot = 2 * np.matmul(curr_node, temp_data.T)
            idx = np.argsort(y2 - dot)

            nb_idx    = idx[2] if len(idx) > 2 else idx[-1]
            node_list = np.append(node_list, [temp_data[nb_idx]], axis=0)
            edge_list = np.append(edge_list, [[i * 2, i * 2 + 1]], axis=0)

            remaining = idx[batch_size:] if batch_size < len(idx) else np.array([], dtype=int)
            temp_data = temp_data[remaining]

        self.W              = node_list.astype(np.float32)
        self.C              = edge_list.astype(np.int64)
        self.E              = np.zeros(len(self.W), dtype=np.float32)
        self.is_initialized = len(self.W) > 0
        self._invalidate_cache()

    def batch_learning(self, X: np.ndarray):
        X_t   = X[:, :self.feature_number].astype(np.float32)
        B, N  = len(X_t), len(self.W)

        x2   = (X_t * X_t).sum(axis=1, keepdims=True)      # (B,1)
        y2   = (self.W * self.W).sum(axis=1)               # (N,)
        dot  = X_t @ self.W.T                              # (B,N)
        dist = np.clip(x2 + y2 - 2.0 * dot, 0.0, None)
        dist = np.sqrt(dist + self.eps)                    # (B,N)

        tmp        = dist.copy()
        s1         = np.argmin(tmp, axis=1)
        tmp[np.arange(B), s1] = np.inf
        s2         = np.argmin(tmp, axis=1)

        self._last_X  = X_t
        sorted_order  = np.argsort(s1)
        sorted_nodes  = s1[sorted_order]
        node_ids      = np.arange(N)
        starts        = np.searchsorted(sorted_nodes, node_ids)
        ends          = np.searchsorted(sorted_nodes, node_ids, side='right')
        starts_l, ends_l = starts.tolist(), ends.tolist()
        for nid in range(N):
            s, e = starts_l[nid], ends_l[nid]
            if s < e:
                self.node_point_map[nid] = sorted_order[s:e]

        adj       = self._get_adj()
        i_adj     = np.eye(N, dtype=np.float32)
        s1_onehot = i_adj[s1]                              # (B,N)

        self.E         += (s1_onehot * dist).sum(0) * self.alpha
        self.Delta_W_1 += ((s1_onehot.T @ X_t)
                           - (self.W.T * s1_onehot.sum(0)).T) * self.alpha

        s1_adj          = adj[s1]
        self.Delta_W_2 += ((s1_adj.T @ X_t)
                           - (self.W.T * s1_adj.sum(0)).T) * self.beta

        self.A_1 += s1_onehot.sum(0)
        self.A_2 += s1_adj.sum(0)

        conn = np.zeros_like(self.S)
        conn[s1, s2] = 1.0
        conn[s2, s1] = 1.0
        t    = s1_onehot + i_adj[s2]
        conn *= (t.T @ t)
        np.clip(conn, None, 1.0, out=conn)
        self.S += conn

    def update_network(self):
        safe_A1 = self.A_1 + self.eps
        safe_A2 = self.A_2 + self.eps
        self.W += (self.Delta_W_1.T / safe_A1).T + \
                  (self.Delta_W_2.T / safe_A2).T

        self.C  = np.argwhere(self.S > 0).astype(np.int64)
        self.remove_isolated_nodes()
        self.E *= self.delta

        if random.random() > 0.9:
            self.remove_non_activated_nodes()

        self._invalidate_cache()

    def remove_isolated_nodes(self):
        if len(self.C) == 0:
            return
        n      = len(self.W)
        degree = np.zeros(n, dtype=np.float32)
        np.add.at(degree, self.C[:, 0], 1.0)
        np.add.at(degree, self.C[:, 1], 1.0)
        isolated = np.argwhere(degree == 0).squeeze(1)
        if len(isolated) == 0:
            return
        keep = np.ones(n, dtype=bool)
        keep[isolated] = False
        self._delete_nodes(keep)

    def remove_non_activated_nodes(self):
        non_act = np.argwhere(self.A_1 == 0).squeeze(1)
        if len(non_act) == 0:
            return
        keep = np.ones(len(self.W), dtype=bool)
        keep[non_act] = False
        self._delete_nodes(keep)

    def _delete_nodes(self, keep_mask: np.ndarray):
        keep_idx = np.argwhere(keep_mask).squeeze(1)
        remap    = np.full((len(self.W),), -1, dtype=np.int64)
        remap[keep_idx] = np.arange(len(keep_idx))

        new_map = {}
        for old_idx, pt_indices in self.node_point_map.items():
            if old_idx < len(keep_mask) and keep_mask[old_idx]:
                new_map[int(remap[old_idx])] = pt_indices
        self.node_point_map = new_map

        if len(self.C) > 0:
            valid  = keep_mask[self.C[:, 0]] & keep_mask[self.C[:, 1]]
            self.C = remap[self.C[valid]]
        else:
            self.C = np.empty((0, 2), dtype=np.int64)

        self.W         = self.W[keep_mask]
        self.E         = self.E[keep_mask]
        self.A_1       = self.A_1[keep_mask]
        self.A_2       = self.A_2[keep_mask]
        self.S         = self.S[keep_mask][:, keep_mask]
        self.Delta_W_1 = self.Delta_W_1[keep_mask]
        self.Delta_W_2 = self.Delta_W_2[keep_mask]

    def add_new_node(self):
        threshold = np.quantile(self.E, 0.85)
        g         = int((self.E > threshold).sum())

        for _ in range(g):
            if len(self.W) >= self.M:
                return
            q1 = int(np.argmax(self.E))
            if self.E[q1] <= 0:
                return

            mask0 = self.C[:, 0] == q1
            mask1 = self.C[:, 1] == q1
            nb    = np.unique(np.concatenate([self.C[mask0, 1],
                                              self.C[mask1, 0]]))
            if len(nb) == 0:
                return

            q2 = int(nb[np.argmax(self.E[nb])])
            if self.E[q2] <= 0:
                return

            q3     = len(self.W)
            new_w  = ((self.W[q1] + self.W[q2]) * 0.5).reshape(1, -1)
            self.W = np.concatenate([self.W, new_w])
            self.E = np.concatenate([self.E, np.zeros(1, dtype=np.float32)])

            self.E[q1] *= self.rho
            self.E[q2] *= self.rho
            self.E[q3]  = (self.E[q1] + self.E[q2]) * 0.5

            keep = ~(((self.C[:, 0] == q1) & (self.C[:, 1] == q2)) |
                     ((self.C[:, 0] == q2) & (self.C[:, 1] == q1)))
            self.C = np.concatenate([self.C[keep],
                                      np.array([[q1, q3], [q2, q3]],
                                               dtype=np.int64)])

            n     = len(self.W)
            new_S = np.zeros((n, n), dtype=np.float32)
            new_S[:n - 1, :n - 1] = self.S
            self.S = new_S
            self.S[q1, q2] = self.S[q2, q1] = 0
            self.S[q1, q3] = self.S[q3, q1] = 1
            self.S[q2, q3] = self.S[q3, q2] = 1

            z1 = np.ones(1, dtype=np.float32)
            z0 = np.zeros((1, self.feature_number), dtype=np.float32)
            self.A_1       = np.concatenate([self.A_1, z1])
            self.A_2       = np.concatenate([self.A_2, z1])
            self.Delta_W_1 = np.concatenate([self.Delta_W_1, z0])
            self.Delta_W_2 = np.concatenate([self.Delta_W_2, z0])

    def cut_edge(self):
        self.remove_non_activated_nodes()
        mask = self.S > 0
        if mask.sum() == 0:
            return
        threshold = np.quantile(self.S[mask], 0.15)
        temp      = self.S.copy()
        temp[temp < threshold] = 0
        self.C    = np.argwhere(temp > 0).astype(np.int64)
        self.remove_isolated_nodes()
        self._invalidate_cache()

    def get_connected_components(self) -> list[np.ndarray]:
        n      = len(self.W)
        parent = np.arange(n, dtype=np.int32)

        def find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        if len(self.C) > 0:
            for i, j in self.C:
                ri, rj = find(int(i)), find(int(j))
                if ri != rj:
                    parent[ri] = rj

        groups: dict[int, list[int]] = defaultdict(list)
        for node in range(n):
            groups[find(node)].append(node)

        return [np.array(members, dtype=np.int64) for members in groups.values()]

    def _planarity_pca(self, node_indices: np.ndarray
                       ) -> tuple[float, np.ndarray, np.ndarray]:
        pts      = self.W[node_indices]
        centroid = pts.mean(axis=0)
        if len(pts) < 3:
            return 0.0, self.normal_axis_np.copy(), centroid

        centered         = pts - centroid
        cov              = np.cov(centered.T)           # (3,3)
        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvals          = np.maximum(eigvals, 0.0)
        total            = eigvals.sum()

        if total < 1e-9:
            return 0.0, eigvecs[:, 0].astype(np.float32), centroid

        planarity = float(eigvals[0] / total)
        normal    = eigvecs[:, 0].astype(np.float32)
        return planarity, normal, centroid

    def _compute_node_normals_cpu(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Weighted covariance per node di CPU dengan multithreading.

        Batch eigendecomposition di-paralel-kan per chunk node.
        """
        if self._cache_normals_np is not None:
            return self._cache_normals_np, self._cache_planarity_np

        N   = len(self.W)
        W   = self.W                                            # (N, 3)
        adj = self._get_adj()                                   # (N, N)

        # ── Hitung centroid lokal tiap node via adj ──────────────────────
        adj_rhop = adj.copy()
        for _ in range(self.node_normal_radius - 1):
            adj_rhop = np.clip(adj_rhop @ adj, None, 1.0)
            np.fill_diagonal(adj_rhop, 0.0)

        degree  = adj_rhop.sum(axis=1, keepdims=True)
        degree  = np.maximum(degree, 1.0)                      # (N, 1)
        nb_sum  = adj_rhop @ W                                  # (N, 3)
        mu      = (W + nb_sum) / (1.0 + degree)                 # (N, 3)

        # ── Weighted covariance batch ────────────────────────────────────
        eye     = np.eye(N, dtype=np.float32)
        weights = (adj_rhop + eye)[:, :, None]                  # (N, N, 1)
        diff    = W[None, :, :] - mu[:, None, :]                # (N, N, 3): diff[i,j] = W[j]-mu[i]
        diff_w  = diff * weights                                # (N, N, 3)

        # Covariance[i] = diff_w[i].T @ diff[i]  →  (N, 3, 3)
        cov = np.einsum('ijk,ijl->ikl', diff_w, diff)           # (N, 3, 3)

        # ── Batch eigendecomposition paralel per chunk ───────────────────
        chunk = max(1, N // self.num_workers)
        ranges = [(s, min(s + chunk, N)) for s in range(0, N, chunk)]

        def _eig_block(rng):
            s, e = rng
            eigvals, eigvecs = np.linalg.eigh(cov[s:e])
            eigvals = np.clip(eigvals, 0.0, None)
            return eigvals, eigvecs

        results = list(self._executor.map(_eig_block, ranges))
        all_eigvals = np.concatenate([r[0] for r in results], axis=0)
        all_eigvecs = np.concatenate([r[1] for r in results], axis=0)

        total = np.clip(all_eigvals.sum(axis=1), 1e-9, None)    # (N,)

        normals   = all_eigvecs[:, :, 0]                         # (N, 3)
        planarity = all_eigvals[:, 0] / total                   # (N,)

        norms   = np.linalg.norm(normals, axis=1, keepdims=True)
        norms   = np.maximum(norms, 1e-6)
        normals = normals / norms

        self._cache_normals_np   = normals.astype(np.float32)
        self._cache_planarity_np = planarity.astype(np.float32)
        return self._cache_normals_np, self._cache_planarity_np

    def detect_flat_clusters(self) -> list[FlatCluster]:
        if self._cache_flat_clusters is not None:
            return self._cache_flat_clusters

        components = self.get_connected_components()

        # Filter by size dulu sebelum paralelisasi
        candidate_comps = [c for c in components
                            if len(c) >= self.min_cluster_size]
        if not candidate_comps:
            self._cache_flat_clusters = []
            return self._cache_flat_clusters

        def _process_comp(comp: np.ndarray):
            planarity, normal, centroid = self._planarity_pca(comp)
            if planarity >= self.planarity_threshold:
                return None
            cos_angle = abs(float(normal @ self.normal_axis_np))
            if cos_angle < self.cos_threshold:
                return None
            return FlatCluster(
                node_indices=comp,
                normal=normal,
                planarity=planarity,
                centroid=centroid.astype(np.float32),
            )

        results = list(self._executor.map(_process_comp, candidate_comps))
        flat_clusters = [fc for fc in results if fc is not None]

        flat_clusters.sort(key=lambda fc: fc.planarity)
        self._cache_flat_clusters = flat_clusters
        return flat_clusters

    def get_node_normals(self) -> list[NodeNormal]:
        if self._cache_node_normals is not None:
            return self._cache_node_normals

        if self.W is None or len(self.W) == 0:
            return []

        n = len(self.W)

        normals_np, planarity_np = self._compute_node_normals_cpu()

        cos_angles = np.abs(normals_np @ self.normal_axis_np)   # (N,)
        is_perp    = cos_angles >= self.cos_threshold           # (N,) bool

        flat_clusters = self.detect_flat_clusters()
        flat_node_set = set(
            int(idx)
            for fc in flat_clusters
            for idx in fc.node_indices
        )

        node_normals = [
            NodeNormal(
                node_idx  = i,
                position  = self.W[i],
                normal    = normals_np[i],
                planarity = float(planarity_np[i]),
                is_flat   = (i in flat_node_set) or bool(is_perp[i]),
            )
            for i in range(n)
        ]

        self._cache_node_normals = node_normals
        return node_normals

    def _collect_points(self, node_index_set: set[int]) -> np.ndarray:
        lists = [self.node_point_map[nid]
                 for nid in node_index_set if nid in self.node_point_map]
        if not lists:
            return np.empty((0, 3), dtype=np.float32)
        all_idx = np.unique(np.concatenate(lists))
        return self._last_X[all_idx, :3].astype(np.float32)

    def get_perpendicular_points_numpy(self) -> tuple[np.ndarray, list[NodeNormal]]:
        if self._last_X is None or not self.node_point_map:
            return np.empty((0, 3), dtype=np.float32), []

        node_normals = self.get_node_normals()
        kept_nns     = [nn for nn in node_normals if nn.is_flat]
        if not kept_nns:
            return np.empty((0, 3), dtype=np.float32), []

        pts = self._collect_points({nn.node_idx for nn in kept_nns})
        return pts, kept_nns

    def get_flat_points_numpy(self) -> np.ndarray:
        if self._last_X is None or not self.node_point_map:
            return np.empty((0, 3), dtype=np.float32)
        flat_clusters = self.detect_flat_clusters()
        if not flat_clusters:
            return np.empty((0, 3), dtype=np.float32)
        flat_node_indices = {
            int(idx)
            for fc in flat_clusters
            for idx in fc.node_indices
        }
        return self._collect_points(flat_node_indices)

    def step(self, data: np.ndarray):
        self.reset_batch()
        self.batch_learning(data)
        self.update_network()
        self.add_new_node()

    def get_numpy(self) -> tuple[np.ndarray, np.ndarray]:
        return self.W, self.C
