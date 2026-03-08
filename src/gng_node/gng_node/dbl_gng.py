import random
from collections import defaultdict
from dataclasses import dataclass

import numpy as np
import torch


@dataclass
class FlatCluster:
    node_indices: torch.Tensor
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


class DBL_GNG:
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
        device: str | None = None,
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

        self.device = torch.device(
            device if device else ("cuda" if torch.cuda.is_available() else "cpu")
        )

        _ax = np.array(normal_axis if normal_axis is not None
                       else [0.0, 0.0, 1.0], dtype=np.float32)
        _ax /= np.linalg.norm(_ax)
        self.normal_axis_np = _ax
        self.normal_axis_t  = torch.tensor(_ax, device=self.device)

        # Precompute sekali
        self.cos_threshold    = float(np.cos(np.deg2rad(max_normal_angle_deg)))
        self.max_normal_angle_deg = max_normal_angle_deg

        self.W:         torch.Tensor | None = None
        self.C:         torch.Tensor | None = None
        self.E:         torch.Tensor | None = None
        self.S:         torch.Tensor | None = None
        self.A_1:       torch.Tensor | None = None
        self.A_2:       torch.Tensor | None = None
        self.Delta_W_1: torch.Tensor | None = None
        self.Delta_W_2: torch.Tensor | None = None

        self.is_initialized = False
        self.node_point_map: dict[int, torch.Tensor] = {}
        self._last_X: torch.Tensor | None = None

        # Cache
        self._cache_adj:           torch.Tensor | None = None
        self._cache_flat_clusters: list[FlatCluster] | None = None
        self._cache_node_normals:  list[NodeNormal]  | None = None
        self._cache_normals_t:     torch.Tensor | None = None
        self._cache_planarity_t:   torch.Tensor | None = None

    def _t(self, arr: np.ndarray, dtype=torch.float32) -> torch.Tensor:
        return torch.tensor(arr, dtype=dtype, device=self.device)

    def get_outlier_points_numpy(self) -> np.ndarray:
        """
        Mengumpulkan point asli yang BUKAN termasuk plane/perpendicular.
        
        Outlier = point yang:
        - Tidak termasuk dalam flat cluster, DAN
        - Tidak termasuk dalam node yang memiliki is_flat=True
        
        Returns:
            np.ndarray: Outlier points dengan shape (N, 3) dtype float32
        """
        if self._last_X is None or not self.node_point_map:
            return np.empty((0, 3), dtype=np.float32)

        # Dapatkan semua node yang merupakan bagian dari plane/perpendicular
        node_normals = self.get_node_normals()
        flat_node_set = {nn.node_idx for nn in node_normals if nn.is_flat}

        # Kumpulkan semua point indices yang termasuk dalam flat/perpendicular nodes
        flat_point_indices = set()
        for nid in flat_node_set:
            if nid in self.node_point_map:
                pt_indices = self.node_point_map[nid]
                flat_point_indices.update(pt_indices.cpu().numpy().tolist())

        # Outlier = semua point KECUALI yang termasuk flat_point_indices
        all_point_indices = set(range(len(self._last_X)))
        outlier_indices = all_point_indices - flat_point_indices

        if not outlier_indices:
            return np.empty((0, 3), dtype=np.float32)

        outlier_idx_tensor = torch.tensor(
            sorted(list(outlier_indices)),
            dtype=torch.long,
            device=self.device
        )
        outlier_points = self._last_X[outlier_idx_tensor, :3].cpu().numpy()
        
        return outlier_points.astype(np.float32)

    def _invalidate_cache(self):
        self._cache_adj           = None
        self._cache_flat_clusters = None
        self._cache_node_normals  = None
        self._cache_normals_t     = None
        self._cache_planarity_t   = None

    def _get_adj(self) -> torch.Tensor:
        if self._cache_adj is not None:
            return self._cache_adj
        n   = len(self.W)
        adj = torch.zeros((n, n), dtype=torch.float32, device=self.device)
        if len(self.C) > 0:
            adj[self.C[:, 0], self.C[:, 1]] = 1.0
            adj[self.C[:, 1], self.C[:, 0]] = 1.0
        self._cache_adj = adj
        return adj

    def reset_batch(self):
        n = len(self.W)
        self.Delta_W_1  = torch.zeros_like(self.W)
        self.Delta_W_2  = torch.zeros_like(self.W)
        self.A_1        = torch.zeros(n, dtype=torch.float32, device=self.device)
        self.A_2        = torch.zeros(n, dtype=torch.float32, device=self.device)
        self.S          = torch.zeros((n, n), dtype=torch.float32, device=self.device)
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

        self.W              = self._t(node_list)
        self.C              = torch.tensor(edge_list, dtype=torch.long, device=self.device)
        self.E              = torch.zeros(len(self.W), dtype=torch.float32, device=self.device)
        self.is_initialized = len(self.W) > 0
        self._invalidate_cache()

    def batch_learning(self, X: np.ndarray):
        X_t   = self._t(X[:, :self.feature_number])
        B, N  = len(X_t), len(self.W)
        b_idx = torch.arange(B, device=self.device)

        x2   = (X_t * X_t).sum(dim=1, keepdim=True)      # (B,1)
        y2   = (self.W * self.W).sum(dim=1)               # (N,)
        dot  = torch.matmul(X_t, self.W.T)                # (B,N)
        dist = torch.clamp(x2 + y2 - 2.0 * dot, min=0.0)
        dist = torch.sqrt(dist + self.eps)                 # (B,N)

        tmp            = dist.clone()
        s1             = tmp.argmin(dim=1)
        tmp[b_idx, s1] = 1e9
        s2             = tmp.argmin(dim=1)

        self._last_X  = X_t
        sorted_order  = torch.argsort(s1)
        sorted_nodes  = s1[sorted_order]
        node_ids      = torch.arange(N, device=self.device)
        starts        = torch.searchsorted(sorted_nodes, node_ids)
        ends          = torch.searchsorted(sorted_nodes, node_ids, right=True)
        starts_l, ends_l = starts.tolist(), ends.tolist()
        for nid in range(N):
            s, e = starts_l[nid], ends_l[nid]
            if s < e:
                self.node_point_map[nid] = sorted_order[s:e]

        adj       = self._get_adj()
        i_adj     = torch.eye(N, dtype=torch.float32, device=self.device)
        s1_onehot = i_adj[s1]                              # (B,N)

        self.E         += (s1_onehot * dist).sum(0) * self.alpha
        self.Delta_W_1 += (torch.matmul(s1_onehot.T, X_t)
                           - (self.W.T * s1_onehot.sum(0)).T) * self.alpha

        s1_adj          = adj[s1]
        self.Delta_W_2 += (torch.matmul(s1_adj.T, X_t)
                           - (self.W.T * s1_adj.sum(0)).T) * self.beta

        self.A_1 += s1_onehot.sum(0)
        self.A_2 += s1_adj.sum(0)

        conn = torch.zeros_like(self.S)
        conn[s1, s2] = 1.0
        conn[s2, s1] = 1.0
        t    = s1_onehot + i_adj[s2]
        conn *= torch.matmul(t.T, t)
        conn.clamp_(max=1.0)
        self.S += conn

    def update_network(self):
        safe_A1 = self.A_1 + self.eps
        safe_A2 = self.A_2 + self.eps
        self.W += (self.Delta_W_1.T / safe_A1).T + \
                  (self.Delta_W_2.T / safe_A2).T

        self.C  = self.S.nonzero(as_tuple=False)
        self.remove_isolated_nodes()
        self.E *= self.delta

        if random.random() > 0.9:
            self.remove_non_activated_nodes()

        self._invalidate_cache()

    def remove_isolated_nodes(self):
        if len(self.C) == 0:
            return
        n      = len(self.W)
        degree = torch.zeros(n, device=self.device)
        degree.scatter_add_(0, self.C[:, 0], torch.ones(len(self.C), device=self.device))
        degree.scatter_add_(0, self.C[:, 1], torch.ones(len(self.C), device=self.device))
        isolated = (degree == 0).nonzero(as_tuple=False).squeeze(1)
        if len(isolated) == 0:
            return
        keep = torch.ones(n, dtype=torch.bool, device=self.device)
        keep[isolated] = False
        self._delete_nodes(keep)

    def remove_non_activated_nodes(self):
        non_act = (self.A_1 == 0).nonzero(as_tuple=False).squeeze(1)
        if len(non_act) == 0:
            return
        keep = torch.ones(len(self.W), dtype=torch.bool, device=self.device)
        keep[non_act] = False
        self._delete_nodes(keep)

    def _delete_nodes(self, keep_mask: torch.Tensor):
        keep_idx = keep_mask.nonzero(as_tuple=False).squeeze(1)
        remap    = torch.full((len(self.W),), -1, dtype=torch.long, device=self.device)
        remap[keep_idx] = torch.arange(len(keep_idx), device=self.device)

        new_map = {}
        for old_idx, pt_indices in self.node_point_map.items():
            if old_idx < len(keep_mask) and keep_mask[old_idx]:
                new_map[int(remap[old_idx].item())] = pt_indices
        self.node_point_map = new_map

        if len(self.C) > 0:
            valid  = keep_mask[self.C[:, 0]] & keep_mask[self.C[:, 1]]
            self.C = remap[self.C[valid]]
        else:
            self.C = torch.empty((0, 2), dtype=torch.long, device=self.device)

        self.W         = self.W[keep_mask]
        self.E         = self.E[keep_mask]
        self.A_1       = self.A_1[keep_mask]
        self.A_2       = self.A_2[keep_mask]
        self.S         = self.S[keep_mask][:, keep_mask]
        self.Delta_W_1 = self.Delta_W_1[keep_mask]
        self.Delta_W_2 = self.Delta_W_2[keep_mask]

    def add_new_node(self):
        threshold = torch.quantile(self.E, 0.85)
        g         = int((self.E > threshold).sum().item())

        for _ in range(g):
            if len(self.W) >= self.M:
                return
            q1 = int(torch.argmax(self.E).item())
            if self.E[q1] <= 0:
                return

            mask0 = self.C[:, 0] == q1
            mask1 = self.C[:, 1] == q1
            nb    = torch.cat([self.C[mask0, 1], self.C[mask1, 0]]).unique()
            if len(nb) == 0:
                return

            q2 = int(nb[torch.argmax(self.E[nb])].item())
            if self.E[q2] <= 0:
                return

            q3     = len(self.W)
            new_w  = ((self.W[q1] + self.W[q2]) * 0.5).unsqueeze(0)
            self.W = torch.cat([self.W, new_w])
            self.E = torch.cat([self.E, torch.zeros(1, device=self.device)])

            self.E[q1] *= self.rho
            self.E[q2] *= self.rho
            self.E[q3]  = (self.E[q1] + self.E[q2]) * 0.5

            keep = ~(((self.C[:, 0] == q1) & (self.C[:, 1] == q2)) |
                     ((self.C[:, 0] == q2) & (self.C[:, 1] == q1)))
            self.C = torch.cat([self.C[keep],
                                 torch.tensor([[q1, q3], [q2, q3]],
                                              dtype=torch.long, device=self.device)])

            n     = len(self.W)
            new_S = torch.zeros((n, n), dtype=torch.float32, device=self.device)
            new_S[:n - 1, :n - 1] = self.S
            self.S = new_S
            self.S[q1, q2] = self.S[q2, q1] = 0
            self.S[q1, q3] = self.S[q3, q1] = 1
            self.S[q2, q3] = self.S[q3, q2] = 1

            z1 = torch.ones(1, device=self.device)
            z0 = torch.zeros((1, self.feature_number), dtype=torch.float32, device=self.device)
            self.A_1       = torch.cat([self.A_1, z1])
            self.A_2       = torch.cat([self.A_2, z1])
            self.Delta_W_1 = torch.cat([self.Delta_W_1, z0])
            self.Delta_W_2 = torch.cat([self.Delta_W_2, z0])

    def cut_edge(self):
        self.remove_non_activated_nodes()
        mask = self.S > 0
        if mask.sum() == 0:
            return
        threshold = torch.quantile(self.S[mask], 0.15)
        temp      = self.S.clone()
        temp[temp < threshold] = 0
        self.C    = temp.nonzero(as_tuple=False)
        self.remove_isolated_nodes()
        self._invalidate_cache()

    def get_connected_components(self) -> list[torch.Tensor]:
        n      = len(self.W)
        parent = np.arange(n, dtype=np.int32)

        def find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        if len(self.C) > 0:
            for i, j in self.C.cpu().numpy():
                ri, rj = find(int(i)), find(int(j))
                if ri != rj:
                    parent[ri] = rj

        groups: dict[int, list[int]] = defaultdict(list)
        for node in range(n):
            groups[find(node)].append(node)

        return [
            torch.tensor(members, dtype=torch.long, device=self.device)
            for members in groups.values()
        ]

    def _planarity_pca(self, node_indices: torch.Tensor
                       ) -> tuple[float, np.ndarray, np.ndarray]:
        pts      = self.W[node_indices].cpu().numpy()
        centroid = pts.mean(axis=0)
        if len(pts) < 3:
            return 0.0, self.normal_axis_np.copy(), centroid

        centered         = pts - centroid
        cov              = np.cov(centered.T)           # (3,3)
        # eigh: eigenvalue ASCENDING — indeks 0 = terkecil = normal bidang
        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvals          = np.maximum(eigvals, 0.0)
        total            = eigvals.sum()

        if total < 1e-9:
            return 0.0, eigvecs[:, 0].astype(np.float32), centroid

        planarity = float(eigvals[0] / total)           # eigenvalue TERKECIL / total
        normal    = eigvecs[:, 0].astype(np.float32)    # eigenvector eigenvalue terkecil
        return planarity, normal, centroid

    def _compute_node_normals_gpu(self) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Weighted covariance per node di GPU.

        Untuk tiap node i:
          mu_i  = rata-rata posisi tetangga (via adj propagation)
          C_i   = sum_j adj[i,j] * (W[j]-mu_i) @ (W[j]-mu_i)^T   → (3,3)
          normal_i = eigenvector eigenvalue TERKECIL dari C_i

        Returns: normals (N,3), planarity (N,) — GPU tensors
        """
        if self._cache_normals_t is not None:
            return self._cache_normals_t, self._cache_planarity_t

        N   = len(self.W)
        W   = self.W                                            # (N, 3)
        adj = self._get_adj()                                   # (N, N)

        # ── Hitung centroid lokal tiap node via adj ──────────────────────
        # mu_i = ( W_i + sum_j adj[i,j]*W_j ) / ( 1 + degree_i )
        # dengan adj sudah mencakup r-hop (propagasi)
        adj_rhop = adj.clone()
        for _ in range(self.node_normal_radius - 1):
            # Perluas radius: adj^r — boolean (clamp agar tetap 0/1)
            adj_rhop = torch.clamp(torch.matmul(adj_rhop, adj), max=1.0)
            adj_rhop.fill_diagonal_(0.0)    # jangan hitung diri sendiri berkali-kali

        degree  = adj_rhop.sum(dim=1, keepdim=True).clamp(min=1.0)  # (N, 1)
        nb_sum  = torch.matmul(adj_rhop, W)                           # (N, 3)
        # centroid per node = (W_i + sum neighbor) / (1 + degree)
        mu      = (W + nb_sum) / (1.0 + degree)                       # (N, 3)

        # ── Weighted covariance batch ────────────────────────────────────
        # diff[i, j] = W[j] - mu[i]   → shape (N, N, 3)
        # Perhatian: axis yang benar:
        #   W.unsqueeze(0)    → (1, N, 3) broadcast ke (N, N, 3): W[j]
        #   mu.unsqueeze(1)   → (N, 1, 3) broadcast ke (N, N, 3): mu[i]
        diff = W.unsqueeze(0) - mu.unsqueeze(1)     # (N, N, 3): diff[i,j] = W[j]-mu[i]

        # Bobot: adj_rhop[i,j] + diri sendiri (diagonal = 1)
        eye     = torch.eye(N, dtype=torch.float32, device=self.device)
        weights = (adj_rhop + eye).unsqueeze(2)     # (N, N, 1)

        # weighted diff: (N, N, 3)
        diff_w  = diff * weights

        # Covariance[i] = diff_w[i].T @ diff[i]  →  (N, 3, 3)
        # diff_w[i]: (N, 3), diff[i]: (N, 3)
        # bmm: (N, 3, N) @ (N, N, 3) = (N, 3, 3)
        cov = torch.bmm(
            diff_w.permute(0, 2, 1),   # (N, 3, N)
            diff                        # (N, N, 3)
        )                               # → (N, 3, 3) ✓

        # ── Batch eigendecomposition ─────────────────────────────────────
        # torch.linalg.eigh: eigenvalue ASCENDING — indeks 0 = terkecil
        try:
            eigvals, eigvecs = torch.linalg.eigh(cov)
        except Exception:
            eigvals, eigvecs = torch.linalg.eigh(cov.cpu())
            eigvals  = eigvals.to(self.device)
            eigvecs  = eigvecs.to(self.device)

        eigvals = eigvals.clamp(min=0.0)                        # (N, 3)
        total   = eigvals.sum(dim=1).clamp(min=1e-9)            # (N,)

        # normal = eigvec eigenvalue TERKECIL = kolom 0 dari eigh output
        normals   = eigvecs[:, :, 0]                            # (N, 3)
        planarity = eigvals[:, 0] / total                       # (N,) — ratio terkecil/total

        # Normalisasi
        norms   = normals.norm(dim=1, keepdim=True).clamp(min=1e-6)
        normals = normals / norms

        self._cache_normals_t   = normals
        self._cache_planarity_t = planarity
        return normals, planarity

    def detect_flat_clusters(self) -> list[FlatCluster]:
        if self._cache_flat_clusters is not None:
            return self._cache_flat_clusters

        flat_clusters = []
        for comp in self.get_connected_components():
            if len(comp) < self.min_cluster_size:
                continue
            planarity, normal, centroid = self._planarity_pca(comp)
            if planarity >= self.planarity_threshold:
                continue
            cos_angle = abs(float(normal @ self.normal_axis_np))
            if cos_angle < self.cos_threshold:
                continue
            flat_clusters.append(FlatCluster(
                node_indices=comp,
                normal=normal,
                planarity=planarity,
                centroid=centroid.astype(np.float32),
            ))

        flat_clusters.sort(key=lambda fc: fc.planarity)
        self._cache_flat_clusters = flat_clusters
        return flat_clusters

    def get_node_normals(self) -> list[NodeNormal]:
        if self._cache_node_normals is not None:
            return self._cache_node_normals

        if self.W is None or len(self.W) == 0:
            return []

        n = len(self.W)

        normals_t, planarity_t = self._compute_node_normals_gpu()

        cos_angles = (normals_t @ self.normal_axis_t).abs()    # (N,)
        is_perp    = cos_angles >= self.cos_threshold           # (N,) bool

        flat_clusters = self.detect_flat_clusters()
        flat_node_set = set(
            int(idx.item())
            for fc in flat_clusters
            for idx in fc.node_indices
        )

        # Transfer ke CPU sekali saja
        normals_np   = normals_t.cpu().numpy()
        planarity_np = planarity_t.cpu().numpy()
        is_perp_np   = is_perp.cpu().numpy()
        W_np         = self.W.cpu().numpy()

        node_normals = [
            NodeNormal(
                node_idx  = i,
                position  = W_np[i],
                normal    = normals_np[i],
                planarity = float(planarity_np[i]),
                is_flat   = (i in flat_node_set) or bool(is_perp_np[i]),
            )
            for i in range(n)
        ]

        self._cache_node_normals = node_normals
        return node_normals

    # ================================================================== #
    #  ⑥ Helper: kumpulkan point asli
    # ================================================================== #
    def _collect_points(self, node_index_set: set[int]) -> np.ndarray:
        lists = [self.node_point_map[nid]
                 for nid in node_index_set if nid in self.node_point_map]
        if not lists:
            return np.empty((0, 3), dtype=np.float32)
        all_idx = torch.cat(lists).unique()
        return self._last_X[all_idx, :3].cpu().numpy().astype(np.float32)

    # ================================================================== #
    #  ⑦ Point asli tegak lurus kamera
    # ================================================================== #
    def get_perpendicular_points_numpy(self) -> tuple[np.ndarray, list[NodeNormal]]:
        if self._last_X is None or not self.node_point_map:
            return np.empty((0, 3), dtype=np.float32), []

        node_normals = self.get_node_normals()
        kept_nns     = [nn for nn in node_normals if nn.is_flat]
        if not kept_nns:
            return np.empty((0, 3), dtype=np.float32), []

        pts = self._collect_points({nn.node_idx for nn in kept_nns})
        return pts, kept_nns

    # ================================================================== #
    #  ⑧ Point asli flat cluster
    # ================================================================== #
    def get_flat_points_numpy(self) -> np.ndarray:
        if self._last_X is None or not self.node_point_map:
            return np.empty((0, 3), dtype=np.float32)
        flat_clusters = self.detect_flat_clusters()
        if not flat_clusters:
            return np.empty((0, 3), dtype=np.float32)
        flat_node_indices = {
            int(idx.item())
            for fc in flat_clusters
            for idx in fc.node_indices
        }
        return self._collect_points(flat_node_indices)

    # ================================================================== #
    #  Convenience
    # ================================================================== #
    def step(self, data: np.ndarray):
        self.reset_batch()
        self.batch_learning(data)
        self.update_network()
        self.add_new_node()

    def get_numpy(self) -> tuple[np.ndarray, np.ndarray]:
        return self.W.cpu().numpy(), self.C.cpu().numpy()