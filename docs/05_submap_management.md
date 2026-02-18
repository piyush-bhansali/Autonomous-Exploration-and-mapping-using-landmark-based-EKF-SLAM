# Submap Management and Global Map Construction

## Table of Contents
1. [Introduction](#1-introduction)
2. [Submap Creation](#2-submap-creation)
3. [Feature-Based SVD Alignment](#3-feature-based-svd-alignment)
4. [SVD Singular-Value Covariance](#4-svd-singular-value-covariance)
5. [Pose Correction Feedback](#5-pose-correction-feedback)
6. [Global Map Maintenance](#6-global-map-maintenance)
7. [Implementation Reference](#7-implementation-reference)

---

## 1. Introduction

The EKF operates in a local coordinate frame and accumulates drift over long trajectories. A separate submap-stitching layer corrects this drift by aligning accumulated local point clouds against the growing global map using shared landmark correspondences.

Every $N$ scans, the local point cloud is declared a complete submap. The submap is registered to the global map using feature-based SVD alignment: wall segments shared between the new submap and the accumulated global wall registry are used as correspondences, and a one-shot SVD rigid body solve yields the correction transform. The resulting pose correction is fed back into the EKF as a direct pose observation, propagating the correction to all correlated landmarks.

The submap layer serves two purposes:
1. It builds and maintains a metric point cloud map of the environment.
2. It provides periodic global consistency corrections to the EKF state.

If alignment fails (degenerate geometry, insufficient shared landmarks), the submap is integrated using the EKF pose without correction; the EKF covariance naturally reflects the degraded accuracy.

---

## 2. Submap Creation

### 2.1 Trigger

A submap is finalised when the scan count reaches a fixed threshold:

$$
n_{\mathrm{scans}} \geq N_{\mathrm{threshold}}, \qquad N_{\mathrm{threshold}} = 50.
$$

At 1 Hz scan rate this corresponds to approximately 50 seconds of travel. This duration is short enough that odometric drift within a single submap is small (typically < 0.05 m).

### 2.2 Point Accumulation

Scan points are expressed in the robot body frame at the time of acquisition. Before accumulation each scan is transformed into the submap-local frame, which is anchored at the robot pose when the submap was initialised, $(x_0, y_0, \theta_0)$.

Let the current robot pose be $(x_r, y_r, \theta_r)$ in the map frame. The relative pose of the current scan with respect to the submap origin is

$$
\Delta x = x_r - x_0, \quad \Delta y = y_r - y_0, \quad \Delta\theta = \theta_r - \theta_0.
$$

Each scan point $\mathbf{p}^{\mathrm{robot}}$ is transformed to the submap frame by the rotation $\mathbf{R}(\theta_r)$ followed by translation, then expressed relative to the submap origin. In practice the implementation passes a 4×4 transformation matrix $\mathbf{T}_{\mathrm{submap}}$ computed from the EKF state at scan time.

All accumulated points are stored in the submap-local frame. When the submap is finalised the full transformation $\mathbf{T}_{\mathrm{submap}}^{\mathrm{map}}$ (start pose in the map frame) is used to project the submap into the global map frame.

### 2.3 Voxel Downsampling

Before alignment each submap is voxel-downsampled at $v = 0.05$ m. This reduces the point count, removes redundant measurements from overlapping scans, and gives uniform spatial density. A submap with fewer than 50 points after downsampling is discarded.

---

## 3. Feature-Based SVD Alignment

### 3.1 Overview

The alignment exploits the wall landmarks already maintained by the EKF. When a submap is finalised, the `FeatureMap` associated with the current submap holds wall endpoints in the (approximate) map frame. The `SubmapStitcher` maintains a `global_walls` registry — a dictionary keyed by landmark ID containing the globally-aligned Hessian parameters and endpoints accumulated from all previous submaps.

Because each wall landmark carries a unique, persistent integer ID assigned by the EKF, finding correspondences between the new submap and the global registry requires only a set intersection — no descriptor matching or nearest-neighbour search.

### 3.2 Shared-Wall Correspondences

Let $\mathcal{L}_{\mathrm{new}}$ be the set of landmark IDs in the current submap's `FeatureMap`, and $\mathcal{L}_{\mathrm{global}}$ be the set of IDs in the global wall registry. The shared IDs are

$$
\mathcal{L}_{\mathrm{shared}} = \mathcal{L}_{\mathrm{new}} \cap \mathcal{L}_{\mathrm{global}}.
$$

Each shared ID provides one pair of corresponding walls: the new submap's version (in approximate map frame via EKF pose) and the globally-corrected version stored in `global_walls`.

### 3.3 Overlap Section Sampling

For each shared wall pair, the system computes the overlapping section in the tangent direction. The target wall's Hessian angle $\alpha_{\mathrm{tgt}}$ defines the canonical tangent vector:

$$
\hat{\mathbf{t}} = [-\sin\alpha_{\mathrm{tgt}},\; \cos\alpha_{\mathrm{tgt}}]^\top.
$$

Each wall's endpoints are projected onto $\hat{\mathbf{t}}$. The overlap interval is

$$
[t_{\mathrm{lo}},\; t_{\mathrm{hi}}] = \bigl[\max(\min_s, \min_g),\; \min(\max_s, \max_g)\bigr],
$$

where $\min_s, \max_s$ and $\min_g, \max_g$ are the projected endpoint extents of the source and global wall, respectively. If $t_{\mathrm{hi}} - t_{\mathrm{lo}} < 0.3\;\mathrm{m}$ the wall pair is skipped (insufficient overlap).

From the overlap interval, $n_{\mathrm{samples}} = 8$ uniformly-spaced parameter values $t_k$ are drawn. Each yields a matched point pair:

$$
\mathbf{p}_k^{\mathrm{src}} = \rho_{\mathrm{src}}\,\hat{\mathbf{n}}_{\mathrm{src}} + t_k\,\hat{\mathbf{t}}, \qquad
\mathbf{p}_k^{\mathrm{tgt}} = \rho_{\mathrm{tgt}}\,\hat{\mathbf{n}}_{\mathrm{tgt}} + t_k\,\hat{\mathbf{t}},
$$

where $\hat{\mathbf{n}} = [\cos\alpha,\; \sin\alpha]^\top$ is the Hessian normal. All pairs from all shared walls are collected into a single set $\mathcal{P}$.

### 3.4 SVD Rigid Body Solve

Given the matched point pairs $\mathcal{P} = \{(\mathbf{p}_i^{\mathrm{src}}, \mathbf{p}_i^{\mathrm{tgt}})\}$, the one-shot SVD alignment solves for the 2D rigid body transform $(\mathbf{R}^*, \mathbf{t}^*)$ that minimises

$$
E(\mathbf{R}, \mathbf{t}) = \sum_{i \in \mathcal{P}} \|\mathbf{R}\,\mathbf{p}_i^{\mathrm{src}} + \mathbf{t} - \mathbf{p}_i^{\mathrm{tgt}}\|^2.
$$

The closed-form solution proceeds by centring:

$$
\bar{\mathbf{p}}^{\mathrm{src}} = \frac{1}{|\mathcal{P}|}\sum_i \mathbf{p}_i^{\mathrm{src}}, \qquad \bar{\mathbf{p}}^{\mathrm{tgt}} = \frac{1}{|\mathcal{P}|}\sum_i \mathbf{p}_i^{\mathrm{tgt}},
$$

$$
\mathbf{q}_i^{\mathrm{src}} = \mathbf{p}_i^{\mathrm{src}} - \bar{\mathbf{p}}^{\mathrm{src}}, \qquad \mathbf{q}_i^{\mathrm{tgt}} = \mathbf{p}_i^{\mathrm{tgt}} - \bar{\mathbf{p}}^{\mathrm{tgt}}.
$$

The 2×2 cross-covariance matrix is

$$
\mathbf{H} = \sum_i \mathbf{q}_i^{\mathrm{src}} (\mathbf{q}_i^{\mathrm{tgt}})^\top.
$$

SVD decomposition $\mathbf{H} = \mathbf{U}\,\mathbf{\Sigma}\,\mathbf{V}^\top$ gives, with a reflection guard,

$$
\mathbf{R}^* = \mathbf{V}\,\mathbf{D}\,\mathbf{U}^\top, \qquad \mathbf{D} = \mathrm{diag}(1,\; \mathrm{sgn}(\det(\mathbf{V}\mathbf{U}^\top))),
$$

$$
\mathbf{t}^* = \bar{\mathbf{p}}^{\mathrm{tgt}} - \mathbf{R}^*\,\bar{\mathbf{p}}^{\mathrm{src}}.
$$

The correction is extracted as a 2D pose increment:

$$
[dx,\; dy,\; d\theta] = \bigl[t^*_x,\; t^*_y,\; \arctan2(R^*_{10},\; R^*_{00})\bigr].
$$

### 3.5 Degeneracy Detection

The condition number of the alignment is

$$
\kappa = \frac{\Sigma_0}{\Sigma_1 + \varepsilon},
$$

where $\Sigma_0 \geq \Sigma_1$ are the two SVD singular values. A high condition number indicates that all shared walls are nearly parallel (e.g., a straight corridor), and the translation along the wall direction is unconstrained. If $\kappa > 100$ the alignment is rejected and the submap is integrated without correction.

### 3.6 First Submap Seeding

The first submap (index 0) is placed directly into the global map without alignment, since no global wall registry yet exists. Its walls are inserted into `global_walls` without any correction applied, establishing the reference frame for subsequent submaps.

---

## 4. SVD Singular-Value Covariance

### 4.1 Derivation

The covariance of the SVD pose estimate is derived from the SVD singular values and the spatial spread of the source points. The sensor noise model uses $\sigma = 0.01\;\mathrm{m}$ (LDS-01 specification), consistent with the EKF wall noise model.

**Translational covariance.** The weaker singular value $\Sigma_1$ quantifies how well the point cloud constrains translation in the worst-case direction. A small $\Sigma_1$ (nearly parallel walls) implies a poorly constrained direction:

$$
\sigma^2_{xy} = \frac{\sigma^2}{\Sigma_1 + \varepsilon}.
$$

**Rotational covariance.** The rotational constraint depends on how far the matched points are spread from their centroid. A larger mean squared distance implies a better-conditioned rotation estimate:

$$
\bar{d}^2 = \frac{1}{|\mathcal{P}|}\sum_i \|\mathbf{p}_i^{\mathrm{src}} - \bar{\mathbf{p}}^{\mathrm{src}}\|^2,
$$

$$
\sigma^2_\theta = \frac{\sigma^2}{|\mathcal{P}|\,\bar{d}^2 + \varepsilon}.
$$

**Full covariance matrix.** The pose correction covariance in $[x, y, \theta]$ space is

$$
\mathbf{R}_{\mathrm{SVD}} = \mathrm{diag}\!\left(\sigma^2_{xy},\; \sigma^2_{xy},\; \sigma^2_\theta\right).
$$

### 4.2 Interpretation

$\mathbf{R}_{\mathrm{SVD}}$ is geometry-aware:

- **Aligned walls, large spread:** small $\sigma^2_{xy}$ and $\sigma^2_\theta$ — tightly constrained correction.
- **Nearly parallel walls (corridor):** caught by degeneracy check ($\kappa > 100$) before covariance is computed; alignment is rejected.
- **Few point pairs:** $|\mathcal{P}|$ small → large $\sigma^2_\theta$ — rotation poorly constrained; EKF downweights accordingly.

The EKF uses $\mathbf{R}_{\mathrm{SVD}}$ as the measurement noise for the pose correction observation, so geometrically weak alignments automatically receive low weight.

---

## 5. Pose Correction Feedback

### 5.1 Correction Application

The SVD result provides the rigid correction applied to the new submap before integration. The 2×2 rotation $\mathbf{R}^*$ and translation $\mathbf{t}^*$ are embedded into a 4×4 homogeneous matrix:

$$
\mathbf{T}_{\mathrm{corr}} = \begin{bmatrix} \mathbf{R}^*_{2\times2} & \mathbf{t}^*_{2\times1} \\ \mathbf{0} & 1 \end{bmatrix}_{4\times4},
$$

and the globally-aligned transform is

$$
\mathbf{T}_{\mathrm{global}} = \mathbf{T}_{\mathrm{corr}}\,\mathbf{T}_{\mathrm{EKF}},
$$

where $\mathbf{T}_{\mathrm{EKF}}$ is the 4×4 transform from the current EKF pose.

### 5.2 EKF Integration

The correction $[dx, dy, d\theta]^\top$ is injected into the EKF as a direct pose observation. The observation model is the identity on the robot pose sub-state. The EKF update uses $\mathbf{R}_{\mathrm{SVD}}$ as the measurement noise covariance.

Because the EKF state includes all landmark parameters, the pose correction propagates to every landmark through the cross-covariance terms $\mathbf{P}_{r,m}$. This is the mechanism by which global consistency is maintained: an SVD correction at one point in the trajectory shifts all correlated landmarks consistently.

### 5.3 Fallback Behaviour

If fewer than 3 point pairs are available (`_svd_align` returns `None`), or if the condition number exceeds 100, no pose correction is sent to the EKF. The submap is still integrated into the global map using the EKF pose transform, maintaining map coverage. The EKF covariance continues to grow to reflect the lack of a global correction.

---

## 6. Global Map Maintenance

### 6.1 Data Structure

The global map is an Open3D tensor point cloud (`o3d.t.geometry.PointCloud`). The tensor API supports GPU acceleration via CUDA where available, falling back to CPU automatically.

In addition to the point cloud, the stitcher maintains `global_walls: Dict[int, Dict]` — a registry of globally-aligned wall parameters keyed by landmark ID. Each entry stores `{rho, alpha, start_point, end_point}` in the map frame. This registry is used for overlap computation in subsequent submap alignments.

### 6.2 Incremental Update

After alignment, the corrected submap is concatenated with the global map:

$$
\mathcal{M} \leftarrow \mathcal{M} \cup \mathcal{S}_{\mathrm{aligned}}.
$$

The merged map is then voxel-downsampled at $v = 0.05\;\mathrm{m}$ to maintain constant point density and prevent unbounded memory growth.

The global wall registry is updated via `_accumulate_walls`: the correction $(R^*, t^*)$ is applied to the new submap's wall endpoints before insertion. For existing landmark IDs the endpoint extent is extended along the stored wall's tangent direction; for new IDs the entry is inserted directly.

### 6.3 Dirty Cache

Numpy conversion from GPU tensors is expensive. The implementation caches the last numpy array and invalidates the cache only when the map is modified (`_map_dirty` flag). Reads that do not modify the map reuse the cached array.

### 6.4 Persistence

The global map is saved as a binary PCD file using Open3D's tensor I/O:

```python
o3d.t.io.write_point_cloud(filepath, global_map_tensor, write_ascii=False, compressed=False)
```

---

## 7. Implementation Reference

| Component | File | Notes |
|---|---|---|
| `SubmapStitcher` | `submap_stitcher.py` | Main class |
| `process_submap` | `submap_stitcher.py:36` | Voxel filter on ingestion |
| `_compute_overlap_correspondences` | `submap_stitcher.py:47` | Tangent-projection overlap + point sampling |
| `_svd_align` | `submap_stitcher.py:92` | SVD 2D rigid body solve |
| `feature_align_submaps` | `submap_stitcher.py:126` | Main alignment entry point |
| `_accumulate_walls` | `submap_stitcher.py:203` | Global wall registry upsert |
| `integrate_submap_to_global_map` | `submap_stitcher.py:267` | Full pipeline |
| `save_global_map` | `submap_stitcher.py:376` | PCD persistence |

### 7.1 Parameters

| Parameter | Default | Notes |
|---|---|---|
| `voxel_size` | 0.05 m | Downsampling resolution |
| `lidar_noise_sigma` | 0.01 m | LDS-01 spec; matches EKF wall noise |
| `n_samples` (overlap points per wall) | 8 | Sampled uniformly over overlap interval |
| Minimum overlap length | 0.3 m | Walls with less overlap are skipped |
| Condition number threshold | 100 | Above this: degenerate geometry, reject |
| `N_threshold` (scans per submap) | 50 | Set in `LocalSubmapGeneratorFeature` |

---

## References

1. **Arun, K. S., Huang, T. S., & Blostein, S. D. (1987).** "Least-Squares Fitting of Two 3-D Point Sets." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 9(5), 698–700. *(SVD rigid body alignment)*

2. **Horn, B. K. P. (1987).** "Closed-Form Solution of Absolute Orientation Using Unit Quaternions." *Journal of the Optical Society of America A*, 4(4), 629–642. *(Closed-form point set registration)*

3. **Bosse, M., Newman, P., Leonard, J., & Teller, S. (2004).** "Simultaneous Localisation and Map Building in Large-Scale Cyclic Environments Using the Atlas Framework." *The International Journal of Robotics Research*, 23(12), 1113–1139.

4. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.

5. **Zhou, Q.-Y., Park, J., & Koltun, V. (2018).** "Open3D: A Modern Library for 3D Data Processing." *arXiv:1801.09847*.
