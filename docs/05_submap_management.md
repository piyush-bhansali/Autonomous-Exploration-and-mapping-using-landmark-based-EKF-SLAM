# Submap Management and Global Map Construction

## Table of Contents
1. [Introduction](#1-introduction)
2. [Submap Creation](#2-submap-creation)
3. [ICP-Based Stitching](#3-icp-based-stitching)
4. [ICP Covariance](#4-icp-covariance)
5. [Pose Correction Feedback](#5-pose-correction-feedback)
6. [Global Map Maintenance](#6-global-map-maintenance)
7. [Implementation Reference](#7-implementation-reference)

---

## 1. Introduction

The EKF operates in a local coordinate frame and accumulates drift over long trajectories. A separate submap-stitching layer corrects this drift by aligning accumulated local point clouds against the growing global map using ICP.

Every $N$ scans, the local point cloud is declared a complete submap. The submap is registered to the global map using point-to-point ICP. The resulting pose correction is fed back into the EKF as a direct pose observation, propagating the correction to all correlated landmarks.

The submap layer serves two purposes:
1. It builds and maintains a metric point cloud map of the environment.
2. It provides periodic global consistency corrections to the EKF state.

---

## 2. Submap Creation

### 2.1 Trigger

A submap is finalised when the scan count reaches a fixed threshold:

$$
n_{\mathrm{scans}} \geq N_{\mathrm{threshold}}, \qquad N_{\mathrm{threshold}} = 50.
$$

At 1 Hz scan rate this corresponds to approximately 50 seconds of travel. This duration is short enough that odometric drift within a single submap is small (typically < 0.05 m), which keeps the initial guess for ICP within the convergence basin.

### 2.2 Point Accumulation

Scan points are expressed in the robot body frame at the time of acquisition. Before accumulation each scan is transformed into the submap-local frame, which is anchored at the robot pose when the submap was initialised, $(x_0, y_0, \theta_0)$.

Let the current robot pose be $(x_r, y_r, \theta_r)$ in the map frame. The relative pose of the current scan with respect to the submap origin is

$$
\Delta x = x_r - x_0, \quad \Delta y = y_r - y_0, \quad \Delta\theta = \theta_r - \theta_0.
$$

Each scan point $\mathbf{p}^{\mathrm{robot}}$ is transformed to the submap frame by the rotation $\mathbf{R}(\theta_r)$ followed by translation, then expressed relative to the submap origin. In practice the implementation passes a 4×4 transformation matrix $\mathbf{T}_{\mathrm{submap}}$ computed from the EKF state at scan time.

All accumulated points are stored in the submap-local frame. When the submap is finalised the full transformation $\mathbf{T}_{\mathrm{submap}}^{\mathrm{map}}$ (start pose in the map frame) is used to project the submap into the global map frame before ICP.

### 2.3 Voxel Downsampling

Before ICP each submap is voxel-downsampled at $v = 0.05$ m. This reduces the point count, removes redundant measurements from overlapping scans, and gives uniform spatial density. A submap with fewer than 50 points after downsampling is discarded without attempting ICP.

---

## 3. ICP-Based Stitching

### 3.1 Algorithm

Point-to-point ICP (Besl & McKay, 1992) minimises the sum of squared distances between matched point pairs. Given a source point cloud $\mathcal{S}$ and a target cloud $\mathcal{T}$, ICP seeks the rigid transform $(\mathbf{R}^*, \mathbf{t}^*)$ that minimises

$$
E(\mathbf{R}, \mathbf{t}) = \sum_{i \in \mathcal{C}} \| \mathbf{R}\,\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i \|^2,
$$

where $\mathcal{C}$ is the set of closest-point correspondences $(\mathbf{p}_i \in \mathcal{S},\, \mathbf{q}_i \in \mathcal{T})$ within the maximum correspondence distance.

**Implementation.** The system uses Open3D's tensor-based ICP (`o3d.t.pipelines.registration.icp`) with:
- Maximum correspondence distance: 0.05 m (equal to the voxel size)
- Maximum iterations: 50
- Convergence: relative fitness and RMSE change < $10^{-6}$
- Initial guess: the transformation matrix from the current EKF pose

The first submap (index 0) is placed directly into the global map without ICP, since there is no existing map to register against.

### 3.2 Fitness Score

Open3D reports an ICP fitness score as the fraction of source points that find a correspondence within the maximum distance. A submap is accepted if

$$
\mathrm{fitness} \geq 0.45.
$$

A fitness below this threshold indicates poor overlap, and the odometric transform is used instead.

### 3.3 Correction Sanity Check

Even when ICP converges, the correction is rejected if it is implausibly large:

$$
\| \Delta \mathbf{t} \| > 1.0\;\mathrm{m} \quad \text{or} \quad |\Delta\theta| > 20°.
$$

A correction this large would indicate ICP has converged to a wrong minimum. In that case the odometric transform is used and no pose correction is sent to the EKF.

---

## 4. ICP Covariance

### 4.1 Derivation

The covariance of the ICP pose estimate is derived from the Gauss–Newton Hessian of the cost function (Censi, 2007). Each inlier correspondence contributes a Jacobian row $\mathbf{J}_i \in \mathbb{R}^{2 \times 3}$ relating a small perturbation $[\delta x, \delta y, \delta\theta]^\top$ to the residual at that point:

$$
\mathbf{J}_i = \begin{bmatrix} -1 & 0 & -\dot{R}\,\mathbf{p}_i[0] \\ 0 & -1 & -\dot{R}\,\mathbf{p}_i[1] \end{bmatrix},
$$

where $\dot{R} = \frac{d\mathbf{R}}{d\theta} = \begin{bmatrix} -\sin\theta & -\cos\theta \\ \cos\theta & -\sin\theta \end{bmatrix}$ evaluated at the ICP solution $\theta^*$.

The information matrix (Gauss–Newton Hessian) is

$$
\mathbf{A} = \sum_{i \in \mathcal{C}_{\mathrm{inlier}}} \mathbf{J}_i^\top \mathbf{J}_i,
$$

and the covariance is

$$
\mathbf{R}_{\mathrm{ICP}} = \sigma^2\,\mathbf{A}^{-1},
$$

where $\sigma = 0.01\;\mathrm{m}$ is the LiDAR range noise from the TurtleBot3 LDS-01 specification. This is the same sensor noise model used for wall covariance in the EKF.

### 4.2 Interpretation

$\mathbf{R}_{\mathrm{ICP}}$ is a $3 \times 3$ matrix in $[x, y, \theta]$ space. Its eigenvalues capture how well the alignment is constrained in each direction:

- Along a flat wall: $x$ and $y$ components along the wall are poorly constrained (large eigenvalue).
- In a corner or L-shaped corridor: all three components are well constrained (small eigenvalues).
- In a featureless corridor: translation along the corridor direction is almost unconstrained (very large eigenvalue in that direction).

This geometry-awareness makes $\mathbf{R}_{\mathrm{ICP}}$ more informative than a fixed diagonal covariance. The EKF uses $\mathbf{R}_{\mathrm{ICP}}$ as the measurement noise for the pose correction observation, so degenerate ICP alignments automatically receive low weight.

### 4.3 Fallback

If fewer than 6 inlier correspondences remain after distance filtering, or if $\mathbf{A}$ is singular, the covariance computation returns `None`. When `None` is returned, the EKF update for the ICP correction is skipped.

---

## 5. Pose Correction Feedback

### 5.1 Correction Extraction

The ICP result is a $4 \times 4$ transform $\mathbf{T}_{\mathrm{refined}}$ from source to target. The correction relative to the odometric initial guess is

$$
\mathbf{T}_{\mathrm{corr}} = \mathbf{T}_{\mathrm{odometric}}^{-1}\,\mathbf{T}_{\mathrm{refined}}.
$$

The correction is extracted as a 2D pose increment:

$$
[dx,\; dy,\; d\theta] = \bigl[\mathbf{T}_{\mathrm{corr}}[0,3],\; \mathbf{T}_{\mathrm{corr}}[1,3],\; \arctan2(\mathbf{T}_{\mathrm{corr}}[1,0],\; \mathbf{T}_{\mathrm{corr}}[0,0])\bigr].
$$

A correction is only forwarded to the EKF when $\|[dx, dy]\| > 0.01\;\mathrm{m}$. Smaller corrections are below the sensor noise level and would add noise without benefit.

### 5.2 EKF Integration

The correction $[dx, dy, d\theta]^\top$ is injected into the EKF as a direct pose observation. The observation model is the identity on the robot pose sub-state. The EKF update uses $\mathbf{R}_{\mathrm{ICP}}$ as the measurement noise covariance.

Because the EKF state includes all landmark parameters, the pose correction propagates to every landmark through the cross-covariance terms $\mathbf{P}_{r,m}$. This is the mechanism by which global consistency is maintained: an ICP correction at one point in the trajectory shifts all correlated landmarks consistently.

---

## 6. Global Map Maintenance

### 6.1 Data Structure

The global map is an Open3D tensor point cloud (`o3d.t.geometry.PointCloud`). The tensor API supports GPU acceleration via CUDA where available, falling back to CPU automatically.

### 6.2 Incremental Update

After ICP, the aligned submap is concatenated with the global map:

$$
\mathcal{M} \leftarrow \mathcal{M} \cup \mathcal{S}_{\mathrm{aligned}}.
$$

The merged map is then voxel-downsampled at $v = 0.05\;\mathrm{m}$ to maintain constant point density and prevent unbounded memory growth.

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
| `process_submap` | `submap_stitcher.py:38` | Voxel filter on ingestion |
| `align_submap_with_icp` | `submap_stitcher.py:47` | ICP + covariance |
| `_compute_icp_covariance` | `submap_stitcher.py:91` | Gauss–Newton Hessian |
| `integrate_submap_to_global_map` | `submap_stitcher.py:159` | Full pipeline |
| `save_global_map` | `submap_stitcher.py:260` | PCD persistence |

### 7.1 Parameters

| Parameter | Default | Notes |
|---|---|---|
| `voxel_size` | 0.05 m | Downsampling resolution |
| `icp_max_correspondence_dist` | 0.05 m | = voxel size |
| `icp_fitness_threshold` | 0.45 | Minimum inlier fraction |
| `lidar_noise_sigma` | 0.01 m | LDS-01 spec; matches EKF wall noise |
| `N_threshold` (scans per submap) | 50 | Set in `LocalSubmapGeneratorFeature` |

---

## References

1. **Besl, P. J., & McKay, N. D. (1992).** "A Method for Registration of 3-D Shapes." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239–256.

2. **Censi, A. (2007).** "An Accurate Closed-Form Estimate of ICP's Covariance." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3167–3172.

3. **Bosse, M., Newman, P., Leonard, J., & Teller, S. (2004).** "Simultaneous Localisation and Map Building in Large-Scale Cyclic Environments Using the Atlas Framework." *The International Journal of Robotics Research*, 23(12), 1113–1139.

4. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.

5. **Zhou, Q.-Y., Park, J., & Koltun, V. (2018).** "Open3D: A Modern Library for 3D Data Processing." *arXiv:1801.09847*.
