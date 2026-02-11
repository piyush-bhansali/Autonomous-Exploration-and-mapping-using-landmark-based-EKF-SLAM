# Thesis Abstract

## A Comparative Study of ICP-Based and Feature-Based SLAM for Autonomous Mobile Robots Using Frontier Exploration

---

### Abstract

Simultaneous Localization and Mapping (SLAM) is fundamental to autonomous mobile robotics, enabling robots to construct maps of unknown environments while tracking their pose. Two dominant paradigms exist: dense scan-matching methods like Iterative Closest Point (ICP) and sparse feature-based approaches using Extended Kalman Filter SLAM (EKF-SLAM). While both have been extensively studied independently, rigorous comparative evaluations under identical operational conditions remain limited. This thesis presents a systematic comparison of ICP-based and feature-based SLAM implementations for autonomous indoor navigation.

We develop two complete SLAM systems within a unified ROS 2 framework: (1) an **ICP-based approach** using scan-to-submap alignment with Hessian-based uncertainty quantification (Censi, 2007), and (2) a **feature-based approach** using geometric landmark extraction (walls in Hessian normal form and corner points) with full EKF-SLAM and Fisher Information Matrix covariance tracking. To ensure fair comparison, both systems operate under identical frontier-based autonomous exploration strategies, eliminating exploration bias and ensuring both methods observe the same environment regions.

The ICP-based system employs dense point cloud registration for high-accuracy short-term localization, accumulating scans into local submaps with pose-only state estimation. The feature-based system extracts geometric primitives from laser scans, performs Mahalanobis distance-based data association with segment overlap validation, and maintains a joint robot-landmark state with full covariance tracking. Both approaches implement submap-based hierarchical mapping for scalability and provide quantitative uncertainty estimates through information-theoretic confidence metrics.

Experimental evaluation is conducted in structured indoor environments (simulated in Gazebo and real-world scenarios) using a TurtleBot3 platform with 2D LiDAR. Performance is assessed across multiple dimensions: localization accuracy via Absolute Trajectory Error (ATE) against ground truth, map quality through point cloud alignment metrics, computational efficiency via per-scan processing time, and uncertainty calibration using chi-squared consistency tests.

Key contributions include: (1) a rigorous controlled comparison framework ensuring identical exploration trajectories, (2) comprehensive uncertainty quantification for both dense and sparse methods, (3) detailed analysis of computational-accuracy trade-offs, (4) open-source ROS 2 implementation with reproducible evaluation methodology, and (5) quantitative guidance for practitioners on selecting appropriate SLAM approaches based on environment structure, computational constraints, and accuracy requirements.

Results demonstrate that ICP-based methods achieve superior short-term localization accuracy and complete environmental coverage at the cost of higher computational load and potential drift accumulation. Feature-based methods provide consistent long-term accuracy through explicit landmark correspondence, compact map representations, and interpretable geometric primitives, but require sufficient environmental structure for reliable feature extraction. Uncertainty calibration analysis reveals that feature-based full covariance tracking better reflects true estimation error compared to ICP Hessian estimates, particularly over extended operation.

This work provides a comprehensive empirical foundation for understanding the practical trade-offs between dense and sparse SLAM paradigms in real-world autonomous navigation scenarios.

---

### Keywords

Simultaneous Localization and Mapping (SLAM), Iterative Closest Point (ICP), Extended Kalman Filter (EKF), Frontier-based Exploration, Uncertainty Quantification, Feature Extraction, Data Association, Mobile Robotics, ROS 2, Comparative Analysis

---

### Thesis Structure

1. **Introduction** - Motivation, problem statement, contributions
2. **Literature Review** - ICP-based SLAM, feature-based SLAM, frontier exploration, comparative studies
3. **Methodology: ICP-Based Mapping** - Algorithm, implementation, uncertainty quantification
4. **Methodology: Feature-Based Mapping** - Feature extraction, EKF-SLAM, data association
5. **Experimental Setup** - Hardware platform, environments, evaluation metrics
6. **Results and Analysis** - Accuracy, efficiency, uncertainty calibration, trade-off analysis
7. **Conclusion and Future Work** - Summary, practical guidance, extensions

---

### Supervisor Information
[Add supervisor name and affiliation]

### Institution
[Add institution name]

### Date
February 2026

---

**Word Count**: 442 words (suitable for most thesis requirements; can be condensed to 250-300 words if needed)
