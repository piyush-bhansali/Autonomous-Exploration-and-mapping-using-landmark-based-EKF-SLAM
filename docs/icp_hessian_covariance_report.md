# Detailed Report: Hessian-Based Uncertainty Calculation for ICP

## Table of Contents
1. [Introduction](#introduction)
2. [Problem Statement](#problem-statement)
3. [ICP Algorithm Overview](#icp-algorithm-overview)
4. [Theoretical Foundation](#theoretical-foundation)
5. [Mathematical Derivation](#mathematical-derivation)
6. [Step-by-Step Implementation](#step-by-step-implementation)
7. [Numerical Example](#numerical-example)
8. [Why This Approach is Correct](#why-this-approach-is-correct)
9. [Implementation in Code](#implementation-in-code)

---

## 1. Introduction

This report explains how we calculate uncertainty for the Iterative Closest Point algorithm. The ICP algorithm aligns two point clouds together. It estimates the transformation between them. However, ICP does not automatically give us uncertainty information. We need to compute this uncertainty separately. This document shows how we use the Hessian matrix to compute proper uncertainty estimates.

The uncertainty estimate tells us how confident we are in the ICP result. Low uncertainty means we trust the alignment. High uncertainty means the alignment might be inaccurate. This information is critical for sensor fusion. The Extended Kalman Filter needs uncertainty estimates to properly weight measurements.

## 2. Problem Statement

### What ICP Gives Us

ICP takes two point clouds as input. These are called the source and target point clouds. ICP outputs a transformation matrix. This matrix describes how to move the source points to align with the target points.

The transformation has three parameters in 2D:
- Translation in x direction: $\Delta x$
- Translation in y direction: $\Delta y$
- Rotation angle: $\Delta \theta$

We can write the transformation as a pose vector:

$$
\mathbf{T} = \begin{bmatrix}
\Delta x \\
\Delta y \\
\Delta \theta
\end{bmatrix}
$$

### What ICP Does Not Give Us

ICP does not tell us the uncertainty in this transformation. We do not know if the result is very accurate or very uncertain. Different environments produce different quality alignments. A corner-rich room gives accurate alignment. A long featureless corridor gives uncertain alignment. We need to quantify this uncertainty mathematically.

### Why We Need Uncertainty

The EKF-SLAM system uses ICP output as a measurement. The EKF needs a covariance matrix for every measurement. This covariance matrix is $\mathbf{R} \in \mathbb{R}^{3 \times 3}$ for the 2D pose. Without proper covariance, the filter cannot work correctly.

If we give the filter wrong covariance values, several problems occur:
- Too small covariance makes the filter overconfident
- The filter will reject good measurements
- The filter becomes inconsistent
- Divergence can occur

Therefore, computing correct covariance is essential.

---

## 3. ICP Algorithm Overview

### Basic ICP Steps

The ICP algorithm works in iterations. Each iteration has two main steps.

**Step 1: Find Correspondences**

For each point in the source cloud, we find the closest point in the target cloud. This creates point pairs. We call these pairs correspondences.

Let $\mathbf{p}_i^s$ be a source point. Let $\mathbf{p}_i^t$ be its closest target point. Then we have a correspondence pair $(\mathbf{p}_i^s, \mathbf{p}_i^t)$.

**Step 2: Estimate Transformation**

We find the transformation that minimizes the distance between all correspondence pairs. This is a least squares problem. The cost function is:

$$
E(\mathbf{T}) = \sum_{i=1}^{N} \| \mathbf{p}_i^t - (R(\Delta\theta) \mathbf{p}_i^s + \mathbf{t}) \|^2
$$

Where:
- $N$ is the number of correspondences
- $R(\Delta\theta)$ is the rotation matrix
- $\mathbf{t} = [\Delta x, \Delta y]^T$ is the translation vector
- $\|\cdot\|$ is the Euclidean distance

### ICP Convergence

These two steps repeat until convergence. Convergence happens when the change in transformation becomes very small. Typically ICP converges in 10-30 iterations. After convergence, we have the final transformation $\mathbf{T}$.

### ICP Output

The standard ICP implementation gives us:
- Final transformation matrix
- Fitness score (percentage of inlier points)
- RMSE (root mean square error)

But it does not give us covariance. We must compute this separately.

---

## 4. Theoretical Foundation

### Fisher Information Matrix

The Fisher Information Matrix tells us how much information the data provides about the parameters. More information means lower uncertainty. The Fisher Information Matrix is related to the curvature of the cost function.

For a least squares problem, the Fisher Information Matrix equals the Hessian matrix. The Hessian matrix is the second derivative of the cost function.

### Cramér-Rao Lower Bound

The Cramér-Rao bound is a fundamental result in estimation theory. It says that the covariance of any unbiased estimator cannot be smaller than the inverse of the Fisher Information Matrix.

Mathematically:

$$
\mathbf{P} \geq \mathbf{I}^{-1}
$$

Where:
- $\mathbf{P}$ is the covariance matrix
- $\mathbf{I}$ is the Fisher Information Matrix
- $\geq$ means the difference is positive semi-definite

For the maximum likelihood estimator with Gaussian noise, we achieve this bound exactly:

$$
\mathbf{P} = \mathbf{I}^{-1}
$$

### Connection to ICP

ICP solves a least squares problem. Each point correspondence gives us information about the transformation. The more point pairs we have, the more information we get. The geometric configuration of points also matters. Points spread out in different directions give more information. Points aligned in a line give less information.

The Hessian matrix captures both effects. It accounts for the number of points and their geometric distribution.

---

## 5. Mathematical Derivation

### Step 1: Define the Residual Function

For a single point pair, we define the residual vector. The residual is the distance between the target point and the transformed source point.

Let $\mathbf{p}_i^s = [x_i^s, y_i^s]^T$ be a source point. Let $\mathbf{p}_i^t = [x_i^t, y_i^t]^T$ be the corresponding target point.

After applying the transformation, the source point becomes:

$$
\mathbf{p}_i^{s,transformed} = R(\Delta\theta) \mathbf{p}_i^s + \mathbf{t}
$$

The rotation matrix is:

$$
R(\Delta\theta) = \begin{bmatrix}
\cos(\Delta\theta) & -\sin(\Delta\theta) \\
\sin(\Delta\theta) & \cos(\Delta\theta)
\end{bmatrix}
$$

We write this more compactly using $c = \cos(\Delta\theta)$ and $s = \sin(\Delta\theta)$:

$$
R = \begin{bmatrix}
c & -s \\
s & c
\end{bmatrix}
$$

The transformed point coordinates are:

$$
x_i^{transformed} = c \cdot x_i^s - s \cdot y_i^s + \Delta x
$$

$$
y_i^{transformed} = s \cdot x_i^s + c \cdot y_i^s + \Delta y
$$

The residual vector is:

$$
\mathbf{r}_i = \mathbf{p}_i^t - \mathbf{p}_i^{s,transformed}
$$

This gives us two residual components:

$$
r_{i,x} = x_i^t - (c \cdot x_i^s - s \cdot y_i^s + \Delta x)
$$

$$
r_{i,y} = y_i^t - (s \cdot x_i^s + c \cdot y_i^s + \Delta y)
$$

### Step 2: Compute the Jacobian

The Jacobian matrix contains the partial derivatives of the residual with respect to the parameters. We have three parameters: $\Delta x$, $\Delta y$, and $\Delta\theta$.

$$
\mathbf{J}_i = \frac{\partial \mathbf{r}_i}{\partial \mathbf{T}} = \begin{bmatrix}
\frac{\partial r_{i,x}}{\partial \Delta x} & \frac{\partial r_{i,x}}{\partial \Delta y} & \frac{\partial r_{i,x}}{\partial \Delta\theta} \\
\frac{\partial r_{i,y}}{\partial \Delta x} & \frac{\partial r_{i,y}}{\partial \Delta y} & \frac{\partial r_{i,y}}{\partial \Delta\theta}
\end{bmatrix}
$$

Let us compute each derivative carefully.

#### Derivative with respect to $\Delta x$

From the first residual component:

$$
r_{i,x} = x_i^t - (c \cdot x_i^s - s \cdot y_i^s + \Delta x)
$$

Taking the derivative:

$$
\frac{\partial r_{i,x}}{\partial \Delta x} = \frac{\partial}{\partial \Delta x} \left[x_i^t - c \cdot x_i^s + s \cdot y_i^s - \Delta x\right]
$$

The first three terms do not depend on $\Delta x$:

$$
\frac{\partial r_{i,x}}{\partial \Delta x} = 0 - 0 + 0 - 1 = -1
$$

From the second residual component:

$$
r_{i,y} = y_i^t - (s \cdot x_i^s + c \cdot y_i^s + \Delta y)
$$

This does not contain $\Delta x$:

$$
\frac{\partial r_{i,y}}{\partial \Delta x} = 0
$$

#### Derivative with respect to $\Delta y$

From the first residual component:

$$
\frac{\partial r_{i,x}}{\partial \Delta y} = 0
$$

Because $r_{i,x}$ does not contain $\Delta y$.

From the second residual component:

$$
\frac{\partial r_{i,y}}{\partial \Delta y} = \frac{\partial}{\partial \Delta y} \left[y_i^t - s \cdot x_i^s - c \cdot y_i^s - \Delta y\right]
$$

$$
\frac{\partial r_{i,y}}{\partial \Delta y} = 0 - 0 - 0 - 1 = -1
$$

#### Derivative with respect to $\Delta\theta$

This is more complex because $c$ and $s$ depend on $\Delta\theta$.

First, we need the derivatives of $c$ and $s$:

$$
\frac{\partial c}{\partial \Delta\theta} = \frac{\partial \cos(\Delta\theta)}{\partial \Delta\theta} = -\sin(\Delta\theta) = -s
$$

$$
\frac{\partial s}{\partial \Delta\theta} = \frac{\partial \sin(\Delta\theta)}{\partial \Delta\theta} = \cos(\Delta\theta) = c
$$

Now for the first residual component:

$$
r_{i,x} = x_i^t - c \cdot x_i^s + s \cdot y_i^s - \Delta x
$$

Taking the derivative:

$$
\frac{\partial r_{i,x}}{\partial \Delta\theta} = 0 - \frac{\partial c}{\partial \Delta\theta} \cdot x_i^s + \frac{\partial s}{\partial \Delta\theta} \cdot y_i^s - 0
$$

$$
= -(-s) \cdot x_i^s + c \cdot y_i^s
$$

$$
= s \cdot x_i^s + c \cdot y_i^s
$$

But we can write this more clearly. Recall that:

$$
\frac{\partial R}{\partial \theta} = \begin{bmatrix}
-s & -c \\
c & -s
\end{bmatrix}
$$

When we apply this derivative matrix to the source point:

$$
\frac{\partial R}{\partial \theta} \mathbf{p}_i^s = \begin{bmatrix}
-s & -c \\
c & -s
\end{bmatrix} \begin{bmatrix}
x_i^s \\
y_i^s
\end{bmatrix} = \begin{bmatrix}
-s \cdot x_i^s - c \cdot y_i^s \\
c \cdot x_i^s - s \cdot y_i^s
\end{bmatrix}
$$

Let us call this:

$$
\frac{\partial \mathbf{p}_i^{s,transformed}}{\partial \theta} = \begin{bmatrix}
dp_x/d\theta \\
dp_y/d\theta
\end{bmatrix} = \begin{bmatrix}
-s \cdot x_i^s - c \cdot y_i^s \\
c \cdot x_i^s - s \cdot y_i^s
\end{bmatrix}
$$

Since the residual is:

$$
r_{i,x} = x_i^t - p_x^{transformed}
$$

We have:

$$
\frac{\partial r_{i,x}}{\partial \Delta\theta} = 0 - \frac{\partial p_x^{transformed}}{\partial \theta} = -(-s \cdot x_i^s - c \cdot y_i^s)
$$

$$
= s \cdot x_i^s + c \cdot y_i^s
$$

Wait, I made a sign error. Let me recalculate carefully.

Actually, from our residual definition:

$$
r_{i,x} = x_i^t - x_i^{transformed}
$$

Where:

$$
x_i^{transformed} = c \cdot x_i^s - s \cdot y_i^s + \Delta x
$$

So:

$$
\frac{\partial x_i^{transformed}}{\partial \Delta\theta} = \frac{\partial c}{\partial \Delta\theta} \cdot x_i^s - \frac{\partial s}{\partial \Delta\theta} \cdot y_i^s
$$

$$
= (-s) \cdot x_i^s - c \cdot y_i^s
$$

Therefore:

$$
\frac{\partial r_{i,x}}{\partial \Delta\theta} = - \frac{\partial x_i^{transformed}}{\partial \Delta\theta} = -(-s \cdot x_i^s - c \cdot y_i^s)
$$

$$
= s \cdot x_i^s + c \cdot y_i^s
$$

Hmm, but in the code I see:

```python
dp_dtheta = dR_dtheta @ p_s
J = np.array([
    [-1.0, 0.0, -dp_dtheta[0]],
    [0.0, -1.0, -dp_dtheta[1]]
])
```

So the Jacobian uses $-dp\_dtheta$. Let me reconsider.

Actually, I think the issue is in how we define things. Let me be very careful.

We have:

$$
\mathbf{p}^{transformed} = R(\Delta\theta) \mathbf{p}^s + \mathbf{t}
$$

The derivative of the rotation applied to the point is:

$$
\frac{\partial (R \mathbf{p}^s)}{\partial \theta} = \frac{\partial R}{\partial \theta} \mathbf{p}^s
$$

Where:

$$
\frac{\partial R}{\partial \theta} = \begin{bmatrix}
-s & -c \\
c & -s
\end{bmatrix}
$$

So:

$$
\frac{\partial (R \mathbf{p}^s)}{\partial \theta} = \begin{bmatrix}
-s & -c \\
c & -s
\end{bmatrix} \begin{bmatrix}
x^s \\
y^s
\end{bmatrix} = \begin{bmatrix}
-s \cdot x^s - c \cdot y^s \\
c \cdot x^s - s \cdot y^s
\end{bmatrix}
$$

The residual is:

$$
\mathbf{r} = \mathbf{p}^t - \mathbf{p}^{transformed} = \mathbf{p}^t - (R \mathbf{p}^s + \mathbf{t})
$$

So:

$$
\frac{\partial \mathbf{r}}{\partial \mathbf{t}} = -\frac{\partial \mathbf{t}}{\partial \mathbf{t}} = -\mathbf{I}
$$

$$
\frac{\partial \mathbf{r}}{\partial \theta} = -\frac{\partial (R \mathbf{p}^s)}{\partial \theta}
$$

Therefore the Jacobian is:

$$
\mathbf{J}_i = \begin{bmatrix}
-1 & 0 & -(- s \cdot x_i^s - c \cdot y_i^s) \\
0 & -1 & -(c \cdot x_i^s - s \cdot y_i^s)
\end{bmatrix}
$$

$$
= \begin{bmatrix}
-1 & 0 & s \cdot x_i^s + c \cdot y_i^s \\
0 & -1 & -c \cdot x_i^s + s \cdot y_i^s
\end{bmatrix}
$$

But the code has a negative sign in front of dp_dtheta. Let me look at the code again:

```python
dR_dtheta = np.array([[-s, -c], [c, -s]])
dp_dtheta = dR_dtheta @ p_s
J = np.array([
    [-1.0, 0.0, -dp_dtheta[0]],
    [0.0, -1.0, -dp_dtheta[1]]
])
```

So `dp_dtheta[0] = -s*x - c*y` and `dp_dtheta[1] = c*x - s*y`.

Then the Jacobian is:

$$
\mathbf{J}_i = \begin{bmatrix}
-1 & 0 & -(-s \cdot x - c \cdot y) \\
0 & -1 & -(c \cdot x - s \cdot y)
\end{bmatrix}
$$

$$
= \begin{bmatrix}
-1 & 0 & s \cdot x + c \cdot y \\
0 & -1 & -c \cdot x + s \cdot y
\end{bmatrix}
$$

Wait, this doesn't match. Let me think again. In the code, the second row should be:

```
J[1, 2] = -dp_dtheta[1] = -(c*x - s*y) = -c*x + s*y
```

So the Jacobian is:

$$
\mathbf{J}_i = \begin{bmatrix}
-1 & 0 & s \cdot x_i^s + c \cdot y_i^s \\
0 & -1 & -c \cdot x_i^s + s \cdot y_i^s
\end{bmatrix}
$$

Hmm, but I got from first principles:

$$
\mathbf{J}_i = \begin{bmatrix}
-1 & 0 & s \cdot x_i^s + c \cdot y_i^s \\
0 & -1 & -c \cdot x_i^s + s \cdot y_i^s
\end{bmatrix}
$$

Great! They match. So our derivation is correct.

Actually, let me just double-check the sign. We have residual:

$$
r_x = x^t - (c \cdot x^s - s \cdot y^s + \Delta x)
$$

Let me differentiate with respect to $\theta$ directly. We have $c = \cos\theta$ and $s = \sin\theta$.

$$
\frac{\partial r_x}{\partial \theta} = 0 - \left(\frac{\partial c}{\partial \theta} \cdot x^s - \frac{\partial s}{\partial \theta} \cdot y^s\right)
$$

$$
= - ((-s) \cdot x^s - c \cdot y^s)
$$

$$
= - (-s \cdot x^s - c \cdot y^s)
$$

$$
= s \cdot x^s + c \cdot y^s
$$

Good! And for the y component:

$$
r_y = y^t - (s \cdot x^s + c \cdot y^s + \Delta y)
$$

$$
\frac{\partial r_y}{\partial \theta} = 0 - \left(\frac{\partial s}{\partial \theta} \cdot x^s + \frac{\partial c}{\partial \theta} \cdot y^s\right)
$$

$$
= - (c \cdot x^s + (-s) \cdot y^s)
$$

$$
= -c \cdot x^s + s \cdot y^s
$$

Perfect! So the complete Jacobian for point $i$ is:

$$
\mathbf{J}_i = \begin{bmatrix}
\frac{\partial r_{i,x}}{\partial \Delta x} & \frac{\partial r_{i,x}}{\partial \Delta y} & \frac{\partial r_{i,x}}{\partial \Delta\theta} \\
\frac{\partial r_{i,y}}{\partial \Delta x} & \frac{\partial r_{i,y}}{\partial \Delta y} & \frac{\partial r_{i,y}}{\partial \Delta\theta}
\end{bmatrix}
$$

$$
\mathbf{J}_i = \begin{bmatrix}
-1 & 0 & s \cdot x_i^s + c \cdot y_i^s \\
0 & -1 & -c \cdot x_i^s + s \cdot y_i^s
\end{bmatrix}
$$

Where $c = \cos(\Delta\theta)$ and $s = \sin(\Delta\theta)$.

### Step 3: Compute the Hessian Matrix

The Hessian matrix is built from all the individual Jacobians. For each point pair, we compute $\mathbf{J}_i^T \mathbf{J}_i$. Then we sum over all points.

$$
\mathbf{A} = \sum_{i=1}^{N} \mathbf{J}_i^T \mathbf{J}_i
$$

This is the Gauss-Newton approximation to the Hessian. It is exact when the residuals are small (which is true at the ICP solution).

Let us compute $\mathbf{J}_i^T \mathbf{J}_i$ explicitly.

$$
\mathbf{J}_i^T = \begin{bmatrix}
-1 & 0 \\
0 & -1 \\
s \cdot x_i^s + c \cdot y_i^s & -c \cdot x_i^s + s \cdot y_i^s
\end{bmatrix}
$$

Now we multiply:

$$
\mathbf{J}_i^T \mathbf{J}_i = \begin{bmatrix}
-1 & 0 \\
0 & -1 \\
s \cdot x_i^s + c \cdot y_i^s & -c \cdot x_i^s + s \cdot y_i^s
\end{bmatrix} \begin{bmatrix}
-1 & 0 & s \cdot x_i^s + c \cdot y_i^s \\
0 & -1 & -c \cdot x_i^s + s \cdot y_i^s
\end{bmatrix}
$$

Let me denote $a_i = s \cdot x_i^s + c \cdot y_i^s$ and $b_i = -c \cdot x_i^s + s \cdot y_i^s$ for simplicity.

$$
\mathbf{J}_i^T \mathbf{J}_i = \begin{bmatrix}
-1 & 0 \\
0 & -1 \\
a_i & b_i
\end{bmatrix} \begin{bmatrix}
-1 & 0 & a_i \\
0 & -1 & b_i
\end{bmatrix}
$$

Computing element by element:

**Element (0,0):**
$$
(-1) \cdot (-1) + 0 \cdot 0 = 1
$$

**Element (0,1):**
$$
(-1) \cdot 0 + 0 \cdot (-1) = 0
$$

**Element (0,2):**
$$
(-1) \cdot a_i + 0 \cdot b_i = -a_i
$$

**Element (1,0):**
$$
0 \cdot (-1) + (-1) \cdot 0 = 0
$$

**Element (1,1):**
$$
0 \cdot 0 + (-1) \cdot (-1) = 1
$$

**Element (1,2):**
$$
0 \cdot a_i + (-1) \cdot b_i = -b_i
$$

**Element (2,0):**
$$
a_i \cdot (-1) + b_i \cdot 0 = -a_i
$$

**Element (2,1):**
$$
a_i \cdot 0 + b_i \cdot (-1) = -b_i
$$

**Element (2,2):**
$$
a_i \cdot a_i + b_i \cdot b_i = a_i^2 + b_i^2
$$

So:

$$
\mathbf{J}_i^T \mathbf{J}_i = \begin{bmatrix}
1 & 0 & -a_i \\
0 & 1 & -b_i \\
-a_i & -b_i & a_i^2 + b_i^2
\end{bmatrix}
$$

Substituting back:

$$
\mathbf{J}_i^T \mathbf{J}_i = \begin{bmatrix}
1 & 0 & -(s \cdot x_i^s + c \cdot y_i^s) \\
0 & 1 & -(-c \cdot x_i^s + s \cdot y_i^s) \\
-(s \cdot x_i^s + c \cdot y_i^s) & -(-c \cdot x_i^s + s \cdot y_i^s) & (s \cdot x_i^s + c \cdot y_i^s)^2 + (-c \cdot x_i^s + s \cdot y_i^s)^2
\end{bmatrix}
$$

Let me simplify element (2,2). We have:

$$
(s \cdot x_i^s + c \cdot y_i^s)^2 + (-c \cdot x_i^s + s \cdot y_i^s)^2
$$

Expanding the first term:

$$
(s \cdot x_i^s)^2 + 2 s c \cdot x_i^s y_i^s + (c \cdot y_i^s)^2
$$

$$
= s^2 (x_i^s)^2 + 2sc x_i^s y_i^s + c^2 (y_i^s)^2
$$

Expanding the second term:

$$
(-c \cdot x_i^s)^2 + 2(-c)(s) x_i^s y_i^s + (s \cdot y_i^s)^2
$$

$$
= c^2 (x_i^s)^2 - 2cs x_i^s y_i^s + s^2 (y_i^s)^2
$$

Adding them:

$$
s^2 (x_i^s)^2 + 2sc x_i^s y_i^s + c^2 (y_i^s)^2 + c^2 (x_i^s)^2 - 2cs x_i^s y_i^s + s^2 (y_i^s)^2
$$

The cross terms cancel:

$$
= (s^2 + c^2)(x_i^s)^2 + (c^2 + s^2)(y_i^s)^2
$$

Since $s^2 + c^2 = \sin^2 + \cos^2 = 1$:

$$
= (x_i^s)^2 + (y_i^s)^2 = \|\mathbf{p}_i^s\|^2
$$

This is just the squared norm of the source point! So element (2,2) simplifies to:

$$
\mathbf{J}_i^T \mathbf{J}_i_{(2,2)} = \|\mathbf{p}_i^s\|^2
$$

The full Hessian is the sum over all points:

$$
\mathbf{A} = \sum_{i=1}^{N} \mathbf{J}_i^T \mathbf{J}_i
$$

$$
\mathbf{A} = \begin{bmatrix}
N & 0 & -\sum_i (s \cdot x_i^s + c \cdot y_i^s) \\
0 & N & -\sum_i (-c \cdot x_i^s + s \cdot y_i^s) \\
-\sum_i (s \cdot x_i^s + c \cdot y_i^s) & -\sum_i (-c \cdot x_i^s + s \cdot y_i^s) & \sum_i \|\mathbf{p}_i^s\|^2
\end{bmatrix}
$$

### Step 4: Add Sensor Noise

The Hessian matrix $\mathbf{A}$ tells us about the geometry. But we also need to account for sensor noise. The LiDAR sensor has measurement noise. For the TurtleBot3 LDS-01 sensor, the manufacturer specifies $\pm 10$mm accuracy.

This means the standard deviation is:

$$
\sigma = 0.01 \text{ meters}
$$

The variance is:

$$
\sigma^2 = 0.0001 \text{ m}^2
$$

This is a fixed value from the sensor specifications. We do not estimate it from the data. Estimating noise from data leads to the "optimism problem". More points would make the estimated noise smaller. This would make the filter overconfident.

Using fixed sensor noise from specifications avoids this problem.

### Step 5: Compute Covariance Matrix

The covariance matrix is:

$$
\mathbf{P} = \sigma^2 \mathbf{A}^{-1}
$$

This formula comes from the Cramér-Rao bound. It tells us the theoretical minimum uncertainty we can achieve given the sensor noise and the information in the data.

The dimensions are:
- $\mathbf{A}$: $3 \times 3$ (information matrix)
- $\mathbf{A}^{-1}$: $3 \times 3$ (inverse information matrix)
- $\sigma^2$: scalar (sensor noise variance)
- $\mathbf{P}$: $3 \times 3$ (covariance matrix)

The diagonal elements of $\mathbf{P}$ are variances:
- $P_{0,0}$: Variance in $\Delta x$ (m²)
- $P_{1,1}$: Variance in $\Delta y$ (m²)
- $P_{2,2}$: Variance in $\Delta\theta$ (rad²)

The off-diagonal elements are covariances showing correlation between parameters.

---

## 6. Step-by-Step Implementation

### Step 1: Extract Transformation Parameters

After ICP converges, we have a $4 \times 4$ transformation matrix. We extract the 2D pose from it.

```
Transform matrix T:
[[ c  -s  0  dx ]
 [ s   c  0  dy ]
 [ 0   0  1  0  ]
 [ 0   0  0  1  ]]
```

We extract:
- $\Delta x = T[0, 3]$
- $\Delta y = T[1, 3]$
- $\Delta\theta = \arctan2(T[1,0], T[0,0])$

The arctan2 function gives us the angle from the rotation matrix elements.

### Step 2: Find Point Correspondences

We need to find which target point corresponds to each source point. We use a KD-Tree for efficient nearest neighbor search.

**Build KD-Tree:**
```python
tree = KDTree(target_points)
```

**Transform source points:**

Apply the estimated transformation to source points:

$$
\mathbf{p}_i^{transformed} = R(\Delta\theta) \mathbf{p}_i^s + \mathbf{t}
$$

```python
R = np.array([[cos(dtheta), -sin(dtheta)],
              [sin(dtheta),  cos(dtheta)]])
transformed_source = (R @ source_points.T).T + np.array([dx, dy])
```

**Find nearest neighbors:**
```python
distances, indices = tree.query(transformed_source)
```

This gives us:
- `distances[i]`: Distance from transformed source point i to its nearest target point
- `indices[i]`: Index of the nearest target point

### Step 3: Filter Inliers

Not all correspondences are good. Some source points may not have a close target point. These are outliers. We filter them using a distance threshold.

```python
max_dist = 0.1  # 10 cm threshold
inlier_mask = distances < max_dist
```

We keep only the inlier pairs:

```python
src_inliers = source_points[inlier_mask]
tgt_inliers = target_points[indices[inlier_mask]]
```

We need at least 6 inlier points for the system to be well-determined. With fewer points, the Hessian matrix becomes singular.

### Step 4: Compute Rotation Derivative

We need $\frac{\partial R}{\partial \theta}$. This is:

$$
\frac{\partial R}{\partial \theta} = \begin{bmatrix}
-\sin(\Delta\theta) & -\cos(\Delta\theta) \\
\cos(\Delta\theta) & -\sin(\Delta\theta)
\end{bmatrix}
$$

```python
c = np.cos(dtheta)
s = np.sin(dtheta)
dR_dtheta = np.array([[-s, -c],
                       [ c, -s]])
```

### Step 5: Build Hessian Matrix

Initialize the Hessian as zeros:

```python
A = np.zeros((3, 3))
```

For each inlier pair, compute the Jacobian and accumulate:

```python
for p_s, p_t in zip(src_inliers, tgt_inliers):
    # Compute dp/dtheta
    dp_dtheta = dR_dtheta @ p_s

    # Build Jacobian (2x3)
    J = np.array([
        [-1.0, 0.0, -dp_dtheta[0]],
        [0.0, -1.0, -dp_dtheta[1]]
    ])

    # Accumulate J^T @ J
    A += J.T @ J
```

After the loop, $\mathbf{A}$ contains the sum of all $\mathbf{J}_i^T \mathbf{J}_i$.

### Step 6: Invert Hessian

We need to invert the Hessian matrix:

```python
try:
    A_inv = np.linalg.inv(A)
except np.linalg.LinAlgError:
    # Singular matrix - use pseudo-inverse
    A_inv = np.linalg.pinv(A)
```

The Hessian can be singular in degenerate cases. For example, in a long featureless corridor, we have no information about translation along the corridor. The pseudo-inverse handles this gracefully.

### Step 7: Scale by Sensor Noise

Finally, compute the covariance:

```python
sigma = 0.01  # 10mm LiDAR accuracy
sigma2 = sigma ** 2
covariance = sigma2 * A_inv
```

The result is a $3 \times 3$ covariance matrix for the pose $[\Delta x, \Delta y, \Delta\theta]^T$.

---

## 7. Numerical Example

Let us work through a simple example to make this concrete.

### Setup

Suppose we have 3 inlier point pairs after ICP:

**Source points (in source frame):**
- $\mathbf{p}_1^s = [1.0, 0.0]^T$
- $\mathbf{p}_2^s = [0.0, 1.0]^T$
- $\mathbf{p}_3^s = [1.0, 1.0]^T$

**Estimated transformation:**
- $\Delta x = 0.1$ m
- $\Delta y = 0.05$ m
- $\Delta\theta = 0.0$ rad (no rotation for simplicity)

**Sensor noise:**
- $\sigma = 0.01$ m
- $\sigma^2 = 0.0001$ m²

### Compute Rotation Matrices

Since $\Delta\theta = 0$:
- $c = \cos(0) = 1$
- $s = \sin(0) = 0$

$$
R = \begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix} = I
$$

$$
\frac{\partial R}{\partial \theta} = \begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix}
$$

### Compute Jacobians

**For point 1: $\mathbf{p}_1^s = [1, 0]^T$**

$$
\frac{\partial (R\mathbf{p}_1^s)}{\partial \theta} = \begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix} \begin{bmatrix}
1 \\
0
\end{bmatrix} = \begin{bmatrix}
0 \\
1
\end{bmatrix}
$$

$$
\mathbf{J}_1 = \begin{bmatrix}
-1 & 0 & 0 \\
0 & -1 & -1
\end{bmatrix}
$$

$$
\mathbf{J}_1^T \mathbf{J}_1 = \begin{bmatrix}
-1 & 0 \\
0 & -1 \\
0 & -1
\end{bmatrix} \begin{bmatrix}
-1 & 0 & 0 \\
0 & -1 & -1
\end{bmatrix} = \begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 1 \\
0 & 1 & 1
\end{bmatrix}
$$

**For point 2: $\mathbf{p}_2^s = [0, 1]^T$**

$$
\frac{\partial (R\mathbf{p}_2^s)}{\partial \theta} = \begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix} \begin{bmatrix}
0 \\
1
\end{bmatrix} = \begin{bmatrix}
-1 \\
0
\end{bmatrix}
$$

$$
\mathbf{J}_2 = \begin{bmatrix}
-1 & 0 & 1 \\
0 & -1 & 0
\end{bmatrix}
$$

$$
\mathbf{J}_2^T \mathbf{J}_2 = \begin{bmatrix}
-1 & 0 \\
0 & -1 \\
1 & 0
\end{bmatrix} \begin{bmatrix}
-1 & 0 & 1 \\
0 & -1 & 0
\end{bmatrix} = \begin{bmatrix}
1 & 0 & -1 \\
0 & 1 & 0 \\
-1 & 0 & 1
\end{bmatrix}
$$

**For point 3: $\mathbf{p}_3^s = [1, 1]^T$**

$$
\frac{\partial (R\mathbf{p}_3^s)}{\partial \theta} = \begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix} \begin{bmatrix}
1 \\
1
\end{bmatrix} = \begin{bmatrix}
-1 \\
1
\end{bmatrix}
$$

$$
\mathbf{J}_3 = \begin{bmatrix}
-1 & 0 & 1 \\
0 & -1 & -1
\end{bmatrix}
$$

$$
\mathbf{J}_3^T \mathbf{J}_3 = \begin{bmatrix}
-1 & 0 \\
0 & -1 \\
1 & -1
\end{bmatrix} \begin{bmatrix}
-1 & 0 & 1 \\
0 & -1 & -1
\end{bmatrix} = \begin{bmatrix}
1 & 0 & -1 \\
0 & 1 & 1 \\
-1 & 1 & 2
\end{bmatrix}
$$

### Sum to Get Hessian

$$
\mathbf{A} = \mathbf{J}_1^T \mathbf{J}_1 + \mathbf{J}_2^T \mathbf{J}_2 + \mathbf{J}_3^T \mathbf{J}_3
$$

$$
= \begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 1 \\
0 & 1 & 1
\end{bmatrix} + \begin{bmatrix}
1 & 0 & -1 \\
0 & 1 & 0 \\
-1 & 0 & 1
\end{bmatrix} + \begin{bmatrix}
1 & 0 & -1 \\
0 & 1 & 1 \\
-1 & 1 & 2
\end{bmatrix}
$$

$$
= \begin{bmatrix}
3 & 0 & -2 \\
0 & 3 & 2 \\
-2 & 2 & 4
\end{bmatrix}
$$

### Invert Hessian

To invert a $3 \times 3$ matrix, we use the formula:

$$
\mathbf{A}^{-1} = \frac{1}{\det(\mathbf{A})} \text{adj}(\mathbf{A})
$$

First, compute the determinant. Using the first row:

$$
\det(\mathbf{A}) = 3 \cdot \det\begin{bmatrix} 3 & 2 \\ 2 & 4 \end{bmatrix} - 0 + (-2) \cdot \det\begin{bmatrix} 0 & 3 \\ -2 & 2 \end{bmatrix}
$$

$$
= 3 \cdot (3 \cdot 4 - 2 \cdot 2) - 2 \cdot (0 \cdot 2 - 3 \cdot (-2))
$$

$$
= 3 \cdot (12 - 4) - 2 \cdot (0 + 6)
$$

$$
= 3 \cdot 8 - 2 \cdot 6 = 24 - 12 = 12
$$

For a simple $3 \times 3$, let me just use numpy in my head (this is getting long). The inverse is approximately:

$$
\mathbf{A}^{-1} \approx \begin{bmatrix}
0.667 & -0.333 & 0.5 \\
-0.333 & 0.667 & -0.5 \\
0.5 & -0.5 & 0.75
\end{bmatrix}
$$

### Compute Covariance

$$
\mathbf{P} = \sigma^2 \mathbf{A}^{-1} = 0.0001 \times \begin{bmatrix}
0.667 & -0.333 & 0.5 \\
-0.333 & 0.667 & -0.5 \\
0.5 & -0.5 & 0.75
\end{bmatrix}
$$

$$
\mathbf{P} = \begin{bmatrix}
6.67 \times 10^{-5} & -3.33 \times 10^{-5} & 5.0 \times 10^{-5} \\
-3.33 \times 10^{-5} & 6.67 \times 10^{-5} & -5.0 \times 10^{-5} \\
5.0 \times 10^{-5} & -5.0 \times 10^{-5} & 7.5 \times 10^{-5}
\end{bmatrix}
$$

### Interpret Results

The standard deviations are:
- $\sigma_x = \sqrt{P_{0,0}} = \sqrt{6.67 \times 10^{-5}} \approx 0.0082$ m = 8.2 mm
- $\sigma_y = \sqrt{P_{1,1}} = \sqrt{6.67 \times 10^{-5}} \approx 0.0082$ m = 8.2 mm
- $\sigma_\theta = \sqrt{P_{2,2}} = \sqrt{7.5 \times 10^{-5}} \approx 0.0087$ rad = 0.5°

These are very small uncertainties. This makes sense because:
1. We only have 3 points (real ICP uses hundreds)
2. The points are well-distributed
3. The sensor noise is small (10mm)

In a real scenario with 100 points, the uncertainty would be roughly 10x smaller.

---

## 8. Why This Approach is Correct

### It Follows Theory

The approach directly implements the Cramér-Rao lower bound. This is a fundamental result in estimation theory. The bound tells us the minimum achievable uncertainty for any unbiased estimator. The maximum likelihood estimator achieves this bound. ICP is essentially a maximum likelihood estimator under Gaussian noise assumptions.

### It Uses Fixed Sensor Noise

We use the sensor noise from manufacturer specifications. This is $\sigma = 0.01$ m for the TurtleBot3 LDS-01 LiDAR. We do not estimate noise from the residuals. This avoids the optimism problem.

**Optimism problem:** If we estimate noise as $\hat{\sigma}^2 = \frac{1}{N}\sum r_i^2$, then more points make $\hat{\sigma}^2$ smaller. This creates artificial confidence. The filter becomes overconfident and inconsistent.

**Our approach:** We use fixed $\sigma^2 = 0.0001$ regardless of point count. The Hessian $\mathbf{A}$ grows with the number of points. So $\mathbf{P} = \sigma^2 \mathbf{A}^{-1}$ decreases with more points. But the decrease is controlled by geometry, not by fitting residuals.

### It Captures Geometry

The Hessian matrix $\mathbf{A}$ automatically captures the geometric configuration. Consider these cases:

**Case 1: Points in all directions**

The Hessian has large eigenvalues in all directions. The covariance is small. The estimate is certain.

**Case 2: Points along a line**

The Hessian has one small eigenvalue (perpendicular direction). The corresponding eigenvector shows the uncertain direction. The covariance is large in that direction.

**Case 3: Few points**

The Hessian is small. The covariance is large. The estimate is uncertain.

All of this emerges naturally from the mathematics. We do not need to hand-tune different cases.

### It Matches Experimental Results

The pioneering work by Censi (2007) validated this approach experimentally. He compared the predicted covariance with empirical uncertainty from many ICP runs. The Hessian-based covariance accurately predicted the true uncertainty distribution.

Other researchers have also validated this approach. It is now standard in the robotics community for ICP uncertainty estimation.

### It Integrates with EKF

The EKF requires a covariance matrix for each measurement. Our computed $\mathbf{P}$ provides exactly this. The EKF uses $\mathbf{P}$ in the innovation covariance:

$$
\mathbf{S} = \mathbf{H} \mathbf{P}_{\text{predict}} \mathbf{H}^T + \mathbf{R}_{\text{ICP}}
$$

Where $\mathbf{R}_{\text{ICP}} = \mathbf{P}$ is our computed covariance. This enables proper fusion of ICP measurements with landmark observations.

---

## 9. Implementation in Code

Here is the complete implementation from our codebase:

```python
def _compute_icp_covariance(self,
                            source: o3d.t.geometry.PointCloud,
                            target: o3d.t.geometry.PointCloud,
                            transform: np.ndarray,
                            lidar_noise_sigma: float = 0.01) -> Optional[np.ndarray]:
    """
    Compute Hessian-based covariance for ICP pose estimate.

    Based on Censi (2007): "An Accurate Closed-Form Estimate of ICP's Covariance"

    Args:
        source: Source point cloud
        target: Target point cloud
        transform: 4x4 transformation matrix from ICP
        lidar_noise_sigma: LiDAR noise standard deviation (m)

    Returns:
        3x3 covariance matrix for [dx, dy, dtheta], or None if failed
    """
    try:
        # Convert tensors to numpy arrays
        source_points = source.point.positions.cpu().numpy()[:, :2]  # 2D (x, y)
        target_points = target.point.positions.cpu().numpy()[:, :2]

        # Need enough points for stable estimation
        if len(source_points) < 6 or len(target_points) < 6:
            return None

        # Extract 2D pose from 4x4 transform
        dx = transform[0, 3]
        dy = transform[1, 3]
        dtheta = np.arctan2(transform[1, 0], transform[0, 0])

        # Rotation matrix and its derivative
        c = np.cos(dtheta)
        s = np.sin(dtheta)
        R = np.array([[c, -s], [s, c]])
        dR_dtheta = np.array([[-s, -c], [c, -s]])

        # Find correspondences using KD-Tree
        tree = KDTree(target_points)
        transformed_source = (R @ source_points.T).T + np.array([dx, dy])
        distances, indices = tree.query(transformed_source)

        # Filter inliers based on distance threshold
        max_dist = 0.1  # 10 cm
        inlier_mask = distances < max_dist

        if np.count_nonzero(inlier_mask) < 6:
            return None

        # Get inlier pairs
        src_inliers = source_points[inlier_mask]
        tgt_inliers = target_points[indices[inlier_mask]]

        # Build Hessian (Information Matrix): A = sum(J^T @ J)
        A = np.zeros((3, 3))

        for p_s, p_t in zip(src_inliers, tgt_inliers):
            # Compute dp/dtheta using rotation derivative
            dp_dtheta = dR_dtheta @ p_s

            # Jacobian of residual r = p_t - (R*p_s + t) w.r.t. [dx, dy, dtheta]
            # dr/dx = -1, dr/dy = -1, dr/dtheta = -dp_dtheta
            J = np.array([
                [-1.0, 0.0, -dp_dtheta[0]],
                [0.0, -1.0, -dp_dtheta[1]]
            ])

            # Accumulate Hessian
            A += J.T @ J

        # Use FIXED LiDAR noise from sensor specifications
        # TurtleBot3 LDS-01: ±10mm accuracy
        # This avoids the "optimism problem" where more points reduce uncertainty
        sigma2 = lidar_noise_sigma ** 2

        # Invert Hessian to get covariance
        try:
            A_inv = np.linalg.inv(A)
        except np.linalg.LinAlgError:
            # Singular matrix - use pseudo-inverse
            # This can happen in degenerate configurations (e.g., corridor)
            A_inv = np.linalg.pinv(A)

        # Covariance = sigma^2 * A^(-1)
        # This is the Cramér-Rao lower bound
        covariance = sigma2 * A_inv

        return covariance

    except Exception as e:
        # Return None if any step fails
        return None
```

### Key Implementation Details

1. **2D Extraction**: We work in 2D (x, y) even though Open3D uses 3D. We extract only the x and y coordinates.

2. **Correspondence Finding**: We use KDTree for fast nearest neighbor search. This is much faster than brute force search.

3. **Inlier Filtering**: We use a 10 cm threshold. Points farther apart are likely mismatches.

4. **Minimum Points**: We require at least 6 inlier points. With fewer points, the system is underconstrained.

5. **Fixed Noise**: We use $\sigma = 0.01$ m from sensor specifications. This is never estimated from data.

6. **Pseudo-Inverse Fallback**: If the Hessian is singular, we use pseudo-inverse. This happens in degenerate cases like corridors.

7. **Error Handling**: We wrap everything in try-except. If anything fails, we return None. The calling code can handle this gracefully.

---

## Summary

This report explained how we compute uncertainty for ICP using the Hessian matrix. The key steps are:

1. Find point correspondences after ICP convergence
2. Compute the Jacobian of the residual for each point pair
3. Accumulate the Hessian as $\mathbf{A} = \sum \mathbf{J}_i^T \mathbf{J}_i$
4. Invert the Hessian to get $\mathbf{A}^{-1}$
5. Scale by fixed sensor noise: $\mathbf{P} = \sigma^2 \mathbf{A}^{-1}$

This approach is theoretically sound. It is based on the Cramér-Rao bound from estimation theory. It uses fixed sensor noise to avoid the optimism problem. It automatically captures geometric effects. It integrates naturally with the EKF framework.

The result is a covariance matrix that properly represents the uncertainty in the ICP pose estimate. This enables rigorous sensor fusion in the SLAM system.

---

## References

1. **Censi, A. (2007).** "An Accurate Closed-Form Estimate of ICP's Covariance." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3167-3172.
   - Original paper deriving the Hessian-based covariance formula
   - Provides theoretical justification and experimental validation
   - Standard reference for ICP uncertainty estimation

2. **Censi, A. (2008).** "An ICP Variant Using a Point-to-Line Metric." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 19-25.
   - Extension to point-to-line ICP
   - Covariance formulation for different ICP variants

3. **Kay, S. M. (1993).** *Fundamentals of Statistical Signal Processing: Estimation Theory*. Prentice Hall.
   - Chapter 3: Cramér-Rao Lower Bound
   - Theoretical foundation for minimum variance estimation

4. **Barfoot, T. D. (2017).** *State Estimation for Robotics*. Cambridge University Press.
   - Chapter 5: Nonlinear Least Squares
   - Section 5.3: Gauss-Newton approximation to the Hessian

5. **Prakhya, S. M., Liu, B., & Lin, W. (2015).** "A Closed-Form Estimate of 3D ICP Covariance." *Proceedings of 14th IAPR International Conference on Machine Vision Applications (MVA)*, pp. 526-529.
   - Extension of Censi's work to 3D
   - Practical implementation considerations

---

**File:** `icp_hessian_covariance_report.md`
**Location:** `/home/piyush/thesis_ws/docs/`
**Pages:** ~30 pages
**Last Updated:** 2026-02-10
