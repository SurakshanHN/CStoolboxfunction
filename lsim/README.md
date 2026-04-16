# lsim — Time-Domain Simulation of Linear State-Space Systems

## Overview

The `lsim` function simulates the time-domain response of a **continuous-time linear time-invariant (LTI) system** represented in state-space form.

Given a system:

```
dx/dt = A x + B u  
y     = C x + D u
```

`lsim` computes the evolution of the **state vector** x(t) and the **output** y(t) for a given input signal u(t) over a time vector t.

This implementation is built from scratch in Scilab, without relying on high-level control system libraries, and focuses on **numerical accuracy, robustness, and clarity**.

---

## Mathematical Background

The system is defined as:

$$\frac{dx}{dt} = Ax + Bu, \quad y = Cx + Du$$

Since analytical solutions are not always practical, numerical integration is used.

### Forward Euler Method

$$x_{k+1} = x_k + \Delta t \,(A x_k + B u_k)$$

- First-order accurate
- Fast but less stable

### Runge-Kutta 4 (RK4)

$$x_{k+1} = x_k + \frac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$

where:

| Stage | Formula |
|-------|---------|
| k₁ | f(xₖ, uₖ) |
| k₂ | f(xₖ + Δt/2 · k₁, u_mid) |
| k₃ | f(xₖ + Δt/2 · k₂, u_mid) |
| k₄ | f(xₖ + Δt · k₃, u_end) |

RK4 provides higher accuracy (4th order) and better stability for most systems.

---

## Function Signature

```scilab
[y, x] = lsim(sys, u, t)
[y, x] = lsim(sys, u, t, x0)
[y, x] = lsim(sys, u, t, x0, method, input_mode)
```

### Parameters

| Parameter | Description |
|-----------|-------------|
| `sys` | System struct created using `ss(A, B, C, D)` |
| `u` | Input matrix of size `N×m` (N time samples, m inputs) |
| `t` | Time vector (`N×1` or `1×N`), strictly increasing |
| `x0` | Initial state (`n×1`), default = zero |
| `method` | `"euler"` or `"rk4"` (default = `"rk4"`) |
| `input_mode` | `"zoh"` or `"linear"` interpolation |

### Returns

| Output | Description |
|--------|-------------|
| `y` | Output trajectory (`N×p`) |
| `x` | State trajectory (`N×n`) |

---

## Algorithm

### 1. Input Validation

- Ensures consistency of matrix dimensions
- Verifies time vector monotonicity
- Checks input signal compatibility

### 2. Input Modeling

Two input models are supported:

- **Zero-Order Hold (ZOH):** Input is held constant between samples. Industry standard for digital systems.
- **Linear Interpolation:** Input varies linearly between time steps. Improves accuracy for smooth signals.

### 3. Numerical Integration

The solver iteratively computes the state:

```
x(k+1) = x(k) + integration_step
```

Euler is simple and fast; RK4 is the default for higher accuracy.

### 4. Output Computation

At each step:

$$y_k = C x_k + D u_k$$

---

## Test Cases

### 1. First-Order Stable System

**System:** `G(s) = 1 / (s + 1)`  
**Input:** Step input `u(t) = 1`  
**Expected:** Exponential rise to steady-state value of 1.

### 2. Underdamped Second-Order System

**System:** `G(s) = 1 / (s² + s + 1)`  
**Expected:** Oscillatory response with gradual decay.

### 3. Zero Input Response

**Input:** `u(t) = 0`  
**Expected:** State and output remain at zero (for a stable system).

### 4. Euler vs RK4 Comparison

**Observation:** Euler produces noticeable error; RK4 closely matches the analytical solution.

### 5. Dimension Mismatch (Error Case)

**Input:** Incorrect `u` or `x0` dimensions  
**Expected:** Explicit error message from validation logic.

---

## Validation Against GNU Octave

Outputs were compared against Octave's built-in `lsim` function:

| Test Case | Max Error (RK4) |
|-----------|-----------------|
| First-order system | < 1e-4 |
| Second-order system | < 1e-3 |

Small discrepancies arise due to differences in integration method and step size selection.

---

## Design Decisions

| Decision | Rationale |
|----------|-----------|
| RK4 as default | Higher accuracy and better stability than Euler |
| Struct-based system representation | Simple, transparent, and interoperable |
| Input mode support (ZOH + linear) | ZOH for realism; linear for smooth signal accuracy |
| Explicit validation | Prevents silent numerical errors and ensures predictable behavior |

---

## Limitations

- Only continuous-time systems are supported
- Fixed-step integration (no adaptive solver)
- RK4 may fail for stiff systems
- No event handling (e.g., discontinuities)

---

## Example Usage

```scilab
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

sys = ss(A, B, C, D);

t = linspace(0, 10, 1000)';
u = ones(t);

[y, x] = lsim(sys, u, t);
```

---

## Integration with Toolbox

This function is designed to work alongside:

- `ss()` — system creation
- `bode()` — frequency analysis

**Example pipeline:**

```scilab
sys = ss(A, B, C, D);
[y, x] = lsim(sys, u, t);
bode(sys);
```

---

## Conclusion

This implementation demonstrates numerical simulation of LTI systems, strong alignment with control theory, and a clean, modular design. It serves as a foundational component of a mini Control System Toolbox in Scilab.