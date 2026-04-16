# `ss` — State-Space System Representation

## 1. Overview

The `ss` function constructs a **continuous-time linear time-invariant (LTI) state-space** system from its four defining matrices. In control theory, the state-space representation is the most general form for describing multi-input multi-output (MIMO) linear systems. It captures internal dynamics (states) that transfer-function representations may obscure, making it essential for modern control design, observer design, and simulation.

This function serves as the foundational building block for the entire toolbox — both `lsim` and `bode` consume the struct returned by `ss`.

---

## 2. Mathematical Background

A continuous-time LTI system is described by:

```
dx/dt = A·x(t) + B·u(t)          (state equation)
  y(t) = C·x(t) + D·u(t)          (output equation)
```

Where:

| Symbol | Dimension | Description |
|--------|-----------|-------------|
| `x(t)` | n×1 | State vector |
| `u(t)` | m×1 | Input vector |
| `y(t)` | p×1 | Output vector |
| `A` | n×n | State (or system) matrix — governs autonomous dynamics |
| `B` | n×m | Input matrix — maps inputs to state derivatives |
| `C` | p×n | Output matrix — maps states to outputs |
| `D` | p×m | Feedthrough matrix — direct input-to-output coupling |

**Key properties:**
- The eigenvalues of `A` determine system stability (Re(λ) < 0 for all eigenvalues ⟹ asymptotically stable).
- The pair `(A, B)` must be controllable for full state control.
- The pair `(A, C)` must be observable for state estimation.
- `D ≠ 0` implies the output responds *instantaneously* to the input (direct feedthrough).

---

## 3. Function Signature

```scilab
sys = ss(A, B, C, D)
```

### Example Usage

```scilab
exec("ss.sci", -1);

// First-order system: dx/dt = -x + u, y = x
sys = ss(-1, 1, 1, 0);

// Access fields
disp(sys.A);      // -1
disp(sys.nx);     // 1
disp(sys.type);   // "ss"
```

---

## 4. Parameters

| Parameter | Type | Dimension | Description |
|-----------|------|-----------|-------------|
| `A` | Real matrix | n×n | State matrix. Must be square. |
| `B` | Real matrix | n×m | Input matrix. Rows must match `A`. |
| `C` | Real matrix | p×n | Output matrix. Columns must match `A`. |
| `D` | Real matrix | p×m | Feedthrough matrix. Must be consistent with `B` columns and `C` rows. |

### Return Value

A Scilab `struct` with the following fields:

| Field | Type | Description |
|-------|------|-------------|
| `.A` | matrix | State matrix |
| `.B` | matrix | Input matrix |
| `.C` | matrix | Output matrix |
| `.D` | matrix | Feedthrough matrix |
| `.nx` | integer | Number of states (n) |
| `.nu` | integer | Number of inputs (m) |
| `.ny` | integer | Number of outputs (p) |
| `.Ts` | double | Sample time (0 = continuous-time) |
| `.type` | string | System type identifier, always `"ss"` |

---

## 5. Algorithm Explanation

The function performs no dynamic computation — it is a **validated constructor**. The algorithm is:

1. **Argument count check** — exactly 4 arguments required.
2. **Type validation** — all matrices must be `"constant"` type (Scilab's numeric type).
3. **Reality check** — reject complex-valued matrices (this toolbox targets real continuous-time systems).
4. **Dimension extraction** — determine `n`, `m`, `p` from matrix shapes.
5. **Dimension cross-validation**:
   - `A` must be square (n×n).
   - `B` must have n rows.
   - `C` must have n columns.
   - `D` must have p rows (matching `C`) and m columns (matching `B`).
6. **Stability warning** — compute eigenvalues of `A`; warn if any have positive real parts (open-loop instability).
7. **Struct assembly** — pack all matrices and metadata into a Scilab struct.

---

## 6. Test Cases

### Test 1: First-order stable SISO system
- **Input:** `A=-1, B=1, C=1, D=0`
- **Expected:** `nx=1, nu=1, ny=1`, all fields match input
- **Interpretation:** Simple exponentially stable system with unit gain

### Test 2: Second-order underdamped system
- **Input:** `A=[0,1;-4,-0.5], B=[0;1], C=[1,0], D=0`
- **Expected:** `nx=2, nu=1, ny=1`
- **Interpretation:** Mass-spring-damper with ωn=2, ζ=0.125

### Test 3: MIMO system (3 states, 2 inputs, 2 outputs)
- **Input:** Diagonal `A`, full `B` and `C`
- **Expected:** `nx=3, nu=2, ny=2`
- **Interpretation:** Demonstrates multi-channel support

### Test 4: Static gain (zero states)
- **Input:** `A=[], B=zeros(0,1), C=zeros(1,0), D=5`
- **Expected:** `nx=0, D=5`
- **Interpretation:** Pure algebraic system y=5u, no dynamics

### Test 5: Dimension mismatch (error case)
- **Input:** `A` is 2×2 but `B` has 3 rows
- **Expected:** Error raised with descriptive message
- **Interpretation:** Validates that malformed inputs are rejected

---

## 7. Limitations

| Limitation | Detail |
|------------|--------|
| **Continuous-time only** | `Ts` is always set to 0. Discrete-time systems are not supported. |
| **Real matrices only** | Complex system matrices are rejected. |
| **No transfer-function conversion** | This version does not convert between `ss` and `tf` representations. |
| **No controllability/observability checks** | These are left to the user. |
| **Stability warning is informational** | The function does not block unstable systems — it only warns. |

---

## Comparison with GNU Octave

| Feature | Octave `control` | This implementation |
|---------|-------------------|---------------------|
| `ss(A,B,C,D)` basic creation | ✅ | ✅ |
| Discrete-time (`ss(A,B,C,D,Ts)`) | ✅ | ❌ (continuous only) |
| `tf` → `ss` conversion | ✅ | ❌ |
| Display overloading | ✅ | ❌ (use `disp(sys.A)` etc.) |
| Property access (`.a`, `.b`) | ✅ (lowercase) | Fields are uppercase (`.A`, `.B`) |
