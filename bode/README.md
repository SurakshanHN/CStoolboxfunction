# `bode` — Frequency Response (Bode Plot) for State-Space Systems

## 1. Overview

The `bode` function computes and optionally plots the **frequency response** of a continuous-time LTI state-space system. The Bode plot is the most widely used tool in classical control for analyzing gain and phase margins, bandwidth, resonance peaks, and overall loop-shaping behavior.

This implementation:
- Computes the transfer function matrix `G(jω)` directly from state-space matrices — no conversion to polynomial transfer functions
- Supports both **SISO** and **MIMO** systems
- Generates magnitude (dB) and phase (degrees) Bode plots
- Automatically determines a suitable frequency range from the system eigenvalues
- Implements phase unwrapping for continuous phase curves

---

## 2. Mathematical Background

### Transfer Function from State-Space

For a continuous-time system `(A, B, C, D)`, the frequency response at angular frequency `ω` is:

```
G(jω) = C · (jωI − A)⁻¹ · B + D
```

Where:
- `j = √(−1)` is the imaginary unit
- `I` is the n×n identity matrix
- `G(jω)` is a p×m complex matrix (one entry per input-output channel)

### Magnitude and Phase

For each element `G_ij(jω)` of the transfer matrix:

```
Magnitude:  |G_ij(jω)|  in dB  = 20 · log₁₀( |G_ij(jω)| )

Phase:      ∠G_ij(jω)  in deg  = atan2( Im(G_ij), Re(G_ij) ) × (180/π)
```

### Matrix Inversion

Instead of explicitly computing `(jωI − A)⁻¹`, the implementation uses **left division** (backslash operator):

```
(jωI − A)⁻¹ · B  =  (jωI − A) \ B
```

This is numerically more stable than `inv(M) * B` because:
1. It avoids forming the full inverse matrix
2. Scilab uses LU decomposition with partial pivoting internally
3. It incurs lower round-off error, especially for ill-conditioned systems

### Phase Unwrapping

Raw `atan2` output is limited to `[−180°, 180°]`. For systems with multiple poles (high-order), the true phase may exceed this range. The unwrapping algorithm ensures successive phase samples differ by less than 180°:

The implementation normalizes phase differences using modular arithmetic:

Δφ = mod(Δφ + 180, 360) − 180

This ensures phase continuity even for large jumps.

### Automatic Frequency Range

When no frequency vector is provided, the function determines a range from the eigenvalues of `A`:

```
ω_min = min(|eigenvalues|),   ω_max = max(|eigenvalues|)
Frequency range: [ω_min / 100,  ω_max × 100]    (2 decades margin each side)
Minimum span: 4 decades
Points: 500 logarithmically spaced
```

This ensures that all system dynamics (poles and zeros) are visible in the plot.

---

## 3. Function Signature

```scilab
// Plot only (no return values)
bode(sys)
bode(sys, w)

// Return data (no plot)
[mag, phase, w] = bode(sys)
[mag, phase, w] = bode(sys, w)
```

### Example Usage

```scilab
exec("../ss/ss.sci", -1);
exec("bode.sci", -1);

// First-order low-pass filter: G(s) = 1/(s+1)
sys = ss(-1, 1, 1, 0);

// Auto-frequency Bode plot
bode(sys);

// Retrieve numerical data at specific frequencies
w = logspace(-2, 2, 500)';
[mag, phase, w_out] = bode(sys, w);
```

---

## 4. Parameters

| Parameter | Type | Dimension | Required | Default | Description |
|-----------|------|-----------|----------|---------|-------------|
| `sys` | struct | — | ✅ | — | State-space system created by `ss()` |
| `w` | vector | Nw×1 | ❌ | auto | Frequency vector in rad/s (strictly positive) |

### Return Values (optional)

| Output | Dimension | Description |
|--------|-----------|-------------|
| `mag` | Nw×1 (SISO) or Nw×p×m (MIMO) | Magnitude in linear scale (absolute) |
| `phase` | Nw×1 (SISO) or Nw×p×m (MIMO) | Phase in degrees (unwrapped) |
| `w` | Nw×1 | Frequency vector used |

When called with **zero output arguments**, the function generates a Bode plot instead.

---

## 5. Algorithm Explanation

### Computation pipeline

1. **Validate** the system struct and frequency vector
2. **Generate frequencies** automatically if not provided (from eigenvalue magnitudes)
3. **Loop** over each frequency `ω_k`:
   a. Form `M = jω_k · I − A` (n×n complex matrix)
   b. Check condition number via `rcond(M)` — warn if near-singular
   c. Compute `G(jω_k) = C · (M \ B) + D` using left division
   d. Extract magnitude |G| (linear scale; converted to dB only for plotting)
   e. Extract phase `∠G` → convert to degrees
4. **Unwrap** the phase to ensure continuity
5. **Plot** (if no output arguments) or **return** data

### Plotting details

- **SISO:** Two vertically stacked subplots (magnitude on top, phase below), both with logarithmic x-axis
- **MIMO:** Grid of `2p × m` subplots — magnitude and phase for each input→output channel
- X-axis: frequency in rad/s (log scale)
- Y-axis top: magnitude in dB (linear scale)
- Y-axis bottom: phase in degrees (linear scale)
- Grid lines enabled for readability

---

## 6. Test Cases

### Test 1: First-order low-pass filter at ω = 1
- **System:** `G(s) = 1/(s+1)`
- **Frequency:** `ω = 1` rad/s (the pole location)
- **Expected:** `|G| = 1/√2 ≈ −3.01 dB`, `∠G = −45°`
- **Result:** Both match to within 0.01 dB and 0.1°
- **Interpretation:** The classic −3 dB point of a first-order system

### Test 2: Second-order resonance peak
- **System:** `ωn = 10, ζ = 0.1`
- **Expected peak:** `1/(2ζ√(1−ζ²)) ≈ 14.02 dB`
- **Result:** Computed peak within 0.5 dB of theory
- **Interpretation:** Validates resonance detection at the natural frequency

### Test 3: Pure integrator — slope and phase
- **System:** `G(s) = 1/s` — `A=0, B=1, C=1, D=0`
- **Frequencies:** `ω = [0.1, 1, 10]`
- **Expected magnitude:** `[+20, 0, −20]` dB (−20 dB/decade slope)
- **Expected phase:** `−90°` at all frequencies
- **Result:** Exact match
- **Interpretation:** Validates the fundamental −20 dB/dec and −90° of a single integrator

### Test 4: DC gain verification
- **System:** `G(s) = 2/(s+1)` — DC gain = 2 → 6.02 dB
- **Frequency:** `ω = 0.001` (very low)
- **Expected:** magnitude ≈ 6.02 dB
- **Result:** Match within 0.01 dB
- **Interpretation:** At very low frequencies, the response should equal the DC gain

### Test 5: Invalid input rejection
- **Input:** a string instead of a system struct
- **Expected:** Error raised
- **Result:** Error caught with descriptive message
- **Interpretation:** Validates type-checking on the first argument

---

## 7. Limitations

| Limitation | Detail |
|------------|--------|
| **Continuous-time only** | Discrete-time frequency response (`e^{jωT}`) is not implemented |
| **No gain/phase margin** | The function does not automatically compute GM, PM, or crossover frequencies |
| **Near-singular warnings** | At frequencies matching eigenvalues, `(jωI − A)` becomes ill-conditioned. The function warns but does not skip these points. |
| **No Nichols or Nyquist** | Only Bode (magnitude + phase vs frequency) is generated |
| **MIMO plotting** | For large MIMO systems, the subplot grid may become unreadable. Consider extracting individual channels. |
| **Phase unwrapping** | The simple algorithm works well for slowly varying phase but may fail for very sparse frequency vectors with rapid phase changes. |

### Numerical considerations

- **Condition number:** The function monitors `rcond(jωI − A)` and warns when it drops below `10⁻¹⁵`. At these frequencies, magnitude and phase values should be treated as approximate.
- **Frequency resolution:** For sharp resonance peaks (low damping), use a dense frequency vector near the resonant frequency to capture the peak accurately.
- **Left division:** Using `M \ B` instead of `inv(M) * B` This approach generally improves numerical stability compared to explicit matrix inversion, especially for ill-conditioned systems.

---

## Comparison with GNU Octave

| Feature | Octave `control` | This implementation |
|---------|-------------------|---------------------|
| `bode(sys)` with auto plot | ✅ | ✅ |
| `[mag, phase] = bode(sys, w)` | ✅ | ✅ |
| Accepts `tf` objects | ✅ | ❌ (requires `ss` struct) |
| Returns linear magnitude | ✅ | ✅ |
| dB conversion | user must compute | used only in plotting |
| Phase unwrapping | ✅ | ✅ (simple algorithm) |
| MIMO subplot grids | ✅ | ✅ |
| Gain/phase margin overlay | ✅ (`margin`) | ❌ |
| Nichols/Nyquist | ✅ (separate functions) | ❌ |

### Key difference
Octave's `bode` returns magnitude in **absolute** (linear) scale by default, requiring the user to convert to dB. This implementation returns magnitude directly in **dB**, which is more convenient for most analysis tasks but differs from the Octave convention.
