// Five test cases for frequency-response computation and plotting.

clear; clc;

exec(fullfile(pwd(), "..", "ss", "ss.sci"), -1);
exec(fullfile(pwd(), "bode.sci"), -1);

mprintf("\n");
mprintf("============================================================\n");
mprintf("  bode() — Test Suite\n");
mprintf("============================================================\n\n");

passed = 0;
failed = 0;

// TEST 1: First-order low-pass filter
// G(s) = 1/(s+1)   =>  A=-1, B=1, C=1, D=0
// At w=1 rad/s: |G| = 1/sqrt(2) => -3.01 dB,  phase = -45°

mprintf("--- Test 1: First-order LPF — verify at w=1 ---\n");

try
    sys1 = ss(-1, 1, 1, 0);
    w1 = [1];   
    [mag1, ph1, ~] = bode(sys1, w1);

    expected_mag = 20 * log10(1 / sqrt(2));  
    expected_ph  = -45;  

    mprintf("  Magnitude at w=1: %.4f dB  (expected %.4f dB)\n", mag1, expected_mag);
    mprintf("  Phase at w=1:     %.4f deg (expected %.4f deg)\n", ph1, expected_ph);

    if abs(mag1 - expected_mag) < 0.01 & abs(ph1 - expected_ph) < 0.1 then
        mprintf("  PASSED.\n");
        passed = passed + 1;
    else
        mprintf("  FAILED — values outside tolerance.\n");
        failed = failed + 1;
    end
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");

// TEST 2: Second-order underdamped system — resonance peak
// G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
// wn=10, zeta=0.1  =>  resonance near w=wn
// A = [0,1; -100,-2],  B=[0;1],  C=[100,0],  D=0
// Peak magnitude ≈ 1/(2*zeta) = 5  => ~14 dB

mprintf("--- Test 2: Second-order resonance peak ---\n");
try
    wn = 10; zeta = 0.1;
    A2 = [0, 1; -wn^2, -2*zeta*wn];
    B2 = [0; 1];
    C2 = [wn^2, 0];
    D2 = 0;
    sys2 = ss(A2, B2, C2, D2);

    w2 = logspace(0, 2, 1000)';
    [mag2, ph2, ~] = bode(sys2, w2);

    peak_mag = max(mag2);
    // Theoretical peak: 1/(2*zeta*sqrt(1-zeta^2)) for resonant frequency
    // For zeta=0.1: ≈ 1/(0.2*0.995) ≈ 5.025  => 14.02 dB
    theoretical_peak_dB = 20 * log10(1 / (2 * zeta * sqrt(1 - zeta^2)));

    mprintf("  Peak magnitude: %.2f dB  (theoretical ≈ %.2f dB)\n", ..
            peak_mag, theoretical_peak_dB);

    if abs(peak_mag - theoretical_peak_dB) < 0.5 then
        mprintf("  PASSED — resonance peak within 0.5 dB of theory.\n");
        passed = passed + 1;
    else
        mprintf("  FAILED — peak deviates too much.\n");
        failed = failed + 1;
    end
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");

// TEST 3: Pure integrator G(s) = 1/s
// A=0, B=1, C=1, D=0
// |G(jw)| = 1/w  => -20 dB/decade slope
// At w=1: 0 dB;  at w=10: -20 dB;  at w=0.1: +20 dB
// Phase = -90° for all w > 0
mprintf("--- Test 3: Pure integrator — slope and phase ---\n");
try
    sys3 = ss(0, 1, 1, 0);
    w3 = [0.1; 1; 10];
    [mag3, ph3, ~] = bode(sys3, w3);


    expected_mags = [20; 0; -20];
    mag_err = max(abs(mag3 - expected_mags));


    ph_err = max(abs(ph3 - (-90)));

    mprintf("  Magnitudes: [%.2f, %.2f, %.2f] dB  (expected [20, 0, -20])\n", ..
            mag3(1), mag3(2), mag3(3));
    mprintf("  Phases:     [%.2f, %.2f, %.2f] deg (expected -90)\n", ..
            ph3(1), ph3(2), ph3(3));

    if mag_err < 0.01 & ph_err < 0.1 then
        mprintf("  PASSED.\n");
        passed = passed + 1;
    else
        mprintf("  FAILED — magnitude error=%.4f, phase error=%.4f.\n", mag_err, ph_err);
        failed = failed + 1;
    end
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");


// TEST 4: Step input / DC gain verification
// G(s) = 2/(s+1)  =>  A=-1, B=1, C=2, D=0
// DC gain = G(0) = C*(-A)^{-1}*B + D = 2*1*1 = 2  => 6.02 dB
// At very low frequency, magnitude should approach 6.02 dB
mprintf("--- Test 4: DC gain at very low frequency ---\n");
try
    sys4 = ss(-1, 1, 2, 0);
    w4 = [0.001];
    [mag4, ~, ~] = bode(sys4, w4);

    dc_gain_dB = 20 * log10(2); 
    mprintf("  Magnitude at w=0.001: %.4f dB  (DC gain = %.4f dB)\n", ..
            mag4, dc_gain_dB);

    if abs(mag4 - dc_gain_dB) < 0.01 then
        mprintf("  PASSED — DC gain verified.\n");
        passed = passed + 1;
    else
        mprintf("  FAILED — DC gain mismatch.\n");
        failed = failed + 1;
    end
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");

// TEST 5: Edge case — dimension mismatch / invalid input
// Passing a non-struct as sys — should error
mprintf("--- Test 5: Invalid sys argument (expect error) ---\n");
try
    [m5, p5, w5] = bode("not_a_system", [1, 10, 100]);
    mprintf("  FAILED — no error raised.\n");
    failed = failed + 1;
catch
    mprintf("  PASSED — correctly rejected: %s\n", lasterror());
    passed = passed + 1;
end
mprintf("\n");

// Summary
mprintf("============================================================\n");
mprintf("  Results: %d passed, %d failed out of %d tests\n", ..
        passed, failed, passed + failed);
mprintf("============================================================\n");


mprintf("\n  Generating Bode plot for first-order LPF (visual check)...\n");
sys_plot = ss(-1, 1, 1, 0);
bode(sys_plot);
mprintf("  Plot window opened. Close it manually.\n");
