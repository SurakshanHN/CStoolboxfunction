// lsim_test.sce — Test Suite for lsim.sci
// Five test cases covering first-order step, underdamped, zero input, step
// comparison, and invalid-dimension error.

clear; clc;

// Load dependencies
exec(fullfile(pwd(), "..", "ss", "ss.sci"), -1);
exec(fullfile(pwd(), "lsim.sci"), -1);

mprintf("\n");
mprintf("============================================================\n");
mprintf("  lsim() — Test Suite\n");
mprintf("============================================================\n\n");

passed = 0;
failed = 0;

// TEST 1: First-order stable system — step response
mprintf("--- Test 1: First-order step response (RK4) ---\n");
try
    sys1 = ss(-1, 1, 1, 0);
    t1 = linspace(0, 5, 5001)';
    u1 = ones(length(t1), 1);
    [y1, x1] = lsim(sys1, u1, t1);

    y_analytical = 1 - exp(-t1);
    max_err = max(abs(y1 - y_analytical));
    mprintf("  Max absolute error vs analytical: %.2e\n", max_err);

    if max_err < 1e-6 then
        mprintf("  PASSED — RK4 matches analytical solution to < 1e-6.\n");
        passed = passed + 1;
    else
        mprintf("  FAILED — error too large.\n");
        failed = failed + 1;
    end
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");

// TEST 2: Second-order underdamped system — impulse-like response
mprintf("--- Test 2: Second-order underdamped free response ---\n");
try
    A2 = [0, 1; -4, -0.5];
    B2 = [0; 1];
    C2 = [1, 0];
    D2 = [0];
    sys2 = ss(A2, B2, C2, D2);

    t2 = linspace(0, 10, 10001)';
    u2 = zeros(length(t2), 1);
    x0_2 = [1; 0];
    [y2, x2] = lsim(sys2, u2, t2, x0_2);

    // Analytical free response
    wn = 2; zeta = 0.125; wd = wn * sqrt(1 - zeta^2);
    y2_anal = exp(-zeta * wn * t2) .* (cos(wd * t2) + (zeta * wn / wd) * sin(wd * t2));
    max_err2 = max(abs(y2 - y2_anal));
    mprintf("  Max error vs analytical: %.2e\n", max_err2);

    if max_err2 < 1e-5 then
        mprintf("  PASSED — underdamped response correct.\n");
        passed = passed + 1;
    else
        mprintf("  FAILED — error too large.\n");
        failed = failed + 1;
    end
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");


// TEST 3: Zero input — state should remain at initial value (stable sys)
mprintf("--- Test 3: Zero input, zero dynamics ---\n");
try
    sys3 = ss(0, 0, 1, 0);
    t3 = linspace(0, 1, 101)';
    u3 = zeros(101, 1);
    x0_3 = [5];
    [y3, x3] = lsim(sys3, u3, t3, x0_3);

    max_err3 = max(abs(y3 - 5));
    mprintf("  Max deviation from x0=5: %.2e\n", max_err3);

    if max_err3 < 1e-12 then
        mprintf("  PASSED — state remains constant.\n");
        passed = passed + 1;
    else
        mprintf("  FAILED — state drifted.\n");
        failed = failed + 1;
    end
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");


// TEST 4: Euler vs RK4 comparison on stiff-ish system
mprintf("--- Test 4: Euler vs RK4 accuracy comparison ---\n");
try
    sys4 = ss(-10, 10, 1, 0);
    t4 = linspace(0, 1, 1001)';    // dt = 0.001
    u4 = ones(length(t4), 1);

    [y4_euler, ~] = lsim(sys4, u4, t4, [0], "euler");
    [y4_rk4,   ~] = lsim(sys4, u4, t4, [0], "rk4");

    y4_anal = 1 - exp(-10 * t4);
    err_euler = max(abs(y4_euler - y4_anal));
    err_rk4   = max(abs(y4_rk4   - y4_anal));

    mprintf("  Euler max error: %.2e\n", err_euler);
    mprintf("  RK4   max error: %.2e\n", err_rk4);

    if err_rk4 < err_euler then
        mprintf("  PASSED — RK4 is more accurate than Euler (as expected).\n");
        passed = passed + 1;
    else
        mprintf("  FAILED — RK4 should beat Euler.\n");
        failed = failed + 1;
    end
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");

// TEST 5: Dimension mismatch (should produce error)
mprintf("--- Test 5: Dimension mismatch (expect error) ---\n");
try
    sys5 = ss(-1, 1, 1, 0);
    t5 = linspace(0, 1, 101)';
    u5 = ones(101, 2);    // 2 columns, but sys has only 1 input
    [y5, x5] = lsim(sys5, u5, t5);
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
