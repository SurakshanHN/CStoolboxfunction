// ss_test.sce — Test Suite for ss.sci
// Exercises ss() with five test cases covering nominal, edge, and error paths.

clear; clc;
exec(fullfile(pwd(), "ss.sci"), -1);

mprintf("\n");
mprintf("============================================================\n");
mprintf("  ss() — Test Suite\n");
mprintf("============================================================\n\n");

passed = 0;
failed = 0;

// TEST 1: First-order stable system
// System: dx/dt = -1*x + 1*u,   y = 1*x
// Single state, single input, single output.
// Expected: struct with nx=1, nu=1, ny=1.
mprintf("--- Test 1: First-order stable SISO system ---\n");
try
    sys1 = ss(-1, 1, 1, 0);
    assert_checkequal(sys1.nx, 1);
    assert_checkequal(sys1.nu, 1);
    assert_checkequal(sys1.ny, 1);
    assert_checkequal(sys1.A, -1);
    assert_checkequal(sys1.B, 1);
    assert_checkequal(sys1.C, 1);
    assert_checkequal(sys1.D, 0);
    mprintf("  PASSED — fields A,B,C,D and dimensions correct.\n");
    passed = passed + 1;
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");


// TEST 2: Second-order underdamped system (mass-spring-damper)
// State: [position; velocity]
// Parameters: m=1, b=0.5, k=4  =>  wn=2, zeta=0.125
//   A = [0, 1; -4, -0.5]
//   B = [0; 1]
//   C = [1, 0]      (output = position)
//   D = [0]
mprintf("--- Test 2: Second-order underdamped system ---\n");
try
    A2 = [0, 1; -4, -0.5];
    B2 = [0; 1];
    C2 = [1, 0];
    D2 = [0];
    sys2 = ss(A2, B2, C2, D2);
    assert_checkequal(sys2.nx, 2);
    assert_checkequal(sys2.nu, 1);
    assert_checkequal(sys2.ny, 1);
    mprintf("  PASSED — 2-state system created, nx=%d nu=%d ny=%d.\n", ..
            sys2.nx, sys2.nu, sys2.ny);
    passed = passed + 1;
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");



// TEST 3: MIMO system (2 inputs, 2 outputs, 3 states)
mprintf("--- Test 3: MIMO system (3 states, 2 inputs, 2 outputs) ---\n");
try
    A3 = [-1, 0, 0; 0, -2, 0; 0, 0, -3];
    B3 = [1, 0; 0, 1; 1, 1];
    C3 = [1, 0, 0; 0, 1, 1];
    D3 = zeros(2, 2);
    sys3 = ss(A3, B3, C3, D3);
    assert_checkequal(sys3.nx, 3);
    assert_checkequal(sys3.nu, 2);
    assert_checkequal(sys3.ny, 2);
    mprintf("  PASSED — MIMO system dimensions: nx=%d nu=%d ny=%d.\n", ..
            sys3.nx, sys3.nu, sys3.ny);
    passed = passed + 1;
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");


// TEST 4: Static gain (zero states)
// A = [], B = [], C = [], D = [5]
// This represents y = 5*u with no dynamics.
mprintf("--- Test 4: Static gain system (zero states) ---\n");
try
    A4 = [];
    B4 = zeros(0, 1);
    C4 = zeros(1, 0);
    D4 = [5];
    sys4 = ss(A4, B4, C4, D4);
    assert_checkequal(sys4.nx, 0);
    assert_checkequal(sys4.nu, 1);
    assert_checkequal(sys4.ny, 1);
    assert_checkequal(sys4.D, 5);
    mprintf("  PASSED — static gain D=5, nx=0.\n");
    passed = passed + 1;
catch
    mprintf("  FAILED — %s\n", lasterror());
    failed = failed + 1;
end
mprintf("\n");


// TEST 5: Edge case — dimension mismatch (should produce error)
// A is 2×2 but B has 3 rows — must be rejected.
mprintf("--- Test 5: Dimension mismatch (expect error) ---\n");
err_caught = %f;
try
    sys5 = ss([1, 0; 0, 1], [1; 2; 3], [1, 0], [0]);
    mprintf("  FAILED — no error raised for mismatched B.\n");
    failed = failed + 1;
catch
    err_caught = %t;
    mprintf("  PASSED — correctly rejected: %s\n", lasterror());
    passed = passed + 1;
end
mprintf("\n");


// Summary
mprintf("============================================================\n");
mprintf("  Results: %d passed, %d failed out of %d tests\n", ..
        passed, failed, passed + failed);
mprintf("============================================================\n");
