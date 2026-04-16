// ss.sci — State-Space System Representation
// Creates a continuous-time state-space system object from matrices A, B, C, D.
// Mathematical model:
//   dx/dt = A*x + B*u
//       y = C*x + D*u


function sys = ss(A, B, C, D)
    
    // ss  Create a continuous-time state-space system representation.
    // Calling Sequence
    //   sys = ss(A, B, C, D)
    // Parameters
    //   A : n×n  state matrix         (real)
    //   B : n×m  input matrix          (real)
    //   C : p×n  output matrix         (real)
    //   D : p×m  direct-feedthrough    (real)
    // Returns
    //   sys : struct with fields
    //         .A, .B, .C, .D        — system matrices
    //         .nx                    — number of states   (n)
    //         .nu                    — number of inputs   (m)
    //         .ny                    — number of outputs  (p)
    //         .type                  — string "ss"
    //         .Ts                    — sample time (0 = continuous)

    // Argument count check 
    if argn(2) < 4 then
        error("ss: exactly 4 arguments required — ss(A, B, C, D)");
    end

    // Type checks
    if typeof(A) <> "constant" then
        error("ss: A must be a real numeric matrix.");
    end
    if typeof(B) <> "constant" then
        error("ss: B must be a real numeric matrix.");
    end
    if typeof(C) <> "constant" then
        error("ss: C must be a real numeric matrix.");
    end
    if typeof(D) <> "constant" then
        error("ss: D must be a real numeric matrix.");
    end

    // Reject complex matrices (continuous-time real systems)
    if ~isreal(A) then
        error("ss: A must be a real matrix (no complex entries).");
    end
    if ~isreal(B) then
        error("ss: B must be a real matrix (no complex entries).");
    end
    if ~isreal(C) then
        error("ss: C must be a real matrix (no complex entries).");
    end
    if ~isreal(D) then
        error("ss: D must be a real matrix (no complex entries).");
    end

    // Extract dimensions 
    [nA_r, nA_c] = size(A);
    [nB_r, nB_c] = size(B);
    [nC_r, nC_c] = size(C);
    [nD_r, nD_c] = size(D);

    // Validate A is square 
    if nA_r <> nA_c then
        error(msprintf("ss: A must be square. Got %d×%d.", nA_r, nA_c));
    end

    n = nA_r;   // number of states
    m = nB_c;   // number of inputs
    p = nC_r;   // number of outputs

    // Validate B dimensions (n×m)
    if nB_r <> n then
        error(msprintf("ss: B must have %d rows to match A (%d×%d). Got %d rows.", ..
                        n, n, n, nB_r));
    end

    // Validate C dimensions (p×n)
    if nC_c <> n then
        error(msprintf("ss: C must have %d columns to match A (%d×%d). Got %d columns.", ..
                        n, n, n, nC_c));
    end

    // Validate D dimensions (p×m)
    if nD_r <> p then
        error(msprintf("ss: D must have %d rows to match C (%d×%d). Got %d rows.", ..
                        p, p, n, nD_r));
    end
    if nD_c <> m then
        error(msprintf("ss: D must have %d columns to match B (%d×%d). Got %d columns.", ..
                        m, n, m, nD_c));
    end

    // Handle degenerate scalar-as-matrix edge case
    // (Scilab treats scalars as 1×1 matrices, so this is fine by default.)

    // Warn if system is unstable (informational only)
    eigenvalues = spec(A);
    if max(real(eigenvalues)) > 0 then
        mprintf("  ss: WARNING — system is open-loop unstable ");
        mprintf("(max Re(eigenvalue) = %.4f).\n", max(real(eigenvalues)));
    end

    // Build system struct
    sys = struct();
    sys.A    = A;
    sys.B    = B;
    sys.C    = C;
    sys.D    = D;
    sys.nx   = n;
    sys.nu   = m;
    sys.ny   = p;
    sys.Ts   = 0;     
    sys.type = "ss";

endfunction
