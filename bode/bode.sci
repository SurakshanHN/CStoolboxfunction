// bode.sci — Frequency Response of Continuous-Time State-Space Systems
function [mag, phase, w] = bode(sys, w)

    // Argument Handling 
    nargs = argn(2);
    nlhs  = argn(1);

    // Validate system
    if typeof(sys) <> "st" then
        error("bode: sys must be a struct created by ss()");
    end

    fields = ["A","B","C","D","nx","nu","ny"];
    for i = 1:length(fields)
        if ~isfield(sys, fields(i)) then
            error("bode: invalid system struct (missing fields)");
        end
    end

    A = sys.A; B = sys.B; C = sys.C; D = sys.D;
    n = sys.nx; m = sys.nu; p = sys.ny;

    // Optional: enforce continuous-time systems only
    if isfield(sys, "type") then
        if sys.type <> "continuous" then
            error("bode: only continuous-time systems supported");
        end
    end

    // Frequency vector 
    if nargs < 2 | w == [] then
        w = _auto_freq(A);
    else
        if typeof(w) <> "constant" then
            error("bode: w must be numeric");
        end
        w = w(:);
        if min(w) <= 0 then
            error("bode: frequencies must be > 0");
        end
    end

    Nw = length(w);
    I = eye(n, n);

    // Preallocation
    mag   = zeros(Nw, p, m);  
    phase = zeros(Nw, p, m);

    // Core computation
    for k = 1:Nw
        jw = %i * w(k);
        M = jw * I - A;

        rc = rcond(M);
        if rc < 1e-10 then
            mprintf("Warning: ill-conditioned matrix at w = %.3e (rcond=%.2e)\n", w(k), rc);
        end

        G = C * (M \ B) + D;

        for i = 1:p
            for j = 1:m
                gij = G(i,j);

                mag(k,i,j) = abs(gij);

                phase(k,i,j) = atan(imag(gij), real(gij)) * 180/%pi;
            end
        end
    end

    // Phase unwrap 
    for i = 1:p
        for j = 1:m
            phase(:,i,j) = _unwrap(phase(:,i,j));
        end
    end

    if nlhs == 0 then
        _plot_bode(w, mag, phase, p, m);
    end

endfunction

function w = _auto_freq(A)

    if size(A,1) == 0 then
        w = logspace(-2, 2, 500)';
        return;
    end

    ev = spec(A);
    ev_mag = abs(ev);

    
    has_zero = or(ev_mag < 1e-6);

    ev_mag_nz = ev_mag(ev_mag > 1e-8);

    if isempty(ev_mag_nz) then
        
        w = logspace(-4, 2, 500)';
        return;
    end

    w_min = min(ev_mag_nz);
    w_max = max(ev_mag_nz);

    log_lo = log10(w_min) - 2;
    log_hi = log10(w_max) + 2;

    // Expand range if integrator present
    if has_zero then
        log_lo = log_lo - 2;
    end

    // Ensure reasonable span
    if (log_hi - log_lo) < 4 then
        mid = (log_lo + log_hi)/2;
        log_lo = mid - 2;
        log_hi = mid + 2;
    end

    w = logspace(log_lo, log_hi, 500)';
endfunction


// PHASE UNWRAP (IMPROVED — handles large jumps robustly)
function phi = _unwrap(phi)

    for k = 2:length(phi)
        dp = phi(k) - phi(k-1);

        // Normalize jump to [-180, 180]
        dp = modulo(dp + 180, 360) - 180;

        phi(k) = phi(k-1) + dp;
    end

endfunction

// PLOT FUNCTION (SAFE LOG HANDLING)
function _plot_bode(w, mag, phase, p, m)

    scf();
    clf();

    mag_safe = max(mag, 1e-16);
    mag_db = 20 * log10(mag_safe);

    if p == 1 & m == 1 then

        subplot(2,1,1);
        plot2d("ln", w, mag_db);
        xgrid();
        xtitle("Bode Plot", "", "Magnitude (dB)");
        gca().log_flags = "lnn";

        subplot(2,1,2);
        plot2d("ln", w, phase);
        xgrid();
        xtitle("", "Frequency (rad/s)", "Phase (deg)");
        gca().log_flags = "lnn";

    else
        idx = 0;
        for i = 1:p
            for j = 1:m
                idx = idx + 1;

                subplot(2*p, m, (i-1)*m + j);
                plot2d("ln", w, mag_db(:,i,j));
                xtitle(msprintf("Mag: y%d/u%d", i, j), "", "");

                subplot(2*p, m, p*m + (i-1)*m + j);
                plot2d("ln", w, phase(:,i,j));
                xtitle(msprintf("Phase: y%d/u%d", i, j), "", "");
            end
        end
    end

endfunction