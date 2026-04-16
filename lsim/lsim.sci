
// lsim.sci — Time-Domain Simulation of Linear State-Space Systems

function [y, x] = lsim(sys, u, t, x0, method, input_mode)

    //  Argument count 
    nargs = argn(2);
    if nargs < 3 then
        error("lsim: usage -> [y,x] = lsim(sys, u, t, ...)");
    end

    //  Validate system 
    if typeof(sys) <> "st" then
        error("lsim: sys must be a struct created by ss()");
    end

    fields = ["A","B","C","D","nx","nu","ny"];
    for i = 1:length(fields)
        if ~isfield(sys, fields(i)) then
            error("lsim: invalid system struct (missing fields)");
        end
    end

    A = sys.A; B = sys.B; C = sys.C; D = sys.D;
    n = sys.nx; m = sys.nu; p = sys.ny;

    // Optional: enforce continuous-time
    if isfield(sys, "type") then
        if sys.type <> "continuous" then
            error("lsim: only continuous-time systems supported");
        end
    end

    //  Dimension checks 
    [Ar, Ac] = size(A);
    [Br, Bc] = size(B);
    [Cr, Cc] = size(C);
    [Dr, Dc] = size(D);

    if Ar <> Ac then error("A must be square"); end
    if Br <> Ar then error("B row mismatch"); end
    if Cc <> Ac then error("C column mismatch"); end
    if Dr <> Cr | Dc <> Bc then error("D dimension mismatch"); end

    //  Time vector 
    t = t(:);
    N = length(t);

    if N < 2 then error("t must have >= 2 points"); end

    dt_vec = diff(t);
    if min(dt_vec) <= 0 then
        error("t must be strictly increasing");
    end

    //  Input u 
    if typeof(u) <> "constant" then
        error("u must be numeric");
    end

    // Handle SISO flexibility
    [ur, uc] = size(u);

    if ur == 1 & uc == N & m == 1 then
        u = u(:);
    end

    [ur, uc] = size(u);
    if ur <> N then
        error("u must have same number of rows as t");
    end
    if uc <> m then
        error("u column count must match number of inputs");
    end

    //  Initial state 
    if nargs < 4 | x0 == [] then
        x0 = zeros(n,1);
    end

    x0 = x0(:);
    if length(x0) <> n then
        error("x0 dimension mismatch");
    end

    //  Method 
    if nargs < 5 then
        method = "rk4";
    end
    method = convstr(method, "l");

    if method <> "euler" & method <> "rk4" then
        error("method must be 'euler' or 'rk4'");
    end

    //  Input mode 
    if nargs < 6 then
        input_mode = "zoh";
    end
    input_mode = convstr(input_mode, "l");

    if input_mode <> "zoh" & input_mode <> "linear" then
        error("input_mode must be 'zoh' or 'linear'");
    end

    //  Preallocate 
    x = zeros(N, n);
    y = zeros(N, p);

    x(1,:) = x0';
    y(1,:) = (C*x0 + D*u(1,:)')';

    // Simulation 
    select method

    // Euler 
    case "euler"
        for k = 1:(N-1)
            dt = t(k+1) - t(k);

            xk = x(k,:)';
            uk = u(k,:)';

            dx = A*xk + B*uk;
            x_next = xk + dt*dx;

            x(k+1,:) = x_next';
            y(k+1,:) = (C*x_next + D*u(k+1,:)')';
        end

    // RK4
    case "rk4"
        for k = 1:(N-1)
            dt = t(k+1) - t(k);

            xk  = x(k,:)';
            uk  = u(k,:)';
            uk1 = u(k+1,:)';

            // Input model
            select input_mode
            case "zoh"
                u1 = uk;
                u2 = uk;
                u3 = uk;
                u4 = uk;

            case "linear"
                um = 0.5*(uk + uk1);
                u1 = uk;
                u2 = um;
                u3 = um;
                u4 = uk1;
            end

            // RK4 steps
            k1 = A*xk + B*u1;
            k2 = A*(xk + 0.5*dt*k1) + B*u2;
            k3 = A*(xk + 0.5*dt*k2) + B*u3;
            k4 = A*(xk + dt*k3)     + B*u4;

            x_next = xk + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

            x(k+1,:) = x_next';
            y(k+1,:) = (C*x_next + D*uk1)';
        end

    end

endfunction