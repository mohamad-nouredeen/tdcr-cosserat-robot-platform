function [flag, rX, tip_position, tendon_length, ...
          QX, time, final_residual, solver_iterations] = ...
    Newtonian(routing, ls, tau_tot, save_true, show_true)
% Newtonian
%
% Force-driven static Cosserat-rod solver for a tendon-driven
% continuum robot.
%
% Inputs:
%   routing     : tendon-routing configuration
%   ls          : number of gradual loading steps
%   tau_tot     : tendon tensions [N]
%   save_true   : save simulation results
%   show_true   : display and plot results
%
% Outputs:
%   flag           : fsolve exit flag
%   rX             : complete backbone coordinates [m]
%   tip_position   : final backbone tip position [m]
%   tendon_length  : geometric length of each tendon [m]

    %% ============================================================
    % Initialize outputs
    % =============================================================

    rX = [];
    tip_position = [];
    tendon_length = [];
    QX = [];
    time = NaN;
    final_residual = NaN;
    solver_iterations = 0;

    if nargin < 4
        save_true = true;
    end

    if nargin < 5
        show_true = true;
    end

    addpath('./tools');

    %% ============================================================
    % Independent robot parameters
    % =============================================================

    [E, G, Ro, Ri, rho] = materials();

    % Gravity acceleration [m/s^2]
    grav = [0; 0; -9.81];

    [~, ~, ~, n_segments, n_tendons, l_j] = ...
        tendons(routing);

    %% ============================================================
    % Check inputs
    % =============================================================

    if ls < 1 || round(ls) ~= ls
        error('ls must be a positive integer.');
    end

    tau_tot = double(tau_tot);

    % For the current one-segment robot, make tau_tot a row vector.
    if n_segments == 1

        tau_tot = tau_tot(:).';

        if numel(tau_tot) ~= n_tendons(1)

            error(['tau_tot must contain %d tendon tensions ', ...
                   'for routing %d.'], ...
                   n_tendons(1), routing);
        end
    end

    if any(~isfinite(tau_tot), 'all')
        error('All tendon-tension values must be finite.');
    end

    if any(tau_tot < 0, 'all')
        error('Tendon tensions must be nonnegative.');
    end

    %% ============================================================
    % Boundary conditions
    % =============================================================

    r0 = zeros(n_segments, 3);

    Q0 = [rot2quat(eye(3))';
          zeros(n_segments - 1, 4)];

    %% ============================================================
    % Hollow circular backbone properties
    % =============================================================

    area = pi*(Ro^2 - Ri^2);

    % Second moment of area
    I = (pi/4)*(Ro^4 - Ri^4);

    % Polar second moment
    J = (pi/2)*(Ro^4 - Ri^4);

    % Shear and extension stiffness
    Kse = diag([ ...
        G*area, ...
        G*area, ...
        E*area]);

    % Bending and torsional stiffness
    Kbt = diag([ ...
        E*I, ...
        E*I, ...
        G*J]);

    %% ============================================================
    % Shooting-method variables
    % =============================================================

    init_guess = zeros(6, 1);

    % Y{j}: state solution for segment j
    % S{j}: spatial coordinates for segment j
    %
    % These variables are shared with the nested functions.
    Y = cell(n_segments, 1);
    S = cell(n_segments, 1);

    % Current segment index used by TACR_ODE
    j = 1;

    % Current gradually applied tension
    tau = zeros(size(tau_tot));

    %% ============================================================
    % Solve using gradual loading
    % =============================================================

    flag = 1;
    t = 1;

    opt = optimoptions( ...
    'fsolve', ...
    'Algorithm', 'levenberg-marquardt', ...
    'Display', 'off', ...
    'FunctionTolerance', 1e-8, ...
    'StepTolerance', 1e-8);

    tic;

    while flag > 0 && t <= ls

        % Gradually apply tendon tensions
        tau = t*tau_tot/ls;

        [init_guess, ~, flag, output] = ...
    fsolve(@shooting_fun, init_guess, opt);

solver_iterations = ...
    solver_iterations + output.iterations;

        t = t + 1;
    end

    time = toc;

    %% ============================================================
    % Check solver result
    % =============================================================

    if flag <= 0

        warning('The Newtonian solver did not exit properly.');
        return;
    end

    % Evaluate the shooting function one final time using the converged
    % initial conditions. This ensures Y and S correspond exactly to
    % the final fsolve solution.
    final_error_vector = shooting_fun(init_guess);
final_residual = norm(final_error_vector);

    %% ============================================================
    % Calculate tendon lengths
    % =============================================================

    % This must be calculated while Y and S are still cell arrays.
    tendon_length = calculateTendonLengths();

    %% ============================================================
    % Assemble the complete backbone solution
    % =============================================================

    l = sum(l_j);

    % Number of nodes per segment
    n_nodes = size(Y{1}, 1);

    % Convert the segment solutions into one matrix.
    %
    % Rows after transposition:
    %   1:3   = position
    %   4:7   = quaternion
    %   8:10  = curvature/twist
    %   11:13 = shear/extension
    Ymat = cell2mat(Y)';

  % Complete backbone coordinates
rX = Ymat(1:3, :);

% Complete backbone orientations
QX = Ymat(4:7, :);

% Final backbone tip position
tip_position = rX(:, end);

    %% ============================================================
    % Display results
    % =============================================================

    if show_true

        fprintf('\n');
        fprintf('============================================================\n');
        fprintf('FINAL BACKBONE TIP POSITION\n');
        fprintf('============================================================\n');

        fprintf('x_tip = %.9f m = %.3f mm\n', ...
            tip_position(1), ...
            tip_position(1)*1000);

        fprintf('y_tip = %.9f m = %.3f mm\n', ...
            tip_position(2), ...
            tip_position(2)*1000);

        fprintf('z_tip = %.9f m = %.3f mm\n', ...
            tip_position(3), ...
            tip_position(3)*1000);

        fprintf('\nTip-position vector [m]:\n');
        disp(tip_position);

        fprintf('Tip-position vector [mm]:\n');
        disp(tip_position*1000);

        fprintf('============================================================\n');
        fprintf('TENDON LENGTHS\n');
        fprintf('============================================================\n');

        for tendon_index = 1:numel(tendon_length)

            fprintf('Tendon %d: %.9f m = %.3f mm\n', ...
                tendon_index, ...
                tendon_length(tendon_index), ...
                tendon_length(tendon_index)*1000);
        end

        fprintf('============================================================\n\n');

        %% Plot complete backbone shape

        plot_TACR( ...
            rX, ...
            n_segments, ...
            l, ...
            n_nodes, ...
            'Newtonian');
    end

   %% ============================================================
% Save results
% =============================================================

if save_true

    rl_j = Ymat(1:3, end);
    Ql_j = Ymat(4:7, end);

    xi0 = Ymat(8:13, 1);

    save_TACR( ...
        routing, ...
        rl_j, ...
        Ql_j, ...
        rX, ...
        QX, ...
        xi0, ...
        time, ...
        tau, ...
        ls);
end

    %% ============================================================
    % Cosserat-rod differential equations
    % =============================================================

    function dy = TACR_ODE(X, y)

    %% ============================================================
    % Unpack the state vector
    % =============================================================

    Q = y(4:7);
    K = y(8:10);
    Gstrain = y(11:13);

    % Force all state variables to be column vectors
    Q = Q(:);
    K = K(:);
    Gstrain = Gstrain(:);

    R = quat2rot(Q);

    %% ============================================================
    % Initialize tendon terms
    % =============================================================

    a = zeros(3, 1);
    b = zeros(3, 1);

    A = zeros(3, 3);
    O = zeros(3, 3);
    H = zeros(3, 3);

    %% ============================================================
    % Tendon contributions
    % =============================================================

    for jj = j:n_segments

        for i = 1:n_tendons(jj)

            % Global tendon index
            it = i + sum(n_tendons(1:jj - 1));

            % Tendon routing position and derivatives
            [D, dD, ddD] = ...
                tendons(routing, j, it, X);

            % Force routing data to be column vectors
            D = D(:);
            dD = dD(:);
            ddD = ddD(:);

            % Local tendon tangent
            G_i = ...
                cross(K, D) ...
                + dD ...
                + Gstrain;

            G_i_norm = norm(G_i);

            % Protection against division by zero
            if G_i_norm < 1e-12
                error('The local tendon tangent norm is nearly zero.');
            end

            A_i = ...
                -hat(G_i)^2 * ...
                (tau(jj, i)/G_i_norm^3);

            O_i = -A_i*hat(D);

            a_i = A_i * ...
                (cross(K, G_i + dD) + ddD);

            a = a + a_i;
            b = b + cross(D, a_i);

            A = A + A_i;
            O = O + O_i;
            H = H + hat(D)*O_i;
        end
    end

    %% ============================================================
    % Backbone constitutive equations
    % =============================================================

    Mat = [ ...
        H + Kbt, O.';
        O,        A + Kse];

    N = Kse*(Gstrain - [0; 0; 1]);

    C = Kbt*K;

    %% ============================================================
    % Static equilibrium equations
    % =============================================================

    rhs_moment = ...
        -cross(K, C) ...
        -cross(Gstrain, N) ...
        -b;

    rhs_force = ...
        -cross(K, N) ...
        -R.'*rho*area*grav ...
        -a;

    % Force both parts to be column vectors
    rhs_moment = rhs_moment(:);
    rhs_force = rhs_force(:);

    rhs = [rhs_moment; rhs_force];

    %% ============================================================
    % Calculate state derivatives
    % =============================================================

    % Backbone-position derivative
    dr = R*Gstrain;

    % Quaternion derivative
    dQ = quaternion_dot_q_omega(Q, K);

    % Curvature and shear/extension derivatives
    dxi = Mat\rhs;

    % Force every derivative to be a column vector
    dr = dr(:);
    dQ = dQ(:);
    dxi = dxi(:);

    %% ============================================================
    % Pack the complete derivative vector
    % =============================================================

    dy = [dr; dQ; dxi];

    % The state has:
    %   position       = 3
    %   quaternion     = 4
    %   K and Gstrain  = 6
    %
    % Total            = 13 states
    if numel(dy) ~= 13

        error(['TACR_ODE returned %d derivatives instead of 13. ', ...
               'Sizes: dr=%s, dQ=%s, dxi=%s.'], ...
               numel(dy), ...
               mat2str(size(dr)), ...
               mat2str(size(dQ)), ...
               mat2str(size(dxi)));
    end
end

    %% ============================================================
    % Shooting objective function
    % =============================================================

    function distal_error = shooting_fun(guess)

        distal_error = zeros(6, 1);

        for j = 1:n_segments

            %% Initial conditions for current segment

            if j == 1

                % Guess base force and curvature
                N0_j = guess(1:3);

                G0_j = ...
                    Kse\N0_j + [0; 0; 1];

                K0_j = guess(4:6);

            else

                % Propagate internal loads from previous segment
                G0_j = ...
                    Kse\N0_jp1 + [0; 0; 1];

                K0_j = ...
                    Kbt\C0_jp1;
            end

            y0 = [ ...
                r0(j, :)';
                Q0(j, :)';
                K0_j;
                G0_j];

            %% Integrate along the current segment

            op = odeset( ...
                'RelTol', 1e-8, ...
                'AbsTol', 1e-8);

            % Store both the spatial coordinates and the state solution.
            [S{j}, Y{j}] = ode45( ...
                @TACR_ODE, ...
                linspace(0, l_j(j)), ...
                y0, ...
                op);

            %% Internal force and moment before the distal plate

            Kl_j = Y{j}(end, 8:10).';
            Gl_j = Y{j}(end, 11:13).';

            %% Initial states for the next segment

            if j < n_segments

                r0(j + 1, :) = ...
                    Y{j}(end, 1:3);

                Q0(j + 1, :) = ...
                    Y{j}(end, 4:7);
            end

            Nl_j = ...
                Kse*(Gl_j - [0; 0; 1]);

            Cl_j = ...
                Kbt*Kl_j;

            %% Forces from tendons ending in current segment

            for i = 1:n_tendons(j)

                it = ...
                    i + sum(n_tendons(1:j - 1));

                [D, dD] = ...
                    tendons( ...
                        routing, ...
                        j, ...
                        it, ...
                        l_j(j));

                G_i = ...
                    cross(Kl_j, D) ...
                    + dD ...
                    + Gl_j;

                Fb_i = ...
                    -tau(j, i)*G_i/norm(G_i);

                Nl_j = Nl_j - Fb_i;

                Cl_j = ...
                    Cl_j - cross(D, Fb_i);
            end

            %% Distal equilibrium or segment propagation

            if j == n_segments

                % Final segment boundary condition
                distal_error = -[Nl_j; Cl_j];

            else

                % Routing-shift force and moment
                FtX = zeros(3, 1);
                LtX = zeros(3, 1);

                a = zeros(3, 1);
                b = zeros(3, 1);

                A = zeros(3, 3);
                O = zeros(3, 3);
                H = zeros(3, 3);

                %% Calculate state discontinuity at segment boundary

                for jj = j:n_segments

                    for i = 1:n_tendons(jj)

                        it = ...
                            i + sum(n_tendons(1:jj - 1));

                        [D, dD_j_m] = ...
                            tendons( ...
                                routing, ...
                                j, ...
                                it, ...
                                l_j(j));

                        [~, dD_j_p] = ...
                            tendons( ...
                                routing, ...
                                j + 1, ...
                                it, ...
                                0);

                        G_i = ...
                            cross(Kl_j, D) ...
                            + Gl_j;

                        G_i_norm = norm(G_i);

                        A_i = ...
                            -hat(G_i)^2 * ...
                            (tau(jj, i)/G_i_norm^3);

                        O_i = -A_i*hat(D);

                        a_i = ...
                            A_i*(dD_j_p - dD_j_m);

                        a = a + a_i;
                        b = b + cross(D, a_i);

                        A = A + A_i;
                        O = O + O_i;
                        H = H + hat(D)*O_i;
                    end
                end

                Mat = [ ...
                    H + Kbt, O.';
                    O,        A + Kse];

                rhs = [ ...
                    -b;
                    -a];

                delta_xi = -Mat\rhs;

                Kl_j_p = ...
                    Kl_j + delta_xi(1:3);

                Gl_j_p = ...
                    Gl_j + delta_xi(4:6);

                %% Tendon forces at the segment transition

                for jj = j + 1:n_segments

                    for i = 1:n_tendons(jj)

                        it = ...
                            i + sum(n_tendons(1:jj - 1));

                        [D, dD_j_p] = ...
                            tendons( ...
                                routing, ...
                                j + 1, ...
                                it, ...
                                0);

                        [~, dD_j_m] = ...
                            tendons( ...
                                routing, ...
                                j, ...
                                it, ...
                                l_j(j));

                        if any(dD_j_p ~= dD_j_m)

                            G_i_p = ...
                                hat(Kl_j_p)*D ...
                                + dD_j_p ...
                                + Gl_j_p;

                            G_i_m = ...
                                hat(Kl_j)*D ...
                                + dD_j_m ...
                                + Gl_j;

                            Ci = ...
                                G_i_p/norm(G_i_p) ...
                                -G_i_m/norm(G_i_m);

                            F = Ci*tau(jj, i);

                            FtX = FtX + F;
                            LtX = LtX + hat(D)*F;
                        end
                    end
                end

                %% Loads propagated to the next segment

                N0_jp1 = Nl_j - FtX;
                C0_jp1 = Cl_j - LtX;
            end
        end
    end

    %% ============================================================
    % Calculate geometric tendon lengths
    % =============================================================

    function lengths = calculateTendonLengths()
    % calculateTendonLengths
    %
    % Calculates the geometric length of every tendon from the
    % converged Cosserat solution.
    %
    % The local tendon tangent in the body frame is:
    %
    %   G_tendon = Gstrain + cross(K,D) + dD
    %
    % The tendon length is:
    %
    %   L_tendon = integral(norm(G_tendon), ds)
    %
    % Output:
    %   lengths = tendon lengths [m]

        total_tendons = sum(n_tendons);

        lengths = zeros(1, total_tendons);

        % Used to determine the segment in which each tendon terminates
        cumulative_tendons = cumsum(n_tendons);

        for tendon_index = 1:total_tendons

            % Find the segment where this tendon terminates
            terminal_segment = find( ...
                tendon_index <= cumulative_tendons, ...
                1, ...
                'first');

            % A tendon ending in terminal_segment passes through all
            % preceding segments.
            for segment_index = 1:terminal_segment

                s_segment = S{segment_index};
                y_segment = Y{segment_index};

                number_of_nodes = numel(s_segment);

                local_length_rate = ...
                    zeros(number_of_nodes, 1);

                for node_index = 1:number_of_nodes

                    s_current = ...
                        s_segment(node_index);

                    % Curvature and twist strain
                    K_current = ...
                        y_segment(node_index, 8:10).';

                    % Shear and extension strain
                    G_current = ...
                        y_segment(node_index, 11:13).';

                    % Tendon routing position and derivative
                    [D_current, dD_current] = ...
                        tendons( ...
                            routing, ...
                            segment_index, ...
                            tendon_index, ...
                            s_current);

                    % Local tendon tangent in the body frame
                    G_tendon = ...
                        cross(K_current, D_current) ...
                        + dD_current ...
                        + G_current;

                    % Tendon length per unit backbone coordinate
                    local_length_rate(node_index) = ...
                        norm(G_tendon);
                end

                % Integrate over the current segment
                lengths(tendon_index) = ...
                    lengths(tendon_index) ...
                    + trapz( ...
                        s_segment, ...
                        local_length_rate);
            end
        end
    end
end
