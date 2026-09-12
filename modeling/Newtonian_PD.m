function [converged, tau_solution_N, rX, tip_position, ...
          obtained_pull_mm, history] = ...
    Newtonian_PD( ...
        routing, ...
        loading_steps, ...
        desired_pull_mm, ...
        tau_initial_N, ...
        active_tendon)
% Newtonian_PD
%
% Iterative displacement-driven solver for the static TDCR model.
%
% The existing Newtonian Cosserat model receives tendon tension [N].
% This outer PD controller changes the tension guess until the simulated
% tendon displacement is equal to the desired tendon displacement.
%
% Inputs:
%   routing          : tendon routing number
%   loading_steps    : loading steps used by Newtonian.m
%   desired_pull_mm  : desired tendon displacements [mm]
%   tau_initial_N    : initial tendon-tension guess [N]
%   active_tendon    : tendon controlled by the PD loop
%
% Example:
%
%   desired_pull_mm = [0 0 10];
%   tau_initial_N   = [0 0 5];
%   active_tendon   = 3;
%
% Outputs:
%   converged          : true when displacement error reaches tolerance
%   tau_solution_N     : calculated tendon tensions [N]
%   rX                 : final simulated backbone shape [m]
%   tip_position       : final simulated tip position [m]
%   obtained_pull_mm   : final simulated tendon displacements [mm]
%   history            : controller iteration history

    %% ============================================================
    % Read robot configuration
    % =============================================================

    [~, ~, ~, n_segments, n_tendons, ~] = ...
        tendons(routing);

    if n_segments ~= 1
        error(['This first PD-controller version is prepared for ', ...
               'a one-segment robot.']);
    end

    number_of_tendons = n_tendons(1);

    %% ============================================================
    % Format and check inputs
    % =============================================================

    desired_pull_mm = double(desired_pull_mm(:).');
    tau_initial_N = double(tau_initial_N(:).');

    if numel(desired_pull_mm) ~= number_of_tendons

        error(['desired_pull_mm must contain %d values.'], ...
            number_of_tendons);
    end

    if numel(tau_initial_N) ~= number_of_tendons

        error(['tau_initial_N must contain %d values.'], ...
            number_of_tendons);
    end

    if active_tendon < 1 || ...
            active_tendon > number_of_tendons

        error('active_tendon is outside the valid tendon range.');
    end

    if any(~isfinite(desired_pull_mm))

        error('desired_pull_mm must contain finite values.');
    end

    if any(~isfinite(tau_initial_N))

        error('tau_initial_N must contain finite values.');
    end

    if desired_pull_mm(active_tendon) < 0

        error(['The desired pull for the active tendon must be ', ...
               'nonnegative.']);
    end

    if any(tau_initial_N < 0)

        error('Initial tendon tensions must be nonnegative.');
    end

    %% ============================================================
    % PD-controller parameters
    % =============================================================

    % Proportional gain [N/mm]
    Kp = 0.25;

    % Derivative gain [N/mm per iteration]
    Kd = 0.05;

    % Accepted tendon-displacement error [mm]
    tolerance_mm = 0.02;

    % Maximum number of controller iterations
    maximum_iterations = 40;

    % Maximum allowed tendon tension [N]
    maximum_tension_N = 10;

    % Limit the tension change during one iteration [N]
    maximum_tension_change_N = 1.0;

    %% ============================================================
    % Calculate reference tendon lengths
    % =============================================================

    % Reference configuration:
    % zero applied tendon tension
    tau_zero_N = zeros(1, number_of_tendons);

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('CALCULATING ZERO-TENSION REFERENCE\n');
    fprintf('============================================================\n');

    [flag_zero, ~, ~, tendon_length_zero_m] = ...
        Newtonian( ...
            routing, ...
            loading_steps, ...
            tau_zero_N, ...
            false, ...
            false);

    if flag_zero <= 0

        error('The zero-tension reference simulation did not converge.');
    end

    fprintf('Reference tendon lengths [mm]:\n');

    fprintf('[%s]\n', ...
        num2str( ...
            tendon_length_zero_m*1000, ...
            ' %.4f'));

    %% ============================================================
    % Initialize controller
    % =============================================================

    tau_guess_N = tau_initial_N;

    previous_error_mm = 0;

    converged = false;

    rX = [];
    tip_position = [];
    obtained_pull_mm = zeros(1, number_of_tendons);

    %% Controller history

    history.iteration = ...
        zeros(maximum_iterations, 1);

    history.tension_N = ...
        zeros(maximum_iterations, number_of_tendons);

    history.pull_mm = ...
        zeros(maximum_iterations, number_of_tendons);

    history.error_mm = ...
        zeros(maximum_iterations, number_of_tendons);

    %% ============================================================
    % Start iterative PD controller
    % =============================================================

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('STARTING ITERATIVE PD CONTROLLER\n');
    fprintf('============================================================\n');

    fprintf('Active tendon       : %d\n', active_tendon);

    fprintf('Desired displacement: %.4f mm\n', ...
        desired_pull_mm(active_tendon));

    fprintf('Initial tension     : %.4f N\n', ...
        tau_guess_N(active_tendon));

    fprintf('Kp                  : %.4f N/mm\n', Kp);
    fprintf('Kd                  : %.4f N/mm per iteration\n', Kd);
    fprintf('Tolerance           : %.4f mm\n', tolerance_mm);

    fprintf('============================================================\n\n');

    final_iteration = maximum_iterations;

    for iteration = 1:maximum_iterations

        %% --------------------------------------------------------
        % Step 1: Solve Cosserat model using current tension guess
        % ---------------------------------------------------------

        [flag, rX_current, tip_current, tendon_length_loaded_m] = ...
            Newtonian( ...
                routing, ...
                loading_steps, ...
                tau_guess_N, ...
                false, ...
                false);

        if flag <= 0

            error(['Newtonian solver failed during PD iteration %d.'], ...
                iteration);
        end

        %% --------------------------------------------------------
        % Step 2: Calculate equivalent tendon displacement
        % ---------------------------------------------------------

        obtained_pull_mm = ...
            (tendon_length_zero_m ...
            - tendon_length_loaded_m)*1000;

        %% --------------------------------------------------------
        % Step 3: Calculate tendon-displacement error
        % ---------------------------------------------------------

        error_vector_mm = ...
            desired_pull_mm - obtained_pull_mm;

        active_error_mm = ...
            error_vector_mm(active_tendon);

        %% Save current solution

        rX = rX_current;
        tip_position = tip_current;

        %% Save controller history

        history.iteration(iteration) = iteration;

        history.tension_N(iteration, :) = ...
            tau_guess_N;

        history.pull_mm(iteration, :) = ...
            obtained_pull_mm;

        history.error_mm(iteration, :) = ...
            error_vector_mm;

        %% Display current iteration

        fprintf('Iteration %02d\n', iteration);

        fprintf('  Tension guess [N] = [%s]\n', ...
            num2str(tau_guess_N, ' %.4f'));

        fprintf('  Obtained pull [mm]= [%s]\n', ...
            num2str(obtained_pull_mm, ' %.4f'));

        fprintf('  Active error [mm] = %.6f\n', ...
            active_error_mm);

        %% --------------------------------------------------------
        % Step 4: Check convergence
        % ---------------------------------------------------------

        if abs(active_error_mm) <= tolerance_mm

            converged = true;
            final_iteration = iteration;

            fprintf('  Status            = CONVERGED\n\n');

            break;
        end

        fprintf('  Status            = correcting tension\n\n');

        %% --------------------------------------------------------
        % Step 5: Calculate derivative error
        % ---------------------------------------------------------

        if iteration == 1

            % Avoid derivative kick during first iteration
            derivative_error_mm = 0;

        else

            derivative_error_mm = ...
                active_error_mm - previous_error_mm;
        end

        %% --------------------------------------------------------
        % Step 6: PD controller output
        % ---------------------------------------------------------

        tension_correction_N = ...
            Kp*active_error_mm ...
            + Kd*derivative_error_mm;

        %% Limit the correction to avoid large jumps

        tension_correction_N = max( ...
            -maximum_tension_change_N, ...
            min( ...
                maximum_tension_change_N, ...
                tension_correction_N));

        %% --------------------------------------------------------
        % Step 7: Update only the active tendon tension
        % ---------------------------------------------------------

        tau_guess_N(active_tendon) = ...
            tau_guess_N(active_tendon) ...
            + tension_correction_N;

        %% Tendons cannot push

        tau_guess_N(active_tendon) = max( ...
            0, ...
            tau_guess_N(active_tendon));

        %% Apply maximum tension safety limit

        tau_guess_N(active_tendon) = min( ...
            maximum_tension_N, ...
            tau_guess_N(active_tendon));

        %% Store error for derivative term

        previous_error_mm = active_error_mm;
    end

    %% ============================================================
    % Trim unused history rows
    % =============================================================

    history.iteration = ...
        history.iteration(1:final_iteration);

    history.tension_N = ...
        history.tension_N(1:final_iteration, :);

    history.pull_mm = ...
        history.pull_mm(1:final_iteration, :);

    history.error_mm = ...
        history.error_mm(1:final_iteration, :);

    %% ============================================================
    % Final tension solution
    % =============================================================

    tau_solution_N = ...
        history.tension_N(end, :);

    %% ============================================================
    % Run and display final Newtonian solution
    % =============================================================

    [flag_final, rX, tip_position, tendon_length_final_m] = ...
        Newtonian( ...
            routing, ...
            loading_steps, ...
            tau_solution_N, ...
            false, ...
            true);

    if flag_final <= 0

        error('The final Newtonian simulation did not converge.');
    end

    obtained_pull_mm = ...
        (tendon_length_zero_m ...
        - tendon_length_final_m)*1000;

    final_error_mm = ...
        desired_pull_mm(active_tendon) ...
        - obtained_pull_mm(active_tendon);

    %% ============================================================
    % Display final controller results
    % =============================================================

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('PD CONTROLLER FINAL RESULTS\n');
    fprintf('============================================================\n');

    if converged

        fprintf('Controller status: CONVERGED\n');

    else

        fprintf('Controller status: NOT CONVERGED\n');
    end

    fprintf('Number of iterations: %d\n', final_iteration);

    fprintf('Final tendon tensions [N]:\n');
    fprintf('[%s]\n', ...
        num2str(tau_solution_N, ' %.6f'));

    fprintf('\nDesired tendon pulls [mm]:\n');
    fprintf('[%s]\n', ...
        num2str(desired_pull_mm, ' %.6f'));

    fprintf('\nObtained tendon pulls [mm]:\n');
    fprintf('[%s]\n', ...
        num2str(obtained_pull_mm, ' %.6f'));

    fprintf('\nActive-tendon displacement error:\n');
    fprintf('%.6f mm\n', final_error_mm);

    fprintf('\nFinal simulated tip position [mm]:\n');
    fprintf('[%.6f %.6f %.6f]\n', ...
        tip_position(1)*1000, ...
        tip_position(2)*1000, ...
        tip_position(3)*1000);

    fprintf('============================================================\n');

%% ============================================================
% Plot displacement-error convergence
% =============================================================

figure;

plot( ...
    history.iteration, ...
    history.error_mm(:, active_tendon), ...
    '-o', ...
    'LineWidth', 1.5);

grid on;

xlabel('PD iteration');
ylabel('Displacement error [mm]');
title('PD displacement-error convergence');


%% ============================================================
% Plot tendon-tension convergence
% =============================================================

figure;

plot( ...
    history.iteration, ...
    history.tension_N(:, active_tendon), ...
    '-o', ...
    'LineWidth', 1.5);

grid on;

xlabel('PD iteration');
ylabel('Tendon tension [N]');
title('PD tendon-tension convergence');
end