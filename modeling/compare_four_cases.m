clear;
clc;
close all;

tau_cases = [ ...
     0  0  0;
    10  0  0;
     0 10  0;
     0  0 10];

routing = 1;

loading_steps_N = 1;
loading_steps_L = 1;

n_cases = size(tau_cases,1);

%% Warm-up execution
tau_warmup = tau_cases(1,:);

Newtonian( ...
    routing, ...
    1, ...
    tau_warmup, ...
    false, ...
    false);

Lagrangian( ...
    routing, ...
    1, ...
    tau_warmup, ...
    false, ...
    false);

%% ============================================================
% Benchmark settings
% =============================================================

n_repeats = 1;

% Tip positions
tip_N_all = zeros(3, n_cases);
tip_L_all = zeros(3, n_cases);

% Tip quaternions
Qtip_N_all = zeros(4, n_cases);
Qtip_L_all = zeros(4, n_cases);

% Tip rotation matrices
Rtip_N_all = zeros(3, 3, n_cases);
Rtip_L_all = zeros(3, 3, n_cases);

% Position and orientation errors
position_error_mm = zeros(n_cases, 1);
orientation_error_deg = zeros(n_cases, 1);

% Computation times
mean_time_N = zeros(n_cases, 1);
mean_time_L = zeros(n_cases, 1);

std_time_N = zeros(n_cases, 1);
std_time_L = zeros(n_cases, 1);

% Convergence information
converged_N = false(n_cases, 1);
converged_L = false(n_cases, 1);

residual_N = zeros(n_cases, 1);
residual_L = zeros(n_cases, 1);

iterations_N = zeros(n_cases, 1);
iterations_L = zeros(n_cases, 1);

% Loading steps actually used
loading_used_N = zeros(n_cases, 1);
loading_used_L = zeros(n_cases, 1);

% Backbone coordinates
rX_N_all = cell(n_cases, 1);
rX_L_all = cell(n_cases, 1);
%% ============================================================
% Run all four tension cases
% =============================================================

for case_idx = 1:n_cases

    tau = tau_cases(case_idx, :);

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('CASE %d / %d\n', case_idx, n_cases);
    fprintf('tau = [%.1f %.1f %.1f] N\n', ...
        tau(1), tau(2), tau(3));
    fprintf('============================================================\n');

    %% --------------------------------------------------------
    % Newtonian approach
    % ---------------------------------------------------------

    times_N = zeros(n_repeats, 1);

    for rep = 1:n_repeats

        [flag_N, ...
         rX_N, ...
         tip_N, ...
         ~, ...
         QX_N, ...
         time_N, ...
         final_residual_N, ...
         solver_iterations_N] = ...
            Newtonian( ...
                routing, ...
                loading_steps_N, ...
                tau, ...
                false, ...
                false);

        times_N(rep) = time_N;
    end

    % Store Newtonian results from final repetition
    tip_N_all(:, case_idx) = tip_N;

    Qtip_N = QX_N(:, end);
    Qtip_N_all(:, case_idx) = Qtip_N;

    Rtip_N = quat2rot(Qtip_N);
    Rtip_N_all(:, :, case_idx) = Rtip_N;

    rX_N_all{case_idx} = rX_N;

    mean_time_N(case_idx) = mean(times_N);
    std_time_N(case_idx) = std(times_N);

    converged_N(case_idx) = flag_N > 0;

    residual_N(case_idx) = final_residual_N;
    iterations_N(case_idx) = solver_iterations_N;

    loading_used_N(case_idx) = loading_steps_N;


    %% --------------------------------------------------------
    % Lagrangian approach
    % ---------------------------------------------------------

    times_L = zeros(n_repeats, 1);

    for rep = 1:n_repeats

        [converged_L_case, ...
         rX_L, ...
         tip_L, ...
         QX_L, ...
         time_L, ...
         final_residual_L, ...
         solver_iterations_L] = ...
            Lagrangian( ...
                routing, ...
                loading_steps_L, ...
                tau, ...
                false, ...
                false);

        times_L(rep) = time_L;
    end

    % Store Lagrangian results from final repetition
    tip_L_all(:, case_idx) = tip_L;

    Qtip_L = QX_L(:, end);
    Qtip_L_all(:, case_idx) = Qtip_L;

    Rtip_L = quat2rot(Qtip_L);
    Rtip_L_all(:, :, case_idx) = Rtip_L;

    rX_L_all{case_idx} = rX_L;

    mean_time_L(case_idx) = mean(times_L);
    std_time_L(case_idx) = std(times_L);

    converged_L(case_idx) = converged_L_case;

    residual_L(case_idx) = final_residual_L;
    iterations_L(case_idx) = solver_iterations_L;

    loading_used_L(case_idx) = loading_steps_L;


    %% --------------------------------------------------------
    % Position difference
    % ---------------------------------------------------------

    position_error_mm(case_idx) = ...
        norm(tip_N - tip_L) * 1000;


%% --------------------------------------------------------
% Orientation difference
% ---------------------------------------------------------

% Fixed reference-frame transformation used by the
% Lagrangian formulation
R0_L = [ ...
    0  0 -1;
    0  1  0;
    1  0  0];

% Convert the Lagrangian tip orientation to the same
% material-frame convention as the Newtonian formulation
Rtip_L_common = Rtip_L * R0_L';

% Relative physical tip orientation
R_relative = Rtip_N' * Rtip_L_common;

cos_theta = ...
    (trace(R_relative) - 1) / 2;

% Protect against numerical round-off
cos_theta = max(-1, min(1, cos_theta));

orientation_error_deg(case_idx) = ...
    rad2deg(acos(cos_theta));

end
%% ============================================================
% Display summary table
% =============================================================

fprintf('\n');
fprintf('=====================================================================================================================\n');
fprintf('                                   NEWTONIAN - LAGRANGIAN COMPARISON\n');
fprintf('=====================================================================================================================\n');
fprintf(' Tension [N]      Method       Tip x [mm]    Tip y [mm]    Tip z [mm]    Mean time [s]   Residual      Iterations   Conv.\n');
fprintf('---------------------------------------------------------------------------------------------------------------------\n');

for case_idx = 1:n_cases

    tau = tau_cases(case_idx, :);

    fprintf('[%2.0f %2.0f %2.0f]      Newtonian    %10.3f    %10.3f    %10.3f    %12.6f   %10.3e    %6d       %d\n', ...
        tau(1), tau(2), tau(3), ...
        tip_N_all(1,case_idx)*1000, ...
        tip_N_all(2,case_idx)*1000, ...
        tip_N_all(3,case_idx)*1000, ...
        mean_time_N(case_idx), ...
        residual_N(case_idx), ...
        iterations_N(case_idx), ...
        converged_N(case_idx));

    fprintf('[%2.0f %2.0f %2.0f]      Lagrangian   %10.3f    %10.3f    %10.3f    %12.6f   %10.3e    %6d       %d\n', ...
        tau(1), tau(2), tau(3), ...
        tip_L_all(1,case_idx)*1000, ...
        tip_L_all(2,case_idx)*1000, ...
        tip_L_all(3,case_idx)*1000, ...
        mean_time_L(case_idx), ...
        residual_L(case_idx), ...
        iterations_L(case_idx), ...
        converged_L(case_idx));

end
disp('DEBUG position errors:')
disp(position_error_mm)

disp('DEBUG orientation errors:')
disp(orientation_error_deg)

%% ============================================================
% Display position and orientation differences
% =============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('TIP DIFFERENCES\n');
fprintf('============================================================\n');
fprintf(' Tension [N]      Position error [mm]    Orientation error [deg]\n');
fprintf('------------------------------------------------------------\n');

for case_idx = 1:n_cases

    tau = tau_cases(case_idx, :);

    fprintf('[%2.0f %2.0f %2.0f]          %10.6f               %10.6f\n', ...
        tau(1), tau(2), tau(3), ...
        position_error_mm(case_idx), ...
        orientation_error_deg(case_idx));

end

fprintf('============================================================\n');

fprintf('\nLOADING STEPS USED\n');
fprintf('Tension [N]      Newtonian   Lagrangian\n');

for case_idx = 1:n_cases
    tau = tau_cases(case_idx,:);

    fprintf('[%2.0f %2.0f %2.0f]          %d            %d\n', ...
        tau(1), tau(2), tau(3), ...
        loading_used_N(case_idx), ...
        loading_used_L(case_idx));
end