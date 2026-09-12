function TDCR_DynamicModel(Tmax_current, filename, show_animation)

    clc;

    if nargin < 1
        Tmax_current = [10; 0; 0];   % default tendon input
    end

    if nargin < 2
        filename = 'tdcr_full_shape_dataset.mat';
    end

    if nargin < 3
        show_animation = true;
    end

    Tmax_current = Tmax_current(:);

    if numel(Tmax_current) ~= 3
        error('Tmax_current must be a 3x1 vector, e.g. [6;0;0].');
    end

    if show_animation
        close all;
    end

    %% Helper: hat map
    hat = @(y)[  0    -y(3)   y(2);
                y(3)   0     -y(1);
               -y(2)  y(1)    0   ];

    %% Robot parameters: one-segment 3-tendon TDCR
    L = 0.25;              % robot length [m]
    N = 100;                % spatial nodes along backbone
    num_disks = 11;        % number of disks
    num_tendons = 3;       % number of tendons

    %% Backbone material: Nitinol SE508 tube
    % PDF gives E = 41-75 GPa. Use this as a tuning parameter.
     E = 83e9;              % Young's modulus [Pa], tune between 41e9 and 75e9
     nu = 0.33;             % assumed Poisson ratio for NiTi
     G = E/(2*(1+nu));      % shear modulus [Pa]

     rho = 6450;            % density [kg/m^3] from PDF: 6.5 g/cm^3
     grav = [0;0;-9.81];    % gravity [m/s^2]
    

    tendon_offset = 6e-3;  % tendon offset from centerline [m]

    % Tendon locations: 0, 120, 240 degrees
    rt = cell(num_tendons,1);
    rt{1} = tendon_offset*[cos(0);       sin(0);       0];
    rt{2} = tendon_offset*[cos(2*pi/3);  sin(2*pi/3);  0];
    rt{3} = tendon_offset*[cos(4*pi/3);  sin(4*pi/3);  0];

    
   %% Cross-section properties: hollow circular SE508 tube
Do = 1.220e-3;         % outer diameter [m]
wall = 0.133e-3;       % wall thickness [m]
Di = 0.954e-3;         % inner diameter [m]

% Optional consistency check
Di_check = Do - 2*wall;
fprintf('Tube ID check: %.6f mm\n', Di_check*1e3);

% Cross-sectional area
A = pi/4*(Do^2 - Di^2);

% Area second moments for hollow circular tube
Ixx = pi/64*(Do^4 - Di^4);
Iyy = Ixx;

% Polar second moment
Izz = pi/32*(Do^4 - Di^4);

J = diag([Ixx, Iyy, Izz]);

% Shear/extension stiffness
Kse = diag([G*A, G*A, E*A]);

% Bending/torsion stiffness
Kbt = diag([E*Ixx, ...
            E*Iyy, ...
            G*Izz]);

    %% Damping and drag
    Bse = 1e-4*zeros(3);        % shear/extension damping
    Bbt = 1e-5*eye(3);     % bending/torsion damping
    Cdrag = 0.03*eye(3);   % quadratic drag matrix

    %% Time parameters
    T = 2.7;               % total simulation time [s]
    dt = 0.01;             % time step [s]
    STEPS = round(T/dt) + 1;

    alpha = -0.2;          % BDF-alpha parameter
    ds = L/(N-1);

    c0 = (1.5 + alpha)/(dt*(1+alpha));
    c1 = -2/dt;
    c2 = (0.5 + alpha)/(dt*(1+alpha));
    d1 = alpha/(1+alpha);

    %% Rest strain
    vstar  = @(s)[0;0;1];
    ustar  = @(s)[0;0;0];
    vsstar = @(s)[0;0;0];
    usstar = @(s)[0;0;0];

    %% State storage
    p = cell(STEPS,N);
    R = cell(STEPS,N);
    n = cell(STEPS,N);
    m = cell(STEPS,N);
    v = cell(STEPS,N);
    u = cell(STEPS,N);
    q = cell(STEPS,N);
    w = cell(STEPS,N);

    vs = cell(STEPS,N);
    us = cell(STEPS,N);
    vt = cell(STEPS,N);
    ut = cell(STEPS,N);
    qt = cell(STEPS,N);
    wt = cell(STEPS,N);
    vst = cell(STEPS,N);
    ust = cell(STEPS,N);

    vh  = cell(STEPS+1,N);
    uh  = cell(STEPS+1,N);
    vsh = cell(STEPS+1,N);
    ush = cell(STEPS+1,N);
    qh  = cell(STEPS+1,N);
    wh  = cell(STEPS+1,N);

    %% Diagnostic storage for extra plots
    shoot_residual_norm = nan(STEPS,1);
    ortho_error_tip     = nan(STEPS,1);

    %% Boundary conditions: fixed/clamped base
    p0 = [0;0;0];
    R0 = eye(3);
    q0 = [0;0;0];
    w0 = [0;0;0];

    for ii = 1:STEPS
        p{ii,1} = p0;
        R{ii,1} = R0;
        q{ii,1} = q0;
        w{ii,1} = w0;
    end

    % External tip force/moment.
    % Tendon tip force/moment are added separately in dynamicIVP().
    nL = [0;0;0];
    mL = [0;0;0];

    %% Solver options
    opt = optimoptions('fsolve', ...
        'Display','off', ...
        'MaxIterations',500, ...
        'MaxFunctionEvaluations',5000, ...
        'FunctionTolerance',1e-8, ...
        'StepTolerance',1e-10);

    %% Initial static solve
    i = 1;

    [Gstatic,~,exitflag] = fsolve(@staticIVP, zeros(6,1), opt);

    if exitflag <= 0
        warning('Initial static solve did not fully converge.');
    end

    % Store final converged static solution
    Estatic = staticIVP(Gstatic);

    shoot_residual_norm(i) = norm(Estatic);
    ortho_error_tip(i) = norm(R{i,N}'*R{i,N} - eye(3),'fro');

    applyStaticBDFalpha();

    if show_animation
        visualize(i);
    end

    %% Dynamic simulation
    for i = 2:STEPS
        guess = [n{i-1,1}; m{i-1,1}];

        [Gdyn,~,exitflag] = fsolve(@dynamicIVP, guess, opt);

        if exitflag <= 0
            fprintf('Warning: dynamic solve did not converge at t = %.3f s\n', (i-1)*dt);
        end

        % Store final converged dynamic solution
        Efinal = dynamicIVP(Gdyn);

        shoot_residual_norm(i) = norm(Efinal);
        ortho_error_tip(i) = norm(R{i,N}'*R{i,N} - eye(3),'fro');

        applyDynamicBDFalpha();

        if show_animation
            visualize(i);
        end
    end

    %% Build full shape data
    shape_data = zeros(STEPS, N, 3);

    for k = 1:STEPS
        for jj = 1:N
            shape_data(k,jj,1) = p{k,jj}(1);
            shape_data(k,jj,2) = p{k,jj}(2);
            shape_data(k,jj,3) = p{k,jj}(3);
        end
    end

    %% Build input data
    input_data = zeros(STEPS,4);

    for k = 1:STEPS
        time_now = (k-1)*dt;
        Tt_now = tendonInput(time_now);
        input_data(k,:) = [time_now, Tt_now(1), Tt_now(2), Tt_now(3)];
    end

    time = input_data(:,1);
    s_grid = linspace(0,L,N);

    %% Final tip position
    tip = p{STEPS,N};

    fprintf('\nFinal tip position:\n');
    fprintf('x = %.6f m\n', tip(1));
    fprintf('y = %.6f m\n', tip(2));
    fprintf('z = %.6f m\n', tip(3));

    %% Extract tip trajectory
    Xtip = zeros(STEPS,1);
    Ytip = zeros(STEPS,1);
    Ztip = zeros(STEPS,1);

    for k = 1:STEPS
        Xtip(k) = p{k,N}(1);
        Ytip(k) = p{k,N}(2);
        Ztip(k) = p{k,N}(3);
    end

    %% Tip positions at selected times
times_report = [0, 0.7, 1.0, 1.5, 2.0, T];

fprintf('\n============================================================\n');
fprintf('TIP POSITIONS AT SELECTED TIMES\n');
fprintf('============================================================\n');

for kk = 1:length(times_report)

    idx = round(times_report(kk)/dt) + 1;
    idx = max(1,min(STEPS,idx));

    fprintf(['t = %.2f s: tip = ' ...
             '[%.6f, %.6f, %.6f] m\n'], ...
             times_report(kk), ...
             Xtip(idx), ...
             Ytip(idx), ...
             Ztip(idx));
end

fprintf('============================================================\n');

    %% Plot 1: tip position components
    figure(1);
    clf;

    subplot(3,1,1)
    plot(time,Xtip,'LineWidth',1.5);
    xlabel('t [s]');
    ylabel('x [m]');
    title('Tip Position - x');
    grid on;

    subplot(3,1,2)
    plot(time,Ytip,'LineWidth',1.5);
    xlabel('t [s]');
    ylabel('y [m]');
    title('Tip Position - y');
    grid on;

    subplot(3,1,3)
    plot(time,Ztip,'LineWidth',1.5);
    xlabel('t [s]');
    ylabel('z [m]');
    title('Tip Position - z');
    grid on;

    saveas(gcf,'TDCR_3T_tip_position_components.png');

    %% Plot 2: final shape
    figure(3);
    clf;
    plotShape(STEPS);
    title('Final 3-Tendon TDCR Shape');
    saveas(gcf,'TDCR_3T_final_shape.png');

    %% Plot 3: tendon tensions vs time
    figure(4);
    clf;

    plot(time, input_data(:,2), 'LineWidth', 1.5);
    hold on;
    plot(time, input_data(:,3), 'LineWidth', 1.5);
    plot(time, input_data(:,4), 'LineWidth', 1.5);

    xlabel('t [s]');
    ylabel('Tendon tension [N]');
    title('Tendon Inputs vs Time');
    legend('T_1','T_2','T_3','Location','best');
    grid on;

    saveas(gcf,'TDCR_tendon_inputs.png');

    %% Plot 4: 3D tip trajectory
    figure(5);
    clf;

    plot3(Xtip, Ytip, Ztip, 'LineWidth', 2);
    hold on;
    plot3(Xtip(1), Ytip(1), Ztip(1), 'go', 'MarkerFaceColor','g');
    plot3(Xtip(end), Ytip(end), Ztip(end), 'ro', 'MarkerFaceColor','r');

    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('3D Tip Trajectory');
    legend('Tip path','Start','End','Location','best');
    grid on;
    axis equal;
    view(3);

    saveas(gcf,'TDCR_3D_tip_trajectory.png');

   %% Plot 5: backbone snapshots in the bending plane
snapshot_times = [0, 0.5, 1.0, 1.5, 2.0, T];

snapshot_ids = round(snapshot_times/dt) + 1;
snapshot_ids = max(1, min(STEPS, snapshot_ids));
snapshot_ids = unique(snapshot_ids, 'stable');

figure(6);
clf;
hold on;

for ss = 1:length(snapshot_ids)

    k = snapshot_ids(ss);

    cx = zeros(1,N);
    cz = zeros(1,N);

    for jj = 1:N
        cx(jj) = p{k,jj}(1);
        cz(jj) = p{k,jj}(3);
    end

    plot(cx, cz, ...
        'LineWidth', 2, ...
        'DisplayName', ...
        sprintf('t = %.2f s', (k-1)*dt));
end

xlabel('x [m]');
ylabel('z [m]');
title('Backbone Shape Snapshots');
legend('Location','best');
grid on;
axis equal;

xlim([-0.03 0.22]);
ylim([-0.05 0.27]);

saveas(gcf, 'TDCR_shape_snapshots_xz.png');
    %% Plot 6: curvature components along backbone
    figure(7);
    clf;

    for ss = 1:length(snapshot_ids)
        k = snapshot_ids(ss);

        ux = zeros(1,N);
        uy = zeros(1,N);
        uz = zeros(1,N);

        for jj = 1:N
            ux(jj) = u{k,jj}(1);
            uy(jj) = u{k,jj}(2);
            uz(jj) = u{k,jj}(3);
        end

        subplot(3,1,1);
        hold on;
        plot(s_grid, ux, 'LineWidth', 1.2, ...
            'DisplayName', sprintf('t=%.2f', (k-1)*dt));
        ylabel('u_x [1/m]');
        title('Curvature Components Along Backbone');
        grid on;

        subplot(3,1,2);
        hold on;
        plot(s_grid, uy, 'LineWidth', 1.2, ...
            'DisplayName', sprintf('t=%.2f', (k-1)*dt));
        ylabel('u_y [1/m]');
        grid on;

        subplot(3,1,3);
        hold on;
        plot(s_grid, uz, 'LineWidth', 1.2, ...
            'DisplayName', sprintf('t=%.2f', (k-1)*dt));
        xlabel('s [m]');
        ylabel('u_z [1/m]');
        grid on;
    end

    subplot(3,1,1);
    legend('Location','best');

    saveas(gcf,'TDCR_curvature_components.png');

    %% Plot 7: curvature magnitude heatmap
    U_mag = zeros(STEPS,N);

    for k = 1:STEPS
        for jj = 1:N
            U_mag(k,jj) = norm(u{k,jj});
        end
    end

    figure(8);
    clf;

    imagesc(s_grid, time, U_mag);
    set(gca,'YDir','normal');

    xlabel('s [m]');
    ylabel('t [s]');
    title('Curvature Magnitude Heatmap ||u(t,s)||');
    colorbar;

    saveas(gcf,'TDCR_curvature_heatmap.png');

    %% Plot 8: internal force and moment magnitude heatmaps
    N_mag = zeros(STEPS,N);
    M_mag = zeros(STEPS,N);

    for k = 1:STEPS
        for jj = 1:N
            N_mag(k,jj) = norm(n{k,jj});
            M_mag(k,jj) = norm(m{k,jj});
        end
    end

    figure(9);
    clf;

    imagesc(s_grid, time, N_mag);
    set(gca,'YDir','normal');

    xlabel('s [m]');
    ylabel('t [s]');
    title('Internal Force Magnitude ||n(t,s)||');
    colorbar;

    saveas(gcf,'TDCR_internal_force_heatmap.png');

    figure(10);
    clf;

    imagesc(s_grid, time, M_mag);
    set(gca,'YDir','normal');

    xlabel('s [m]');
    ylabel('t [s]');
    title('Internal Moment Magnitude ||m(t,s)||');
    colorbar;

    saveas(gcf,'TDCR_internal_moment_heatmap.png');

    %% Plot 9: tip velocity
    Vtip = zeros(STEPS,3);
    Vtip_mag = zeros(STEPS,1);

    for k = 1:STEPS
        Vtip(k,:) = (R{k,N}*q{k,N})';
        Vtip_mag(k) = norm(Vtip(k,:));
    end
  %% Maximum tip-speed result
[max_tip_speed, idx_max_speed] = max(Vtip_mag);
time_max_speed = time(idx_max_speed);

fprintf('\n============================================================\n');
fprintf('MAXIMUM TIP SPEED\n');
fprintf('============================================================\n');
fprintf('Maximum tip speed = %.6f m/s\n',max_tip_speed);
fprintf('Time of maximum speed = %.3f s\n',time_max_speed);
fprintf('============================================================\n');

    figure(11);
    clf;

    subplot(4,1,1);
    plot(time, Vtip(:,1), 'LineWidth', 1.5);
    ylabel('v_x [m/s]');
    title('Tip Velocity');
    grid on;

    subplot(4,1,2);
    plot(time, Vtip(:,2), 'LineWidth', 1.5);
    ylabel('v_y [m/s]');
    grid on;

    subplot(4,1,3);
    plot(time, Vtip(:,3), 'LineWidth', 1.5);
    ylabel('v_z [m/s]');
    grid on;

    subplot(4,1,4);
    plot(time, Vtip_mag, 'LineWidth', 1.5);
    xlabel('t [s]');
    ylabel('|v| [m/s]');
    grid on;

    saveas(gcf,'TDCR_tip_velocity.png');

    %% Plot 10: shooting residual norm
    figure(12);
    clf;

    semilogy(time, max(shoot_residual_norm, eps), 'LineWidth', 1.5);

    xlabel('t [s]');
    ylabel('||Boundary residual||');
    title('Shooting Method Boundary Residual');
    grid on;

    saveas(gcf,'TDCR_shooting_residual.png');

    %% Plot 11: tip rotation orthogonality error
    figure(13);
    clf;

    semilogy(time, max(ortho_error_tip, eps), 'LineWidth', 1.5);

    xlabel('t [s]');
    ylabel('||R_{tip}^T R_{tip} - I||');
    title('Tip Rotation Orthogonality Error');
    grid on;

    saveas(gcf,'TDCR_rotation_orthogonality_error.png');
    %% Numerical diagnostic summary
[max_shooting_residual, idx_max_residual] = ...
    max(shoot_residual_norm);

[max_orthogonality_error, idx_max_orthogonality] = ...
    max(ortho_error_tip);

fprintf('\n============================================================\n');
fprintf('NUMERICAL DIAGNOSTICS\n');
fprintf('============================================================\n');

fprintf('Maximum shooting residual = %.6e\n', ...
    max_shooting_residual);

fprintf('Time of maximum shooting residual = %.3f s\n', ...
    time(idx_max_residual));

fprintf('Maximum rotation orthogonality error = %.6e\n', ...
    max_orthogonality_error);

fprintf('Time of maximum orthogonality error = %.3f s\n', ...
    time(idx_max_orthogonality));

fprintf('============================================================\n');

    %% Save full simulation and diagnostic data
    save(filename, ...
         'shape_data', ...
         'input_data', ...
         'shoot_residual_norm', ...
         'ortho_error_tip', ...
         'U_mag', ...
         'N_mag', ...
         'M_mag', ...
         'Vtip', ...
         'Vtip_mag', ...
         's_grid', ...
         'dt', ...
         'L', ...
         'N', ...
         'STEPS', ...
         'Tmax_current');

    fprintf('Saved full shape dataset and diagnostics to %s\n', filename);

    %% Rotation consistency check
    errR = norm(R{STEPS,N}'*R{STEPS,N} - eye(3),'fro');
    fprintf('Final tip rotation orthogonality error = %.3e\n', errR);

    %% Nested functions

    function Tt = tendonInput(time_now)

        Tmax = Tmax_current(:);

        t1 = 0.2;   % start pull
        t2 = 0.7;   % fully pulled
        t3 = 1.5;   % start release
        t4 = 2.0;   % fully released

        if time_now < t1
            scale = 0;
        elseif time_now < t2
            scale = (time_now - t1)/(t2 - t1);
        elseif time_now < t3
            scale = 1;
        elseif time_now < t4
            scale = 1 - (time_now - t3)/(t4 - t3);
        else
            scale = 0;
        end

        Tt = scale*Tmax;
    end

    function applyStaticBDFalpha()
        for jj = 1:N
            vh{i+1,jj}  = (c1+c2)*v{i,jj};
            uh{i+1,jj}  = (c1+c2)*u{i,jj};
            vsh{i+1,jj} = (c1+c2)*vs{i,jj};
            ush{i+1,jj} = (c1+c2)*us{i,jj};

            qh{i+1,jj} = [0;0;0];
            wh{i+1,jj} = [0;0;0];

            q{i,jj} = [0;0;0];
            w{i,jj} = [0;0;0];

            vt{i,jj}  = [0;0;0];
            ut{i,jj}  = [0;0;0];
            qt{i,jj}  = [0;0;0];
            wt{i,jj}  = [0;0;0];
            vst{i,jj} = [0;0;0];
            ust{i,jj} = [0;0;0];
        end
    end

    function applyDynamicBDFalpha()
        for jj = 1:N
            vh{i+1,jj}  = c1*v{i,jj}  + c2*v{i-1,jj}  + d1*vt{i,jj};
            uh{i+1,jj}  = c1*u{i,jj}  + c2*u{i-1,jj}  + d1*ut{i,jj};

            vsh{i+1,jj} = c1*vs{i,jj} + c2*vs{i-1,jj} + d1*vst{i,jj};
            ush{i+1,jj} = c1*us{i,jj} + c2*us{i-1,jj} + d1*ust{i,jj};

            qh{i+1,jj} = c1*q{i,jj} + c2*q{i-1,jj} + d1*qt{i,jj};
            wh{i+1,jj} = c1*w{i,jj} + c2*w{i-1,jj} + d1*wt{i,jj};
        end
    end

    function Eres = staticIVP(Gguess)

        % Reset base boundary values
        p{i,1} = p0;
        R{i,1} = R0;
        q{i,1} = q0;
        w{i,1} = w0;

        n{i,1} = Gguess(1:3);
        m{i,1} = Gguess(4:6);

        for j_idx = 1:N-1
            [ps,Rs,ns,ms,vs{i,j_idx},us{i,j_idx},v{i,j_idx},u{i,j_idx}] = ...
                staticODE(p{i,j_idx},R{i,j_idx},n{i,j_idx},m{i,j_idx},j_idx);

            p{i,j_idx+1} = p{i,j_idx} + ds*ps;

            Rtmp = R{i,j_idx} + ds*Rs;
            R{i,j_idx+1} = projectSO3(Rtmp);

            n{i,j_idx+1} = n{i,j_idx} + ds*ns;
            m{i,j_idx+1} = m{i,j_idx} + ds*ms;
        end

        % Fill algebraic/static quantities at the actual tip node
        [~,~,~,~,vs{i,N},us{i,N},v{i,N},u{i,N}] = ...
            staticODE(p{i,N},R{i,N},n{i,N},m{i,N},N);

        q{i,N} = [0;0;0];
        w{i,N} = [0;0;0];

        Eres = [n{i,N} - nL;
                m{i,N} - mL];
    end

    function Eres = dynamicIVP(Gguess)

        % Reset base boundary values
        p{i,1} = p0;
        R{i,1} = R0;
        q{i,1} = q0;
        w{i,1} = w0;

        n{i,1} = Gguess(1:3);
        m{i,1} = Gguess(4:6);

        for j_idx = 1:N-1
            [ps,Rs,ns,ms,qs,ws,vs{i,j_idx},us{i,j_idx}, ...
             v{i,j_idx},u{i,j_idx},vt{i,j_idx},ut{i,j_idx}, ...
             qt{i,j_idx},wt{i,j_idx},vst{i,j_idx},ust{i,j_idx}] = ...
                dynamicODE(p{i,j_idx},R{i,j_idx},n{i,j_idx},m{i,j_idx}, ...
                           q{i,j_idx},w{i,j_idx},j_idx);

            p{i,j_idx+1} = p{i,j_idx} + ds*ps;

            Rtmp = R{i,j_idx} + ds*Rs;
            R{i,j_idx+1} = projectSO3(Rtmp);

            n{i,j_idx+1} = n{i,j_idx} + ds*ns;
            m{i,j_idx+1} = m{i,j_idx} + ds*ms;
            q{i,j_idx+1} = q{i,j_idx} + ds*qs;
            w{i,j_idx+1} = w{i,j_idx} + ds*ws;
        end

        % Compute algebraic and derivative quantities at the actual tip node
        [~,~,~,~,~,~,vs{i,N},us{i,N}, ...
         v{i,N},u{i,N},vt{i,N},ut{i,N}, ...
         qt{i,N},wt{i,N},vst{i,N},ust{i,N}] = ...
            dynamicODE(p{i,N},R{i,N},n{i,N},m{i,N}, ...
                       q{i,N},w{i,N},N);

        time_now = (i-1)*dt;
        Tt = tendonInput(time_now);

        % Compute tendon boundary load at the true tip s = L
        [nTenTip,mTenTip] = tendonTipLoad(R{i,N},u{i,N},v{i,N},Tt);

        Eres = [n{i,N} - (nL + nTenTip);
                m{i,N} - (mL + mTenTip)];
    end

    function [ps,Rs,ns,ms,vs_local,us_local,v_local,u_local] = ...
        staticODE(p_local,R_local,n_local,m_local,j_idx)

        s_now = ds*(j_idx-1);

        v_local = Kse\(R_local'*n_local) + vstar(s_now);
        u_local = Kbt\(R_local'*m_local) + ustar(s_now);

        % Static initial solve assumes zero tendon actuation at t = 0.
        Tt = zeros(num_tendons,1);

        [At,Gt,H,a,b] = tendonTerms(u_local,v_local,Tt);

        nb = Kse*(v_local - vstar(s_now));
        mb = Kbt*(u_local - ustar(s_now));

        rhs = [-cross(u_local,nb) - R_local'*rho*A*grav - a;
               -cross(u_local,mb) - cross(v_local,nb) - b];

        Mat = [Kse+At, Gt;
               Gt',    Kbt+H];

        sol = Mat\rhs;

        vs_local = sol(1:3);
        us_local = sol(4:6);

        ps = R_local*v_local;
        Rs = R_local*hat(u_local);

        % Static Cosserat propagation
        ns = -rho*A*grav;
        ms = -hat(ps)*n_local;
    end

    function [ps,Rs,ns,ms,qs,ws,vs_local,us_local, ...
              v_local,u_local,vt_local,ut_local,qt_local,wt_local, ...
              vst_local,ust_local] = ...
        dynamicODE(p_local,R_local,n_local,m_local,q_local,w_local,j_idx)

        s_now = ds*(j_idx-1);
        time_now = (i-1)*dt;
        Tt = tendonInput(time_now);

        % Algebraic solve for current v and u from implicit constitutive law
        v_local = (Kse+c0*Bse)\ ...
            (R_local'*n_local + Kse*vstar(s_now) - Bse*vh{i,j_idx});

        u_local = (Kbt+c0*Bbt)\ ...
            (R_local'*m_local + Kbt*ustar(s_now) - Bbt*uh{i,j_idx});

        % BDF-alpha time derivatives
        vt_local = c0*v_local + vh{i,j_idx};
        ut_local = c0*u_local + uh{i,j_idx};
        qt_local = c0*q_local + qh{i,j_idx};
        wt_local = c0*w_local + wh{i,j_idx};

        % Tendon distributed terms
        [At,Gt,H,a,b] = tendonTerms(u_local,v_local,Tt);

        % Local internal force/moment from constitutive law
        nb = Kse*(v_local - vstar(s_now)) + Bse*vt_local;
        mb = Kbt*(u_local - ustar(s_now)) + Bbt*ut_local;

        % Additional distributed loads not due to tendons
        fbar_local = [0;0;0];
        lbar_local = [0;0;0];

        % Local RHS terms
        LambdaN = -a ...
            + rho*A*(hat(w_local)*q_local + qt_local) ...
            + Cdrag*(q_local.*abs(q_local)) ...
            - R_local'*(rho*A*grav + fbar_local);

        LambdaM = -b ...
            + rho*(hat(w_local)*J*w_local + J*wt_local) ...
            - hat(v_local)*nb ...
            - R_local'*lbar_local;

        GammaV = hat(u_local)*nb ...
            - Kse*vsstar(s_now) ...
            + Bse*vsh{i,j_idx};

        GammaU = hat(u_local)*mb ...
            - Kbt*usstar(s_now) ...
            + Bbt*ush{i,j_idx};

        % Coupled tendon/Cosserat solve for v_s and u_s
        Mat = [Kse+c0*Bse+At, Gt;
               Gt',           Kbt+c0*Bbt+H];

        sol = Mat\[-GammaV + LambdaN;
                   -GammaU + LambdaM];

        vs_local = sol(1:3);
        us_local = sol(4:6);

        vst_local = c0*vs_local + vsh{i,j_idx};
        ust_local = c0*us_local + ush{i,j_idx};

        % Spatial ODE system
        ps = R_local*v_local;
        Rs = R_local*hat(u_local);

        % R^T n_s = -A v_s - G u_s + LambdaN
        % R^T m_s = -G^T v_s - H u_s + LambdaM
        ns = R_local*(-At*vs_local - Gt*us_local + LambdaN);
        ms = R_local*(-Gt'*vs_local - H*us_local + LambdaM);

        qs = vt_local - hat(u_local)*q_local + hat(w_local)*v_local;
        ws = ut_local - hat(u_local)*w_local;
    end

    function [At,Gt,H,a,b] = tendonTerms(u_local,v_local,Tt)

        At = zeros(3);
        Gt = zeros(3);
        H  = zeros(3);
        a  = zeros(3,1);
        b  = zeros(3,1);

        for kk = 1:num_tendons
            % Constant-offset tendon routing:
            % p_s_tendon_body = u_hat*r + v
            ptsb = hat(u_local)*rt{kk} + v_local;
            ptsb_norm = max(norm(ptsb),1e-9);

            Ak = -Tt(kk)/ptsb_norm^3 * hat(ptsb)*hat(ptsb);
            Gk = -Ak*hat(rt{kk});
            Hk = hat(rt{kk})*Gk;

            ak = Ak*(hat(u_local)*ptsb);
            bk = hat(rt{kk})*ak;

            At = At + Ak;
            Gt = Gt + Gk;
            H  = H  + Hk;
            a  = a  + ak;
            b  = b  + bk;
        end
    end

    function [nTip, mTip] = tendonTipLoad(R_tip,u_tip,v_tip,Tt)

        nTip = zeros(3,1);
        mTip = zeros(3,1);

        for kk = 1:num_tendons
            pts = R_tip*(hat(u_tip)*rt{kk} + v_tip);
            pts_norm = max(norm(pts),1e-9);
            tangent = pts/pts_norm;

            nTip = nTip - Tt(kk)*tangent;
            mTip = mTip - Tt(kk)*hat(R_tip*rt{kk})*tangent;
        end
    end

    function Rproj = projectSO3(Rin)

        [U,~,V] = svd(Rin);
        Rproj = U*V';

        if det(Rproj) < 0
            U(:,3) = -U(:,3);
            Rproj = U*V';
        end
    end

    function visualize(step_idx)

        if mod(step_idx,2) ~= 0
            return;
        end

        figure(2);
        clf;
        plotShape(step_idx);
        title(sprintf('3-Tendon TDCR Shape at t = %.3f s', (step_idx-1)*dt));
        drawnow;
        pause(0.01);
    end

    function plotShape(step_idx)

        cx = zeros(1,N);
        cy = zeros(1,N);
        cz = zeros(1,N);

        for jj = 1:N
            cx(jj) = p{step_idx,jj}(1);
            cy(jj) = p{step_idx,jj}(2);
            cz(jj) = p{step_idx,jj}(3);
        end

        plot3(cx,cy,cz,'b-','LineWidth',2.5);
        hold on;

        % Tendon lines
        tendon_lines = cell(num_tendons,1);

        for kk = 1:num_tendons
            tendon_lines{kk} = zeros(3,N);
        end

        for jj = 1:N
            for kk = 1:num_tendons
                tendon_lines{kk}(:,jj) = p{step_idx,jj} + R{step_idx,jj}*rt{kk};
            end
        end

        for kk = 1:num_tendons
            plot3(tendon_lines{kk}(1,:), ...
                  tendon_lines{kk}(2,:), ...
                  tendon_lines{kk}(3,:), ...
                  'k-', 'LineWidth', 1);
        end

        % Disk markers
        disk_ids = round(linspace(1,N,num_disks));

        for dd = 1:num_disks
            idx = disk_ids(dd);
            plot3(p{step_idx,idx}(1), ...
                  p{step_idx,idx}(2), ...
                  p{step_idx,idx}(3), ...
                  'co', 'MarkerSize', 6, 'LineWidth', 1.2);
        end

        % Tip
        plot3(cx(end),cy(end),cz(end), ...
              'ro','MarkerSize',8,'MarkerFaceColor','r');

        % Base
        plot3(0,0,0,'ks','MarkerFaceColor','k','MarkerSize',8);

        axis([-0.08 0.08 -0.08 0.08 0 0.18]);
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
        grid on;
        daspect([1 1 1]);
        view(3);
    end

end