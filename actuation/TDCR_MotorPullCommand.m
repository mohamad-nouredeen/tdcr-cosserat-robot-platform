function TDCR_MotorPullCommand(pull_mm)
% TDCR_MotorPullCommand
%
% Open-loop tendon-displacement command for three Dynamixel RX-28 motors.
%
% The requested tendon pulls [mm] are converted directly to RX-28 Goal
% Positions using the spool radius:
%
%   tendon_pull = spool_radius * motor_rotation
%
% IMPORTANT:
% This is position control at the motor. The commanded spool displacement
% is not directly measured at the tendon, so real tendon motion may differ
% slightly because of tendon stretch, slack, spool effects, and friction.
%
% Operation:
%   1. Receive desired tendon pulls [mm].
%   2. Convert tendon pulls [mm] to motor Goal Positions.
%   3. Move all motors to zero_pos.
%   4. Move motors to the calculated target positions.
%   5. Keep holding the target positions.
%   6. Wait until the user presses Enter.
%   7. Return motors to zero_pos.
%   8. Disable torque and close communication.

    clc;

    %% ============================================================
    % Input

    if nargin < 1
        pull_mm = [0 0 10];
    end

    pull_mm = double(pull_mm(:)');

    if numel(pull_mm) ~= 3
        error('pull_mm must contain three values, for example [0 0 10].');
    end

    if any(~isfinite(pull_mm))
        error('All tendon-pull values must be finite.');
    end

    if any(pull_mm < 0)
        error('Tendon-pull values must be nonnegative.');
    end

    %% ============================================================
    % Communication settings
   

    DEVICENAME = 'COM3';
    BAUDRATE = 1000000;
    PROTOCOL_VERSION = 1.0;

    %% ============================================================
    % Motor configuration


    motor_ids = [1 2 3];

    % Neutral motor positions corresponding to zero tendon pull
    zero_pos = [3 3 3];

    % Spool radius [mm]
    spool_radius_mm = 2;

    % Direction in which each motor pulls its tendon.
    %
    % Use 1 if increasing the Goal Position pulls the tendon.
    % Use -1 if decreasing the Goal Position pulls the tendon.
    motor_sign = [1 1 1];

    %% ============================================================
    % Motor safety limits
    

    goal_min = 1;
    goal_max = 1023;

    %% ============================================================
    % Movement settings
   

    MOVING_SPEED_VALUE = 150;
    TORQUE_LIMIT_VALUE = 250;

    pull_time = 1.0;       % Time to reach pulling position [s]
    return_time = 1.0;     % Time to return to zero [s]
    command_dt = 0.05;     % Interval between position commands [s]

    %% ============================================================
    % RX-28 control-table addresses
    

    ADDR_TORQUE_ENABLE = 24;
    ADDR_GOAL_POSITION = 30;
    ADDR_MOVING_SPEED  = 32;
    ADDR_TORQUE_LIMIT  = 34;

    TORQUE_ENABLE = 1;
    TORQUE_DISABLE = 0;
    COMM_SUCCESS = 0;

    %% ============================================================
    % Dynamixel SDK path
    % =============================================================

    sdkRoot = ...
        'C:\Users\mhmdn\AppData\Roaming\MathWorks\MATLAB Add-Ons\Collections\DynamixelSDK';

    LIB_NAME = 'dxl_x64_c';

    %% ============================================================
    % Hardware state
    

    port_num = [];
    port_opened = false;
    torque_enabled = false;
    shutdown_done = false;

    %% ============================================================
    % Convert tendon pull [mm] to motor Goal Position
    

    % One RX-28 Goal Position unit is approximately 0.29 degrees.
    unit_rad = 0.29*pi/180;

    % Tendon-spool relationship:
    %
    %   tendon_pull = spool_radius * motor_angle
    %
    % Therefore:
    %
    %   motor_angle = tendon_pull / spool_radius
    %
    % Motor position-unit change:
    %
    %   delta_units =
    %       pull_mm / (spool_radius_mm * unit_rad)

    delta_units = pull_mm ./ ...
        (spool_radius_mm * unit_rad);

    % Target Goal Position:
    %
    %   target = zero + direction * position change

    target_goal = zero_pos + ...
        motor_sign .* round(delta_units);

    target_goal = round(target_goal);

    %% ============================================================
    % Verify target positions
    % =============================================================

    for jj = 1:3

        if motor_sign(jj) > 0

            available_units = ...
                goal_max - zero_pos(jj);

        else

            available_units = ...
                zero_pos(jj) - goal_min;
        end

        maximum_pull_mm = ...
            available_units * ...
            spool_radius_mm * ...
            unit_rad;

        if target_goal(jj) < goal_min || ...
                target_goal(jj) > goal_max

            error(['Tendon %d requested pull %.2f mm, but the maximum ', ...
                   'available pull is approximately %.2f mm with the ', ...
                   'current zero position and motor safety limits.'], ...
                   jj, ...
                   pull_mm(jj), ...
                   maximum_pull_mm);
        end
    end

    %% ============================================================
    % Display command information
    % =============================================================

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('TDCR MOTOR PULL COMMAND\n');
    fprintf('============================================================\n');

    fprintf('Commanded tendon pull = [%.3f %.3f %.3f] mm\n', ...
        pull_mm(1), ...
        pull_mm(2), ...
        pull_mm(3));

    fprintf('Zero positions        = [%d %d %d]\n', ...
        zero_pos(1), ...
        zero_pos(2), ...
        zero_pos(3));

    fprintf('Position changes      = [%d %d %d] units\n', ...
        round(delta_units(1)), ...
        round(delta_units(2)), ...
        round(delta_units(3)));

    fprintf('Target positions      = [%d %d %d]\n', ...
        target_goal(1), ...
        target_goal(2), ...
        target_goal(3));

    fprintf('Motor IDs             = [%d %d %d]\n', ...
        motor_ids(1), ...
        motor_ids(2), ...
        motor_ids(3));

    fprintf('Spool radius          = %.3f mm\n', ...
        spool_radius_mm);

    fprintf('Moving speed          = %d\n', ...
        MOVING_SPEED_VALUE);

    fprintf('Torque limit          = %d\n', ...
        TORQUE_LIMIT_VALUE);

    fprintf('Pull time             = %.2f seconds\n', ...
        pull_time);

    fprintf('Return time           = %.2f seconds\n', ...
        return_time);

    fprintf('\nMake sure DYNAMIXEL Wizard is CLOSED.\n');
    fprintf('Make sure the robot and tendons are safe.\n');

    input('\nPress Enter to initialize and start...', 's');

    %% ============================================================
    % Emergency cleanup
    % =============================================================

    % This ensures torque is disabled if an unexpected error or
    % interruption occurs.

    cleanupObject = onCleanup(@safeShutdown);

    try

        %% ========================================================
        % 1. Initialize motors
        % =========================================================

        initializeHardware();

        %% ========================================================
        % 2. Move motors to zero positions
        % =========================================================

        fprintf('\nMoving motors to zero positions...\n');

        sendGoals(zero_pos);

        pause(1.0);

        %% ========================================================
        % 3. Move motors to calculated pulling positions
        % =========================================================

        fprintf('\nPulling commanded tendons...\n');

        rampGoals( ...
            zero_pos, ...
            target_goal, ...
            pull_time, ...
            command_dt);

        %% ========================================================
        % 4. Hold motors at pulling positions
        % =========================================================

        fprintf('\n');
        fprintf('============================================================\n');
        fprintf('PULLING POSITIONS REACHED\n');
        fprintf('============================================================\n');

        fprintf('Commanded pulls = [%.3f %.3f %.3f] mm\n', ...
            pull_mm(1), ...
            pull_mm(2), ...
            pull_mm(3));

        fprintf('Motor goals     = [%d %d %d]\n', ...
            target_goal(1), ...
            target_goal(2), ...
            target_goal(3));

        fprintf('\nMotors are holding the commanded positions.\n');
        fprintf('Motor torque remains enabled.\n');

        %% ========================================================
        % 5. Wait until user wants to return
        % =========================================================

        input(['\nPress Enter when you want the motors ', ...
               'to return to zero...'], ...
               's');

        %% ========================================================
        % 6. Return motors to zero positions
        % =========================================================

        fprintf('\nReturning motors to zero positions...\n');

        rampGoals( ...
            target_goal, ...
            zero_pos, ...
            return_time, ...
            command_dt);

        pause(0.5);

        fprintf('Motors returned to zero positions.\n');
        fprintf('Motor command completed successfully.\n');

    catch ME

        fprintf('\nERROR: %s\n', ME.message);
        fprintf('Starting safe shutdown...\n');
    end

    %% ============================================================
    % Normal shutdown
    % =============================================================

    safeShutdown();

    % Remove the cleanup object after normal shutdown.
    % shutdown_done prevents duplicate shutdown actions.

    cleanupObject = [];

    %% ============================================================
    % Nested helper functions
    % =============================================================

    function initializeHardware()
        % Add the Dynamixel SDK folders to the MATLAB path.

        addpath(genpath(fullfile( ...
            sdkRoot, ...
            'matlab')));

        addpath(fullfile( ...
            sdkRoot, ...
            'c', ...
            'include', ...
            'dynamixel_sdk'));

        addpath(fullfile( ...
            sdkRoot, ...
            'c', ...
            'build', ...
            'win64', ...
            'output'));

        %% Dynamixel SDK files

        dllFile = fullfile( ...
            sdkRoot, ...
            'c', ...
            'build', ...
            'win64', ...
            'output', ...
            'dxl_x64_c.dll');

        headerFile = fullfile( ...
            sdkRoot, ...
            'c', ...
            'include', ...
            'dynamixel_sdk', ...
            'dynamixel_sdk.h');

        portHeader = fullfile( ...
            sdkRoot, ...
            'c', ...
            'include', ...
            'dynamixel_sdk', ...
            'port_handler.h');

        packetHeader = fullfile( ...
            sdkRoot, ...
            'c', ...
            'include', ...
            'dynamixel_sdk', ...
            'packet_handler.h');

        %% Check SDK files

        if ~isfile(dllFile)

            error('Cannot find the Dynamixel SDK DLL at: %s', ...
                dllFile);
        end

        if ~isfile(headerFile)

            error('Cannot find the Dynamixel SDK header at: %s', ...
                headerFile);
        end

        %% Load Dynamixel SDK

        if ~libisloaded(LIB_NAME)

            fprintf('\nLoading Dynamixel SDK...\n');

            loadlibrary( ...
                dllFile, ...
                headerFile, ...
                'addheader', ...
                portHeader, ...
                'addheader', ...
                packetHeader, ...
                'alias', ...
                LIB_NAME);
        end

        %% Create port and packet handlers

        port_num = portHandler(DEVICENAME);

        packetHandler();

        %% Open COM port

        if openPort(port_num)

            port_opened = true;

            fprintf('Opened port %s successfully.\n', ...
                DEVICENAME);

        else

            error('Failed to open port %s.', ...
                DEVICENAME);
        end

        %% Set baud rate

        if setBaudRate(port_num, BAUDRATE)

            fprintf('Baud rate set to %d.\n', ...
                BAUDRATE);

        else

            error('Failed to set baud rate.');
        end

        %% Ping motors

        fprintf('\nPinging motors...\n');

        for jj = 1:3

            id = motor_ids(jj);

            model_number = ...
                pingGetModelNum( ...
                    port_num, ...
                    PROTOCOL_VERSION, ...
                    id);

            checkDxResult(id, 'ping');

            fprintf( ...
                '[ID:%03d] Ping OK, model number %d\n', ...
                id, ...
                model_number);
        end

        %% Configure speed and torque limit

        fprintf('\nConfiguring motors...\n');

        for jj = 1:3

            id = motor_ids(jj);

            write2ByteTxRx( ...
                port_num, ...
                PROTOCOL_VERSION, ...
                id, ...
                ADDR_MOVING_SPEED, ...
                MOVING_SPEED_VALUE);

            checkDxResult( ...
                id, ...
                'set moving speed');

            write2ByteTxRx( ...
                port_num, ...
                PROTOCOL_VERSION, ...
                id, ...
                ADDR_TORQUE_LIMIT, ...
                TORQUE_LIMIT_VALUE);

            checkDxResult( ...
                id, ...
                'set torque limit');

            fprintf( ...
                '[ID:%03d] Speed = %d, Torque Limit = %d\n', ...
                id, ...
                MOVING_SPEED_VALUE, ...
                TORQUE_LIMIT_VALUE);
        end

        %% Enable motor torque

        fprintf('Enabling motor torque...\n');

        for jj = 1:3

            id = motor_ids(jj);

            write1ByteTxRx( ...
                port_num, ...
                PROTOCOL_VERSION, ...
                id, ...
                ADDR_TORQUE_ENABLE, ...
                TORQUE_ENABLE);

            checkDxResult( ...
                id, ...
                'enable torque');
        end

        torque_enabled = true;
    end

    function rampGoals( ...
            start_goals, ...
            end_goals, ...
            duration_s, ...
            dt_s)
        % Gradually move all three motors from start_goals
        % to end_goals.

        number_of_steps = max( ...
            2, ...
            round(duration_s/dt_s) + 1);

        for kk = 1:number_of_steps

            alpha = ...
                (kk - 1)/(number_of_steps - 1);

            current_goals = round( ...
                (1 - alpha)*start_goals + ...
                alpha*end_goals);

            sendGoals(current_goals);

            if kk < number_of_steps
                pause(dt_s);
            end
        end
    end

    function sendGoals(goal_positions)
        % Send one Goal Position to every Dynamixel motor.

        if any(goal_positions < goal_min) || ...
                any(goal_positions > goal_max)

            error(['One or more Goal Positions are outside ', ...
                   'the safety range [%d, %d].'], ...
                   goal_min, ...
                   goal_max);
        end

        for jj = 1:3

            id = motor_ids(jj);

            write2ByteTxRx( ...
                port_num, ...
                PROTOCOL_VERSION, ...
                id, ...
                ADDR_GOAL_POSITION, ...
                goal_positions(jj));

            checkDxResult( ...
                id, ...
                'write goal position');
        end
    end

    function safeShutdown()
        % Disable torque, close the port and unload the SDK.

        if shutdown_done
            return;
        end

        shutdown_done = true;

        fprintf('\nDisabling torque and closing port...\n');

        %% Disable torque

        if port_opened && torque_enabled

            for jj = 1:3

                try

                    id = motor_ids(jj);

                    write1ByteTxRx( ...
                        port_num, ...
                        PROTOCOL_VERSION, ...
                        id, ...
                        ADDR_TORQUE_ENABLE, ...
                        TORQUE_DISABLE);

                catch

                    % Continue shutting down if one motor fails.
                end
            end
        end

        %% Close port

        if port_opened

            try

                closePort(port_num);

            catch

                % Continue shutdown if closing the port fails.
            end

            port_opened = false;
        end

        %% Unload SDK

        if libisloaded(LIB_NAME)

            try

                unloadlibrary(LIB_NAME);

            catch

                % Continue shutdown if unloading the library fails.
            end
        end

        torque_enabled = false;

        fprintf('Shutdown finished.\n');
    end

    function checkDxResult(id, action_name)
        % Check serial communication and Dynamixel packet errors.

        communication_result = ...
            getLastTxRxResult( ...
                port_num, ...
                PROTOCOL_VERSION);

        motor_error = ...
            getLastRxPacketError( ...
                port_num, ...
                PROTOCOL_VERSION);

        %% Communication error

        if communication_result ~= COMM_SUCCESS

            fprintf( ...
                '[ID:%03d] Communication failed during %s.\n', ...
                id, ...
                action_name);

            fprintf( ...
                'Communication result = %d\n', ...
                communication_result);

            if exist('printTxRxResult', 'file') == 2

                printTxRxResult( ...
                    PROTOCOL_VERSION, ...
                    communication_result);
            end

            error('Dynamixel communication failed.');
        end

        %% Dynamixel packet error

        if motor_error ~= 0

            fprintf( ...
                '[ID:%03d] Motor error during %s.\n', ...
                id, ...
                action_name);

            fprintf( ...
                'Motor error value = %d\n', ...
                motor_error);

            if exist('printRxPacketError', 'file') == 2

                printRxPacketError( ...
                    PROTOCOL_VERSION, ...
                    motor_error);
            end

            error('Dynamixel returned a packet error.');
        end
    end
end