%
%
%TENDONS Position of the tendons of a tendon actuated coninuum robot in the
%cross-sectional frame. Different routing patterns are defined. The tendons
%ending in the proximal segments have no influence on the distal segments.
%The tendons ending in the distal segments are routed parallel in the
%proximal segments at the location of their entry point in the segment they
%terminate in.
%The function also returns other configuration parameters for the specified
%routing. For the configuration parameters and/or the tension outputs, the
%specified routing configuration is the only necessary input.
%
%   [D, dD, ddD, n_segments, n_tendons, l_j] = ...
%                                      tendons(routing)
%                                      tendons(routing, segment, tendon, X)
%
%input:
%routing, the selected routing (see comments in the swith-case statement
% below)
%segment, the current considered segment.
%tendon, the current considered tendon.
%X, the location of the considered cross-section of the considered segment
% on a normalized length from 0 to 1.
%
%output:
%D, the position of the tendon.
%dD, the first derivative of the position of the tendon.
%ddD, the second derivative of the position of the tendon.
%n_segments, the number of sectinos.
%n_tendons, the number of tendons per segment.
%l_j, the lenght of each segment.
%
%

function [D, dD, ddD, n_segments, n_tendons, l_j] = ...
                                       tendons(routing, segment, tendon, X)
% dummy values for parameters 2 -> 4 when only the confiuation is needed
if nargin == 1
    segment = 1;
    tendon = 1;
    X = 1;
end
switch routing % swith configuration
    %% P1: 1 segment - 3 tendons - parallel routing at 120 degrees
case 1

    n_segments = 1;
    n_tendons = 3;

    % Use the actual free bending length from the clamp to the tip.
    l_j = 0.25;               % [m]

    tendon_offset = 6e-3;     % [m]
    tendon_angles = [0, 2*pi/3, 4*pi/3];

    theta = tendon_angles(tendon);

    % Backbone axis is local z.
    % Tendons therefore lie in the local x-y plane.
    D = tendon_offset * ...
        [cos(theta); sin(theta); 0];

    % Parallel tendon routing
    dD = zeros(3,1);
    ddD = zeros(3,1);
%% P2 : 1 segment - 3 tendons - parallel routing at 120 deg(newtonian)
case 2
    n_segments = 1;
    n_tendons = 3;
    l_j = 0.15;
    tendon_offset = 6e-3;
    tendon_angles = [0, 2*pi/3, 4*pi/3];

    rot = @(tendon_offset, theta) tendon_offset.* ...
        [cos(theta); sin(theta); zeros(1, length(theta))];
    D = rot(tendon_offset, tendon_angles(tendon));
    dD = zeros(3, 1);
    ddD = zeros(3, 1);

 
    %% H3 : 1 segment - 3 tendons - helical routing at 120 deg
case 3
    n_segments = 1;
    n_tendons = 3;
    l_j = 0.15;
    tendon_offset = 8e-3;

    phaseshift = R_theta_3(120*(tendon - 1));
    f = 2*pi/l_j;
    c = cos(X*f);
    s = sin(X*f);
    dc = -f*s;
    ds = f*c;
    ddc = -f*f*c;
    dds = -f*f*s;

    D = phaseshift*tendon_offset*[c; s; 0];
    dD = phaseshift*tendon_offset*[dc; ds; 0];
    ddD = phaseshift*tendon_offset*[ddc; dds; 0];
    %% P4 : 1 segment - 4 tendons - parallel routing at 90 deg
case 4
    n_segments = 1;
    n_tendons = 4;
    l_j = 0.15;
    tendon_offset = 8e-3;
    tendon_angles = [0, pi/2, pi, 3*pi/2];

    rot = @(r, theta) r .* [cos(theta); sin(theta); 0];
    D = rot(tendon_offset, tendon_angles(tendon));
    dD = zeros(3,1);
    ddD = zeros(3,1);

%% error if the demanded robot number corresponds to no implemented robot
otherwise
        error('please implement routing in "tendons.m" before using it');
end

end

function R = R_theta_3(theta)
    R = [ cosd(theta) -sind(theta)  0          ;
          sind(theta)  cosd(theta)  0          ;
          0            0            1         ];

end
