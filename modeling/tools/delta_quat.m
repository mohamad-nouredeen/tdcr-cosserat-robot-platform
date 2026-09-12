function theta = delta_quat(q1, q2)
% DELTA_QUAT
% Returns the smallest orientation difference between two quaternions.
%
% Inputs:
%   q1, q2 : quaternions in scalar-first form [w; x; y; z]
%
% Output:
%   theta  : orientation difference [rad]

    % Force column vectors
    q1 = q1(:);
    q2 = q2(:);

    % Normalize quaternions
    q1 = q1 / norm(q1);
    q2 = q2 / norm(q2);

    % Quaternion sign ambiguity:
    % q and -q represent the same physical orientation.
    d = abs(dot(q1, q2));

    % Numerical protection
    d = max(-1, min(1, d));

    % Smallest relative rotation angle
    theta = 2 * acos(d);

end