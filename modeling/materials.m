function [E, G, Ro, Ri, rho] = materials()

    E   = 83e9;            % [Pa] preliminary measured modulus
    nu  = 0.33;              % [-] nominal Poisson's ratio for Nitinol
    G = E/(2*(1 + nu));    % [Pa] approximately 31.20 GPa    % [Pa] = 31.28 GPa
     % Hollow tube geometry
    Ro = 1.220e-3/2;    % [m] outer radius = 0.610 mm
    Ri = 0.954e-3/2;    % [m] inner radius = 0.477 mm
    rho = 6450;              % [kg/m^3] nominal Nitinol density

end
