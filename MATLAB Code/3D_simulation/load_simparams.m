function [simIn, matrices, inputs] = load_simparams(model)


% Construct & load params
[A_yz, B_yz, C_yz, D_yz, ~] = make_ballbot(); % yz plane
[A_xz, B_xz, C_xz, D_xz, ~] = deal(0);  % ideally some function makes this, or
                                        % unwraps from make_ballbot() call
[A_xy, B_xy, C_xy, D_xy, ~] = deal(0);  % "

ics_yz = [0; -0.3; 0; 0]; % initial conditions in minimal coords
ics_xz = [0; 0; 0; 0]; 
ics_xy = [0; 0; 0; 0];

planar_torque = [0; 0; 0]; % converted from motor torques to planar

% XZ Plane
simIn = simIn.setVariable("A_xz", A_xz);
simIn = simIn.setVariable("B_xz", B_xz);
simIn = simIn.setVariable("C_xz", C_xz);
simIn = simIn.setVariable("D_xz", D_xz);
simIn = simIn.setVariable("ics_xz", ics_xz);

% YZ Plane
simIn = simIn.setVariable("A_yz", A_yz);
simIn = simIn.setVariable("B_yz", B_yz);
simIn = simIn.setVariable("C_yz", C_yz);
simIn = simIn.setVariable("D_yz", D_yz);
simIn = simIn.setVariable("ics_yz", ics_yz);

% XY Plane
simIn = simIn.setVariable("A_xy", A_xy);
simIn = simIn.setVariable("B_xy", B_xy);
simIn = simIn.setVariable("C_xy", C_xy);
simIn = simIn.setVariable("D_xy", D_xy);
simIn = simIn.setVariable("ics_xy", ics_xy);

% Inputs
simIn = simIn.setVariable("planar_torque", planar_torque);

end
