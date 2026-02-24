function [T1, T2, T3] = real_motor_torques_from_virtual(Tx, Ty, Tz, bb)
alpha = bb.alpha;
beta  = bb.beta;

cosa = cos(alpha);
cb = cos(beta);
sb = sin(beta);

T1 = (1/3) * ( ...
      Tz + (2/cosa) * (Tx*cb - Ty*sb) );

T2 = (1/3) * ( ...
      Tz + (1/cosa) * ( sb*(-sqrt(3)*Tx + Ty) ...
                      - cb*( Tx + sqrt(3)*Ty ) ) );

T3 = (1/3) * ( ...
      Tz + (1/cosa) * ( sb*( sqrt(3)*Tx + Ty) ...
                      + cb*(-Tx + sqrt(3)*Ty ) ) );

end