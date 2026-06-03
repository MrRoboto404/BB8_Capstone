function [Tx, Ty, Tz] = Tmotors_to_Tplane(T1,T2,T3)
    alpha = params().alpha;
    beta  = params().beta;
    Tx = -(cos(alpha)*(T1*cos(beta))-T2*sin(beta + pi/6)+T3*sin(beta-pi/6));
    Ty = cos(alpha)*(-T1*sin(beta))-T2*cos(beta+pi/6)+T3*cos(beta-pi/6);
    Tz = T1 + T2 + T3;
end
