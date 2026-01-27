function [A,B,C,D,sp] = makespring()
% GEN PARAMS
sp.m = 2;      % kg
sp.k = 0.1;    % N/m
sp.b = 1;      % N*s/m

% MATRICES
A = [0, 1; -sp.k/sp.m, -sp.b/sp.m];
B = [0; 1/sp.m];
C = [1, 0];
D = 0;
