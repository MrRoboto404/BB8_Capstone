function [A,B,C,D,sp] = makespring()
% GEN PARAMS
sp.m = 1;      % kg
sp.k = 50;    % N/m
sp.b = 2;      % N*s/m

% MATRICES
A = [0, 1; -sp.k/sp.m, -sp.b/sp.m];
B = [0; 1/sp.m];
C = [1, 0];
D = 0;
