% builds model from parameters. callable from simulink.

[A, B, C, D, sp] = makespring();
save('sm.mat', 'A', 'B', 'C', 'D', 'sp')