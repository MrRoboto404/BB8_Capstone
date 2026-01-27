% called by simulink to re-update

[A, B, C, D, bb] = make_ballbot();
save('ballbot.mat', 'A', 'B', 'C', 'D', 'bb')