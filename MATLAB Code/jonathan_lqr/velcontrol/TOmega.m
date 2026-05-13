data = readtable('T12.txt');
figure
plot(data.time, data.vel)
figure
plot(data.time,data.T_estimate,data.time,data.T_target)

%%
data = readtable('T15.txt');
figure
plot(data.time, data.vel)
figure
plot(data.time,data.T_estimate,data.time,data.T_target)

%%
data = readtable('T20.txt');
figure
plot(data.time, data.vel)
figure
plot(data.time,data.T_estimate,data.time,data.T_target)

%%
figure
data = readtable('T12.txt');
plot(data.time, data.vel)
hold on
data = readtable('T15.txt');
plot(data.time, data.vel)
data = readtable('T20.txt');
plot(data.time, data.vel)
%%
figure
data = readtable('T12.txt');
plot(data.time,data.T_estimate,data.time,data.T_target)

hold on
data = readtable('T15.txt');
plot(data.time,data.T_estimate,data.time,data.T_target)
data = readtable('T20.txt');
plot(data.time,data.T_estimate,data.time,data.T_target)
