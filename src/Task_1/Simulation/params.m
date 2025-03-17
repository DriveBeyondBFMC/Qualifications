clear all;
%%
l = 26; % cm
v = 17; % cm/s
Ld = 25; % cm
K = 500;
tau = 2;
taud = 0.15;
tauSteer = 0.17;


%% Spline gen

% x = [0 40 50 50 50 20  -10 -10 -10 0];  % X-coordinates
% y = [0 0  10 30 80 100 80  30  10  0];  % Y-coordinates

x = [0  40 66.5 * sin(pi / 4) + 40       66.5 + 40 106 106 100*sin(pi/4) + 6   6    -100*sin(pi/4) + 6  -96 -96, 200 250  0  ];
y = [0  0  66.5 * (-cos(pi / 4) + 1)     66.5      116 266 266 + 100*sin(pi/4) 366  266 + 100*sin(pi/4) 266 100  50  -100 -10];

[xFine, yFine, R] = splineGenerator(x, y);

t_sim = linspace(0, 10, length(xFine))';

signalDataset = Simulink.SimulationData.Dataset;

ts_xFine = timeseries(xFine, t_sim);
ts_yFine = timeseries(yFine, t_sim);
ts_curvature = timeseries(R, t_sim);

signalDataset = signalDataset.addElement(ts_xFine, 'xFine');
signalDataset = signalDataset.addElement(ts_yFine, 'yFine');
signalDataset = signalDataset.addElement(ts_curvature, 'curvature');

save('splineSignal.mat', 'signalDataset');


