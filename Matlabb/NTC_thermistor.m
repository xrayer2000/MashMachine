%% Steinhart–Hart calibration for your NTC
clc; clear; close all, format short eng

% ==============================
% Measured calibration points
% ==============================
% Resistance in Ohm, Temperature in Celsius
% R = [20.20e3; 50.45e3; 111.0e3];   % Ohm
% T_C = [67; 42; 24.45];               % °C

%real
R = [19.15e3; 49.8e3; 113.1e3];   % Ohm
T_C = [67.5; 41.95; 22.4];   

% Convert temperature to Kelvin
T_K = T_C + 273.15;

% ==============================
% Solve for A, B, C
% ==============================
X = [ ...
    ones(3,1), ...
    log(R), ...
    (log(R)).^3 ...
];

Y = 1 ./ T_K;

ABC = X \ Y;

A = ABC(1)
B = ABC(2)
C = ABC(3)

% ==============================
% Verification at calibration points
% ==============================
T_check = 1 ./ (A + B*log(R) + C*(log(R)).^3) - 273.15;

disp('Verification at calibration points:')
table(R, T_C, T_check, ...
    'VariableNames', {'Resistance_Ohm','Measured_T_C','Model_T_C'})

% ==============================
% Plot temperature curve
% ==============================
R_plot = linspace(10e3, 150e3, 2000);   % Ohm
T_plot = 1 ./ (A + B*log(R_plot) + C*(log(R_plot)).^3) - 273.15;

figure;
plot(R_plot, T_plot, 'LineWidth', 2); hold on;
scatter(R, T_C, 60, 'filled');   % calibration points
grid on;
xlabel('Resistance (Ohm)');
ylabel('Temperature (°C)');
title('Steinhart–Hart calibrated NTC');
legend('Steinhart–Hart curve','Calibration points','Location','Best');
