format short eng; close all; clc;

% ============================================================
% Load model from IDD
% ============================================================
load('setPoint75_amx2300_no_noise_integration.mat')
sysd = idss(setPoint75_amx2300_no_noise_integration);

%SSM
%sysc = SSM_O4_de1_PEM_MOESP_SIM;

% ============================================================
% Discretize model
% ============================================================
Ts   = 1.0;

% sysd = c2d(sysc, Ts, 'zoh');

A = sysd.A;
B = sysd.B;
C = sysd.C;

n = size(A,1);    % = 1

% ============================================================
% Augment with *leaky* integrator
% eI(k+1) = alpha*eI(k) + (r - y)
% ============================================================
alpha = 0.99;    % <<< CRITICAL (do NOT use 1.0)

Ae = [ A        zeros(n,1);
      -C        alpha      ]

Be = [ B;
       0 ]

Ce = [ C  0 ]

% ============================================================
% Check properties
% ============================================================
rankCtr = rank(ctrb(Ae,Be))
rankObs = rank(obsv(Ae,Ce))

O = obsv(Ae, Ce);
[U, S, V] = svd(O);
single_value_decomposition = diag(S)

condA  = cond(A)
condAe = cond(Ae)

% ============================================================
% LQR on augmented system (LQGI)
% ============================================================
Qx = diag([0  100  30])
Qu = 2e3

Le = dlqr(Ae, Be, Qx, Qu);

Lx = Le(1:n);
Li = Le(end);

disp("Lx:"); disp(Lx);
disp("Li:"); disp(Li);

% ============================================================
% Kalman filter on augmented system
% ============================================================
sigmaT = 0.2;
V = sigmaT^2;

% Increase process noise on UNOBSERVABLE states 0,1
W  = diag([1e-4 1e-4]);  % High noise on unobservable states
Wi = 1e-4;
We = blkdiag(W, Wi);

Ke = dlqe(Ae, eye(n+1), Ce, We, V);

Kx = Ke(1:n);
Ki = Ke(end);

disp("Kx:"); disp(Kx);
disp("Ki:"); disp(Ki);

% ============================================================
% Closed-loop poles
% ============================================================
p_ctrl = eig(Ae - Be*Le);
p_est  = eig(Ae - Ke*Ce);

disp("Controller poles:");
disp(p_ctrl)

disp("Estimator poles:");
disp(p_est)

% ============================================================
% Stability check (unit circle)
% ============================================================
max_abs_ctrl = max(abs(p_ctrl));
max_abs_est  = max(abs(p_est));

disp("Max |pole| (controller):")
disp(max_abs_ctrl)

disp("Max |pole| (estimator):")
disp(max_abs_est)

% Boolean checks
disp("Controller stable (<1):")
disp(all(abs(p_ctrl) < 1))

disp("Estimator stable (<1):")
disp(all(abs(p_est) < 1))
