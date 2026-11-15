format short g; close all; clc;
rng(34);

% ============================================================
% Load & discretize
% ============================================================
load('air_SSM_65.mat');
sysc = idss(air_SSM_65);

sysc_nodelay = sysc;
sysc_nodelay.InputDelay = 0;

Ts   = 1.0;
sysd = c2d(sysc_nodelay, Ts, 'zoh');

A  = sysd.A;
Bq  = sysd.B(:,1);   % heater power
Bair = sysd.B(:,2);   %
C  = sysd.C;        % [1 0]
n  = size(A,1);     % 2

% ============================================================
% Operating point from estimation data
% ============================================================
T0 = mean(data_est_all.OutputData{2});

U0 = data_est_all.InputData{2};

Q0    = mean(U0(:,1));
Tair0 = mean(U0(:,2));

fprintf('\n=== Operating point (Exp1 / 65°C) ===\n')
fprintf('T0    = %.4f °C\n', T0)
fprintf('Q0    = %.4f W\n',  Q0)
fprintf('Tair0 = %.4f °C\n\n', Tair0)

% ============================================================
% State scaling
% ============================================================
S    = diag([1, 1e3]);
Sinv = diag([1, 1e-3]);

As = S * A * Sinv;
Bs = S * Bq;
Bs_air = S * Bair;
Cs = C * Sinv;      % [1 0]

fprintf('=== Scaled plant ===\n');
disp('As:'); disp(As)
disp('Bs:'); disp(Bs)
disp('Cs:'); disp(Cs)
fprintf('cond(As) = %.2f\n\n', cond(As))

% ============================================================
% Equilibrium state
% ============================================================
u0 = [Q0; Tair0];

x0 = (eye(n)-As)\([Bs Bs_air]*u0);

fprintf('=== Equilibrium state ===\n')
disp(x0)

fprintf('Predicted equilibrium output:\n')
disp(Cs*x0)

% ============================================================
% Augmented system (for LQR design only)
% ============================================================
alpha = 0.99;

Ae = [As,  zeros(n,1);
     -Cs,  alpha     ];
Be = [Bs;  0];

fprintf('=== Augmented system (LQR only) ===\n');
disp('Ae:'); disp(Ae)
disp('Be:'); disp(Be)

fprintf('rank(ctrb) = %d  (need %d)\n', rank(ctrb(Ae,Be)), n+1)
fprintf('rank(obsv(Ae,[Cs 0])) = %d  (need %d — will be 2, by design)\n\n', ...
         rank(obsv(Ae,[Cs 0])), n+1)

% ============================================================
% LQR on full augmented system
% ============================================================
Qy = 100;
Qi = 1;
Qu = 2;
Qx = diag([Qy, 0, Qi]);

Le = dlqr(Ae, Be, Qx, Qu);
Lx = Le(1:n);
Li = Le(end);

fprintf('=== LQR gains ===\n');
fprintf('Lx = [%.6f  %.6f]\n', Lx(1), Lx(2));
fprintf('Li =  %.6f\n\n',      Li);

p_ctrl = eig(Ae - Be*Le);
fprintf('Controller poles:\n'); disp(p_ctrl)
fprintf('Max |pole|: %.6f\n',  max(abs(p_ctrl)))
fprintf('Stable: %d\n\n',      all(abs(p_ctrl) < 1))

% ============================================================
% Plant-only Kalman (2x2)
% xI is not estimated — updated directly from measurement
% ============================================================
sigmaT = 0.037 / sqrt(12); %Max 31865 16bit, 0.037 quantization
V_kf   = sigmaT^2;
sigmaW1 = 1e-2;   % temperature state — relatively well known
sigmaW2 = 1e-1;   % hidden dynamic — more uncertain
We_kf   = diag([sigmaW1^2, sigmaW2^2]);

Ke = dlqe(As, eye(n), Cs, We_kf, V_kf);

fprintf('=== Plant Kalman gains ===\n');
fprintf('Ke = [%.6f  %.6f]\n\n', Ke(1), Ke(2));

p_est = eig(As - Ke*Cs);
fprintf('Estimator poles:\n'); disp(p_est)
fprintf('Max |pole|: %.6f\n',  max(abs(p_est)))
fprintf('Stable: %d\n\n',      all(abs(p_est) < 1))

fprintf('rank(obsv) plant = %d  (need %d)\n', rank(obsv(As,Cs)), n)
[~,S_o,~] = svd(obsv(As,Cs));
fprintf('svd(obsv):\n'); disp(diag(S_o))

% ============================================================
% Export matrices for C++
% ============================================================
fprintf('\n=== C++ MODEL CONSTANTS ===\n')
fprintf('//As\n')
fprintf('{{%.8f,%.8f},\n',As(1,1),As(1,2))
fprintf('{%.8f,%.8f}}\n\n',As(2,1),As(2,2))
fprintf('//Bs\n')
fprintf('{%.8f,%.8f}\n\n',Bs(1),Bs(2))
fprintf('//Ee\n')
fprintf('{%.8f,%.8f}\n\n',Bs_air(1),Bs_air(2))
fprintf('//Cs=[1 0]\n\n')
fprintf('//Le\n')
fprintf('{%.8f,%.8f,%.8f}\n\n',Le(1),Le(2),Le(3))
fprintf('//Ke\n')
fprintf('{%.8f,%.8f}\n\n',Ke(1),Ke(2))
fprintf('//alpha=%.4f\n',alpha)
fprintf('//x2_physical=x2_scaled*1e-3\n\n')

% ============================================================
% Simulation — split LQGI
% ============================================================
r      = 65.0;
Tair   = 12.0;
kLoss  = 3.5;
Qmax   = 2200.0;
Qmin   = 0.0;
N      = 4000; %min
band1_ = 4.5;
band2_ = 0.5;
gamma_ = 0.8;

% Initial conditions
xhat = [67; 0.0];
xI   = 0.0;
x    = [67; 0.0];

% Pre-allocate logs
T_log        = zeros(1, N);
Q_log        = zeros(1, N);
xI_log       = zeros(1, N);
innov_log    = zeros(1, N);
phase_log    = zeros(1, N);
lqgi_sat_log = false(1, N);

% ============================================================
for k = 1:N
    T = x(1);
    T_log(k) = T;
    xI_log(k) = xI;

    error  = r - xhat(1); %xhat(1) kalman estimate of T
    Qff = kLoss * (r - Tair);
    % Qff = 0;
    u_lqgi = 0;
    % u_Kr   = 5 * kLoss * error;
    u_Kr = 0;

    % =========================================================
    % PHASE LOGIC
    % =========================================================
    if error > band1_
        % ---- PHASE 1: BANG ----
        Qcmd         = Qmax;
        phase_log(k) = 1;

    elseif error > band2_
        % ---- PHASE 2: SLOPE ----
        deltaY = Qmax - Qff - u_lqgi - u_Kr;
        deltaX = band1_ - band2_;
        t      = (band1_ - error) / deltaX;
        t      = max(0, min(1, t));
        s      = t ^ gamma_;
        Qcmd   = Qmax - deltaY * s;
        Qcmd   = max(Qff + u_lqgi + u_Kr, min(Qmax, Qcmd));
        phase_log(k) = 2;

    else
        % ---- PHASE 3: REGULATE (LQGI) ----
        Ttilde = (xhat(1)) - r;
        u_raw  = -Le(1) * Ttilde ...
                 -Le(2) * xhat(2) ...
                 -Le(3) * xI;

        % gL    = min(1, abs(error) / 0.3);
        % u_raw = u_raw * gL;

        u_lqgi          = u_raw;
        lqgi_sat_log(k) = (u_raw < 0);

        Qcmd         = Qff + u_lqgi + u_Kr;
        phase_log(k) = 3;
        xI           = alpha*xI + (r - T);   % ← here
    end

    % Outside and after the if/elseif/else — reset when NOT in phase 3
    % if phase_log(k) ~= 3
    %     xI = 0;
    % end

    Q        = max(min(Qcmd, Qmax), Qmin);
    Q_log(k) = Q;

    % True plant
    x = As*x + Bs*Q + Bs_air*Tair;
    x(1) = x(1) + sigmaT*randn();
    
    % Kalman
    xpred = As*xhat + Bs*Q + Bs_air*Tair;
    innov = x(1) - xpred(1);
    innov_log(k) = innov;
    xhat  = xpred + Ke*innov;
end

% Fixed point check
x_ss = (eye(n) - As) \ (Bs*Q0 + Bs_air*Tair0);
fprintf('Fixed point: T=%.4f (should be %.4f)\n', x_ss(1), T0)

T_offset = x_ss(1) - T0;
fprintf('Offset: %.4f\n', T_offset)

% ============================================================
% Plots
% ============================================================

tmin = (0:N-1)/60;

figure('Units','normalized','OuterPosition',[0 0 1 1]);

subplot(4,1,1)
idx1 = phase_log==1;
idx2 = phase_log==2;

area(tmin(idx1),repmat(r+10,1,sum(idx1)),Tair-2,...
    'FaceColor',[0.5 1 1],'FaceAlpha',0.3,'EdgeColor','none');
hold on

area(tmin(idx2),repmat(r+10,1,sum(idx2)),Tair-2,...
    'FaceColor',[0.98 0.5 0.95],'FaceAlpha',0.3,'EdgeColor','none');

plot(tmin,T_log,'b','LineWidth',1.2)
yline(r,'r--')

ylabel('Temp (°C)')
title('Split LQGI')
grid on

subplot(4,1,2)
plot(tmin,Q_log,'k')
ylabel('Power (W)')
grid on

subplot(4,1,3)
plot(tmin,xI_log,'g')
ylabel('xI')
grid on

subplot(4,1,4)
plot(tmin,innov_log,'m')
ylabel('Innovation')
xlabel('Time (min)')
grid on