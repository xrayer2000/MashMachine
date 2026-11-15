clc; close all; format compact; format short g;
clear all varibles;


Ts = 1;   % sampling time [s]

% ================= FILES =================
estFiles = {
    "est_50_Mash Temperature (°C) and Power in-data-as-joinbyfield-2026-05-05";

};

valFiles = {

  
    "val_50_Mash Temperature (°C) and Power in-data-as-joinbyfield-2026-05-05";
};

% ================= LOAD MULTIPLE ESTIMATION FILES =================
data_est = cell(numel(estFiles),1);

for k = 1:numel(estFiles)

    % Load estimation data
    TT_est = loadMashCSV(estFiles{k}, Ts);

    data_est{k} = iddata(TT_est.Temp, TT_est.Power, Ts);
    data_est{k}.InputName  = "PowerIn";
    data_est{k}.OutputName = "Temperature";

    % ================= FIGURE: ESTIMATION =================
    figure;

    yyaxis left
    plot(TT_est.Time, TT_est.Temp, "r",'LineWidth',2)
    ylabel('Temperature [°C]','Color','r')
    ax = gca;
    ax.YColor = 'r';     

    yyaxis right
    stairs(TT_est.Time, TT_est.Power,"b", 'LineWidth',2.5)
    ylabel('Power [W]','Color','b')
    ax = gca;
    ax.YColor = 'b';      
    ylim([0 2600])

    grid on
    xlabel('Time')
    legend('Temperature','Power In')
       sgtitle(sprintf('Mashtun estimation data %d: Temperature y(t) and Power Input u(t)', k), 'Interpreter','latex')
    addAxisMargins(TT_est.Temp, TT_est.Power, 0.1, 0.1);

end

% Merge all estimation datasets into one
data_est_all = merge(data_est{:});


% ================= LOOP OVER VALIDATION FILES =================
for k = 1:numel(valFiles)

    %% -------- Load validation data --------
    TT_val = loadMashCSV(valFiles{k}, Ts);

    data_val{k} = iddata(TT_val.Temp, TT_val.Power, Ts);
    data_val{k}.InputName  = "PowerIn";
    data_val{k}.OutputName = "Temperature";

    %% ================= FIGURE: VALIDATION =================
    figure;

    yyaxis left
    plot(TT_val.Time, TT_val.Temp, "r",'LineWidth',2)
    ax = gca;
    ax.YColor = 'r';
    ylabel('Temperature [°C]')

    yyaxis right
    stairs(TT_val.Time, TT_val.Power, "b",'LineWidth',2.5)
    ylabel('Power [W]')
    ax = gca;
    ax.YColor = 'b';
    ylim([0 2600])

    grid on
    xlabel('Time')
    legend('Temperature','Power In')
    sgtitle(sprintf('Mashtun Validation data %d: Temperature y(t) and Power Input u(t)', k),'Interpreter','latex')
    addAxisMargins(TT_val.Temp, TT_val.Power, 0.1, 0.1);

end
data_vals = merge(data_val{:});







function addAxisMargins(yLeft, yRight, xMarginFrac, yMarginFracL, yMarginFracR)
% addAxisMargins  Add margins to x-axis and both y-axes (yyaxis-safe)
%
% yLeft, yRight : data vectors actually plotted
% xMarginFrac   : fraction of X-span (default 5 %)
% yMarginFracL  : fraction of left Y-span (default 5 %)
% yMarginFracR  : fraction of right Y-span (default 5 %)

    % ---------------- Defaults ----------------
    if nargin < 5 || isempty(yMarginFracR), yMarginFracR = 0.05; end
    if nargin < 4 || isempty(yMarginFracL), yMarginFracL = 0.05; end
    if nargin < 3 || isempty(xMarginFrac),  xMarginFrac  = 0.05; end

    ax = gca;

    % ---------------- X-axis ----------------
    xl = ax.XLim;

    if isdatetime(xl)
        dx = xl(2) - xl(1);
        if dx <= seconds(0)
            dx = seconds(1);
        end
        ax.XLim = [xl(1) - xMarginFrac*dx, ...
                   xl(2) + xMarginFrac*dx];
    else
        dx = xl(2) - xl(1);
        if dx == 0
            dx = 1;
        end
        ax.XLim = [xl(1) - xMarginFrac*dx, ...
                   xl(2) + xMarginFrac*dx];
    end

    % ---------------- Left Y-axis ----------------
    yyaxis left
    yl = yLeft(:);
    yl = yl(isfinite(yl));     % remove NaN/Inf

    if isempty(yl)
        return
    end

    minL = min(yl);
    maxL = max(yl);
    spanL = maxL - minL;

    if spanL == 0
        spanL = max(abs(yl)) + eps;
    end

    padL = yMarginFracL * spanL;
    ylim([minL - padL, maxL + padL]);

    % ---------------- Right Y-axis ----------------
    yyaxis right
    yr = yRight(:);
    yr = yr(isfinite(yr));     % remove NaN/Inf

    if isempty(yr)
        return
    end

    minR = min(yr);
    maxR = max(yr);
    spanR = maxR - minR;

    if spanR == 0
        spanR = max(abs(yr)) + eps;
    end

    padR = yMarginFracR * spanR;
    ylim([minR - padR, maxR + padR]);
end

function TT = loadMashCSV(filename, Ts)

    T = readtable(filename,'VariableNamingRule','preserve');

    % Time: epoch ms → datetime
    Time = datetime(T{:,1}/1000, ...
        'ConvertFrom','posixtime', ...
        'TimeZone','Europe/Stockholm');

    % Signals (handle 'undefined')
    Temp  = str2double(string(T{:,2}));
    Power = str2double(string(T{:,3}));

    % Remove rows where both are missing
    idx = ~(isnan(Temp) & isnan(Power));
    Time  = Time(idx);
    Temp  = Temp(idx);
    Power = Power(idx);

    % Remove duplicate timestamps
    [Time, ia] = unique(Time,'stable');
    Temp  = Temp(ia);
    Power = Power(ia);

    % Timetable
    TT = timetable(Time, Temp, Power);

    % Regularize
    TT = retime(TT,'regular','mean','TimeStep',seconds(Ts));

    % Fill missing
    TT.Temp  = fillmissing(TT.Temp ,'linear');
    TT.Power = fillmissing(TT.Power,'previous');
end