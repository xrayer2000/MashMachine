clc; close all; format compact; format short g;
clear all varibles;

Ts = 1;   % sampling time [s]
temps = [50 65 75];

% ================= FILES =================
estFiles = {
    "est_50_Mash Temperature and Power in-data-as-joinbyfield-2026-05-24 19_52_36.csv";
    "est_65_Mash Temperature and Power in-data-as-joinbyfield-2026-05-23 23_35_04.csv";
    "est_75_Mash Temperature and Power in-data-as-joinbyfield-2026-05-19 22_27_18.csv";
};

airEstFiles = {
    "est_50_Air Temperature-data-2026-05-24 19_49_46.csv";
    "est_65_Air Temperature-data-2026-05-23 23_35_13.csv";
    "est_75_Air Temperature-data-2026-05-19 22_27_53.csv";
};

valFiles = {
    "val_50_Mash Temperature and Power in-data-as-joinbyfield-2026-05-25 01_08_57.csv";
    "val_65_Mash Temperature and Power in-data-as-joinbyfield-2026-05-23 21_29_11.csv";
    "val_75_Mash Temperature and Power in-data-as-joinbyfield-2026-05-21 21_56_59.csv";
};

airValFiles = {
    "val_50_Air Temperature-data-2026-05-25 01_09_06.csv";
    "val_65_Air Temperature-data-2026-05-23 21_29_19.csv";
    "val_75_Air Temperature-data-2026-05-21 21_58_04.csv";
};

% ================= LOAD MULTIPLE ESTIMATION FILES =================
data_est = cell(numel(estFiles),1);

for k = 1:numel(estFiles)

    TT_est = loadMashCSV(estFiles{k}, airEstFiles{k}, Ts);

    U = [TT_est.Power, TT_est.AirTemp];
    data_est{k} = iddata(TT_est.Temp, U, Ts);
    data_est{k}.InputName = ["PowerIn","AirTemp"];
    data_est{k}.OutputName = "Temperature";

    % ================= FIGURE =================
    figure;
    yyaxis left
    plot(TT_est.Properties.RowTimes, TT_est.Temp, 'r', 'LineWidth', 2)
    ylabel('Temperature [°C]')
    
    yyaxis right
    plot(TT_est.Properties.RowTimes, TT_est.Power, 'b', 'LineWidth', 2.5)
    ylabel('Power [W]')
    ylim([0 2600])
    
    ax = gca;
    ax.YAxis(1).Color = 'r';
    ax.YAxis(2).Color = 'b';
    
    grid on
    xlabel('Time')
    legend('Temperature', 'Power In')
    title(sprintf('Estimation data - %d °C', temps(k)))
    end
    
    data_est_all = merge(data_est{:});

% ================= VALIDATION =================
for k = 1:numel(valFiles)
    TT_val = loadMashCSV(valFiles{k}, airValFiles{k}, Ts);
    if ~istimetable(TT_val)
        TT_val = table2timetable(TT_val);
    end
    U = [TT_val.Power, TT_val.AirTemp];
    data_val{k} = iddata(TT_val.Temp, U, Ts);
    data_val{k}.InputName = ["PowerIn","AirTemp"];
    data_val{k}.OutputName = "Temperature";
    ax = gca;
    figure;
    yyaxis left
    plot(TT_val.Properties.RowTimes, TT_val.Temp, 'r', 'LineWidth', 2)
    ylabel('Temperature [°C]')
    yyaxis right
    stairs(TT_val.Properties.RowTimes, TT_val.Power, 'b', 'LineWidth', 2.5)
    ylabel('Power [W]')
    ylim([0 2600])
    ax = gca;
    ax.YAxis(1).Color = 'r';
    ax.YAxis(2).Color = 'b';
    grid on
    xlabel('Time')
    legend('Temperature', 'Power In')
    title(sprintf('Validation data - %d °C', temps(k)))
end
data_vals = merge(data_val{:});

% ================= FUNCTION =================
function TT = loadMashCSV(fileMain, fileAir, Ts)

    % -------- MAIN FILE --------
    T1 = readtable(fileMain,'VariableNamingRule','preserve');

    Time1 = datetime(T1{:,1}/1000, ...
        'ConvertFrom','posixtime', ...
        'TimeZone','Europe/Stockholm');

    Temp  = str2double(string(T1{:,2}));
    Power = str2double(string(T1{:,3}));

    TT1 = timetable(Time1, Temp, Power);

    % -------- AIR TEMP FILE --------
    T2 = readtable(fileAir,'VariableNamingRule','preserve');

    Time2 = datetime(T2{:,1}/1000, ...
        'ConvertFrom','posixtime', ...
        'TimeZone','Europe/Stockholm');

    AirTemp = str2double(string(T2{:,2}));

    % Clean bad values
    AirTemp(AirTemp < -30 | AirTemp > 50) = NaN;

    TT2 = timetable(Time2, AirTemp);

    % -------- SYNCHRONIZE --------
    TT = synchronize(TT1, TT2, 'regular', 'mean', 'TimeStep', seconds(Ts));

    % -------- FILL --------
    TT.Temp    = fillmissing(TT.Temp ,'linear');
    TT.Power   = fillmissing(TT.Power,'previous');
    TT.AirTemp = fillmissing(TT.AirTemp,'previous');

end