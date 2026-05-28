%% Solar Panel Digital Twin - automatic Simulink/Simscape builder
% Based on Chapter 6 figures/tables from "Solar Panel Digital Twin.pdf".
% The script creates the model, wires it, runs it for 6300 s, saves it, and
% plots voltage, power, and temperature-offset corrected power.

clear; clc; close all;

%% Model and simulation settings
modelName = 'SolarPanelDigitalTwin_Auto';
tStop = 6300;
Ts = 4;
TcellMask = 40;

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

load_system('simulink');
load_system('ee_lib');
load_system('fl_lib');
load_system('nesl_utility');

new_system(modelName);
open_system(modelName);

set_param(modelName, ...
    'StopTime', num2str(tStop), ...
    'SolverType', 'Variable-step', ...
    'Solver', 'ode23t', ...
    'MaxStep', num2str(Ts), ...
    'ReturnWorkspaceOutputs', 'on');

%% Input data
if evalin('base', "exist('time_s','var') && exist('irradiance_Wm2','var') && exist('temperature_C','var')")
    time_s = evalin('base', 'time_s(:)');
    irradiance_Wm2 = evalin('base', 'irradiance_Wm2(:)');
    temperature_C = evalin('base', 'temperature_C(:)');
else
    time_s = (0:Ts:tStop).';
    daylight = sin(pi*time_s/tStop);
    daylight(daylight < 0) = 0;
    irradiance_Wm2 = 420 + 410*daylight.^0.85 + 22*sin(2*pi*time_s/850);
    irradiance_Wm2 = max(50, min(1000, irradiance_Wm2));
    temperature_C = 36 + 15*daylight.^1.25 + 0.7*sin(2*pi*time_s/1200);
end

Ir_input = timeseries(irradiance_Wm2, time_s);
T_input = timeseries(temperature_C, time_s);
assignin('base', 'Ir_input', Ir_input);
assignin('base', 'T_input', T_input);
assignin('base', 'time_s', time_s);
assignin('base', 'irradiance_Wm2', irradiance_Wm2);
assignin('base', 'temperature_C', temperature_C);

%% Table 6.1 and Table 6.3 parameters
PV1 = struct('Name','PV1','Cells',45,'Rows',5,'Cols',9, ...
    'Pmax',85,'Vmp',17,'Imp',5.00,'Voc',21.5,'Isc',5.49, ...
    'BypassRatingA',10,'FuseA',9,'Tcell',TcellMask);
PV2 = struct('Name','PV2','Cells',36,'Rows',4,'Cols',9, ...
    'Pmax',100,'Vmp',17,'Imp',5.88,'Voc',21.5,'Isc',6.37, ...
    'BypassRatingA',12,'FuseA',10,'Tcell',TcellMask);
PV3 = struct('Name','PV3','Cells',36,'Rows',4,'Cols',9, ...
    'Pmax',100,'Vmp',17,'Imp',5.88,'Voc',21.5,'Isc',6.37, ...
    'BypassRatingA',12,'FuseA',10,'Tcell',TcellMask);

%% Exact Simscape block paths
solarCellPath = 'ee_lib/Sources/Solar Cell';
batteryPath = 'ee_lib/Sources/Battery';
diodePath = 'fl_lib/Electrical/Electrical Elements/Diode';
currentSensorPath = 'fl_lib/Electrical/Electrical Sensors/Current Sensor';
voltageSensorPath = 'fl_lib/Electrical/Electrical Sensors/Voltage Sensor';
electricalReferencePath = 'fl_lib/Electrical/Electrical Elements/Electrical Reference';
solverConfigPath = 'nesl_utility/Solver Configuration';
simulinkPSPath = 'nesl_utility/Simulink-PS Converter';
psSimulinkPath = 'nesl_utility/PS-Simulink Converter';
connectionPortPath = 'nesl_utility/Connection Port';

%% Inputs
add_block('simulink/Sources/From Workspace', [modelName '/irradiance'], ...
    'VariableName', 'Ir_input', 'Position', [40 80 170 110]);
add_block('simulink/Sources/From Workspace', [modelName '/temperature'], ...
    'VariableName', 'T_input', 'Position', [40 165 170 195]);
add_block('simulink/Sources/Clock', [modelName '/time'], ...
    'Position', [40 255 90 285]);

add_block(simulinkPSPath, [modelName '/Ir Simulink-PS'], ...
    'Unit', 'W/m^2', 'Position', [220 73 300 117]);
add_block('simulink/Sinks/Scope', [modelName '/IR'], 'Position', [220 20 280 60]);
add_block('simulink/Sinks/Scope', [modelName '/T'], 'Position', [220 145 280 205]);
add_block('simulink/Sinks/Scope', [modelName '/T1'], 'Position', [220 245 280 305]);
add_block('simulink/User-Defined Functions/Fcn', [modelName '/Temp_index'], ...
    'Expr', 'floor(u/4)+1', 'Position', [120 250 185 290]);

add_line(modelName, 'irradiance/1', 'Ir Simulink-PS/1', 'autorouting', 'on');
add_line(modelName, 'irradiance/1', 'IR/1', 'autorouting', 'on');
add_line(modelName, 'temperature/1', 'T/1', 'autorouting', 'on');
add_line(modelName, 'time/1', 'Temp_index/1', 'autorouting', 'on');
add_line(modelName, 'Temp_index/1', 'T1/1', 'autorouting', 'on');

%% PV subsystems
pv1Block = buildPVSubsystem(modelName, PV1, [370 70 510 210], solarCellPath, connectionPortPath);
pv2Block = buildPVSubsystem(modelName, PV2, [370 280 510 420], solarCellPath, connectionPortPath);
pv3Block = buildPVSubsystem(modelName, PV3, [370 490 510 630], solarCellPath, connectionPortPath);

pv1Ports = subsystemPorts(pv1Block);
pv2Ports = subsystemPorts(pv2Block);
pv3Ports = subsystemPorts(pv3Block);
irPS = get_param([modelName '/Ir Simulink-PS'], 'PortHandles');
add_line(modelName, irPS.RConn(1), pv1Ports.ir, 'autorouting', 'on');
add_line(modelName, irPS.RConn(1), pv2Ports.ir, 'autorouting', 'on');
add_line(modelName, irPS.RConn(1), pv3Ports.ir, 'autorouting', 'on');

%% Electrical network
add_block(currentSensorPath, [modelName '/PV1 Current Sensor'], 'Position', [590 95 690 165]);
add_block(currentSensorPath, [modelName '/PV2 Current Sensor'], 'Position', [590 305 690 375]);
add_block(currentSensorPath, [modelName '/PV3 Current Sensor'], 'Position', [590 515 690 585]);
add_block(voltageSensorPath, [modelName '/PV1 Voltage Sensor'], 'Position', [790 90 880 170]);
add_block(voltageSensorPath, [modelName '/PV23 Voltage Sensor'], 'Position', [790 360 880 440]);

add_block(diodePath, [modelName '/PV1 Bypass Diode 10A'], 'Position', [630 175 690 235]);
add_block(diodePath, [modelName '/PV2 Bypass Diode 12A'], 'Position', [630 385 690 445]);
add_block(diodePath, [modelName '/PV3 Bypass Diode 12A'], 'Position', [630 595 690 655]);

add_block(batteryPath, [modelName '/Battery'], 'Position', [1010 265 1090 405]);
set_param([modelName '/Battery'], 'Vnom', '12', 'R1', '0.05', 'AH', '320');

add_block(electricalReferencePath, [modelName '/Electrical Reference'], 'Position', [910 690 950 730]);
add_block(solverConfigPath, [modelName '/Solver Configuration'], ...
    'UseLocalSolver', 'off', ...
    'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
    'LocalSolverSampleTime', num2str(Ts), ...
    'DoDC', 'on', ...
    'Position', [1040 690 1120 750]);

cs1 = currentSensorPorts([modelName '/PV1 Current Sensor']);
cs2 = currentSensorPorts([modelName '/PV2 Current Sensor']);
cs3 = currentSensorPorts([modelName '/PV3 Current Sensor']);
vs1 = voltageSensorPorts([modelName '/PV1 Voltage Sensor']);
vs23 = voltageSensorPorts([modelName '/PV23 Voltage Sensor']);
d1 = diodePorts([modelName '/PV1 Bypass Diode 10A']);
d2 = diodePorts([modelName '/PV2 Bypass Diode 12A']);
d3 = diodePorts([modelName '/PV3 Bypass Diode 12A']);
bat = batteryPorts([modelName '/Battery']);
ref = get_param([modelName '/Electrical Reference'], 'PortHandles');
solver = get_param([modelName '/Solver Configuration'], 'PortHandles');

add_line(modelName, pv1Ports.p, cs1.p, 'autorouting', 'on');
add_line(modelName, cs1.n, bat.p, 'autorouting', 'on');
add_line(modelName, cs1.n, vs1.p, 'autorouting', 'on');

add_line(modelName, pv2Ports.p, cs2.p, 'autorouting', 'on');
add_line(modelName, cs2.n, bat.p, 'autorouting', 'on');
add_line(modelName, pv3Ports.p, cs3.p, 'autorouting', 'on');
add_line(modelName, cs3.n, bat.p, 'autorouting', 'on');
add_line(modelName, cs2.n, vs23.p, 'autorouting', 'on');

add_line(modelName, pv1Ports.n, ref.LConn(1), 'autorouting', 'on');
add_line(modelName, pv2Ports.n, ref.LConn(1), 'autorouting', 'on');
add_line(modelName, pv3Ports.n, ref.LConn(1), 'autorouting', 'on');
add_line(modelName, bat.n, ref.LConn(1), 'autorouting', 'on');
add_line(modelName, vs1.n, ref.LConn(1), 'autorouting', 'on');
add_line(modelName, vs23.n, ref.LConn(1), 'autorouting', 'on');
add_line(modelName, solver.RConn(1), ref.LConn(1), 'autorouting', 'on');

add_line(modelName, d1.anode, pv1Ports.n, 'autorouting', 'on');
add_line(modelName, d1.cathode, cs1.n, 'autorouting', 'on');
add_line(modelName, d2.anode, pv2Ports.n, 'autorouting', 'on');
add_line(modelName, d2.cathode, cs2.n, 'autorouting', 'on');
add_line(modelName, d3.anode, pv3Ports.n, 'autorouting', 'on');
add_line(modelName, d3.cathode, cs3.n, 'autorouting', 'on');

%% Measurements and outputs
addPSOut(modelName, 'I_PV1',  [950  70 1030 100], 'A');
addPSOut(modelName, 'V_PV1',  [950 120 1030 150], 'V');
addPSOut(modelName, 'I_PV2',  [950 270 1030 300], 'A');
addPSOut(modelName, 'I_PV3',  [950 320 1030 350], 'A');
addPSOut(modelName, 'V_PV23', [950 390 1030 420], 'V');

add_line(modelName, cs1.i, psInput([modelName '/I_PV1_PS']), 'autorouting', 'on');
add_line(modelName, vs1.v, psInput([modelName '/V_PV1_PS']), 'autorouting', 'on');
add_line(modelName, cs2.i, psInput([modelName '/I_PV2_PS']), 'autorouting', 'on');
add_line(modelName, cs3.i, psInput([modelName '/I_PV3_PS']), 'autorouting', 'on');
add_line(modelName, vs23.v, psInput([modelName '/V_PV23_PS']), 'autorouting', 'on');

add_block('simulink/Math Operations/Sum', [modelName '/PV23 Current Sum'], ...
    'Inputs', '++', 'Position', [1125 290 1155 340]);
add_block('simulink/Math Operations/Product', [modelName '/P_PV1 Product'], ...
    'Position', [1185 85 1225 145]);
add_block('simulink/Math Operations/Product', [modelName '/P_PV23 Product'], ...
    'Position', [1185 330 1225 390]);

add_line(modelName, 'I_PV2_PS/1', 'PV23 Current Sum/1', 'autorouting', 'on');
add_line(modelName, 'I_PV3_PS/1', 'PV23 Current Sum/2', 'autorouting', 'on');
add_line(modelName, 'I_PV1_PS/1', 'P_PV1 Product/1', 'autorouting', 'on');
add_line(modelName, 'V_PV1_PS/1', 'P_PV1 Product/2', 'autorouting', 'on');
add_line(modelName, 'PV23 Current Sum/1', 'P_PV23 Product/1', 'autorouting', 'on');
add_line(modelName, 'V_PV23_PS/1', 'P_PV23 Product/2', 'autorouting', 'on');

add_block('simulink/Math Operations/Sum', [modelName '/Battery Current Sum'], ...
    'Inputs', '++', 'Position', [1185 450 1225 500]);
add_block('simulink/Continuous/Integrator', [modelName '/Battery Charge Integrator'], ...
    'InitialCondition', '0', 'Position', [1260 455 1300 495]);
add_block('simulink/Math Operations/Gain', [modelName '/Ah to SOC Percent'], ...
    'Gain', '100/(320*3600)', 'Position', [1335 455 1415 495]);
add_block('simulink/Math Operations/Bias', [modelName '/Initial SOC 60pct'], ...
    'Bias', '60', 'Position', [1450 455 1510 495]);
add_block('simulink/Discontinuities/Saturation', [modelName '/SOC Saturation'], ...
    'UpperLimit', '100', 'LowerLimit', '0', 'Position', [1545 455 1605 495]);

add_line(modelName, 'I_PV1_PS/1', 'Battery Current Sum/1', 'autorouting', 'on');
add_line(modelName, 'PV23 Current Sum/1', 'Battery Current Sum/2', 'autorouting', 'on');
add_line(modelName, 'Battery Current Sum/1', 'Battery Charge Integrator/1', 'autorouting', 'on');
add_line(modelName, 'Battery Charge Integrator/1', 'Ah to SOC Percent/1', 'autorouting', 'on');
add_line(modelName, 'Ah to SOC Percent/1', 'Initial SOC 60pct/1', 'autorouting', 'on');
add_line(modelName, 'Initial SOC 60pct/1', 'SOC Saturation/1', 'autorouting', 'on');

addToWorkspace(modelName, 'V_PV1', [1300 95 1390 125]);
addToWorkspace(modelName, 'P_PV1', [1300 135 1390 165]);
addToWorkspace(modelName, 'V_PV23', [1300 340 1390 370]);
addToWorkspace(modelName, 'P_PV23', [1300 380 1390 410]);
addToWorkspace(modelName, 'I_PV1', [1300 175 1390 205]);
addToWorkspace(modelName, 'I_PV23', [1300 420 1390 450]);
addToWorkspace(modelName, 'Battery_SOC_est', [1640 460 1765 490]);

add_line(modelName, 'V_PV1_PS/1', 'V_PV1/1', 'autorouting', 'on');
add_line(modelName, 'P_PV1 Product/1', 'P_PV1/1', 'autorouting', 'on');
add_line(modelName, 'V_PV23_PS/1', 'V_PV23/1', 'autorouting', 'on');
add_line(modelName, 'P_PV23 Product/1', 'P_PV23/1', 'autorouting', 'on');
add_line(modelName, 'I_PV1_PS/1', 'I_PV1/1', 'autorouting', 'on');
add_line(modelName, 'PV23 Current Sum/1', 'I_PV23/1', 'autorouting', 'on');
add_line(modelName, 'SOC Saturation/1', 'Battery_SOC_est/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/Mux', [modelName '/PV1 Mux'], ...
    'Inputs', '3', 'Position', [1435 85 1465 165]);
add_block('simulink/Signal Routing/Mux', [modelName '/PV23 Mux'], ...
    'Inputs', '3', 'Position', [1435 330 1465 410]);
add_block('simulink/Sinks/Scope', [modelName '/PV1 plots'], ...
    'Position', [1510 70 1590 170]);
add_block('simulink/Sinks/Scope', [modelName '/PV2&3 plots'], ...
    'Position', [1510 315 1590 415]);
add_block('simulink/Signal Routing/Mux', [modelName '/Battery Mux'], ...
    'Inputs', '2', 'Position', [1640 520 1670 575]);
add_block('simulink/Sinks/Scope', [modelName '/Battery plots'], ...
    'Position', [1710 510 1790 585]);

add_line(modelName, 'V_PV1_PS/1', 'PV1 Mux/1', 'autorouting', 'on');
add_line(modelName, 'P_PV1 Product/1', 'PV1 Mux/2', 'autorouting', 'on');
add_line(modelName, 'I_PV1_PS/1', 'PV1 Mux/3', 'autorouting', 'on');
add_line(modelName, 'PV1 Mux/1', 'PV1 plots/1', 'autorouting', 'on');
add_line(modelName, 'V_PV23_PS/1', 'PV23 Mux/1', 'autorouting', 'on');
add_line(modelName, 'P_PV23 Product/1', 'PV23 Mux/2', 'autorouting', 'on');
add_line(modelName, 'PV23 Current Sum/1', 'PV23 Mux/3', 'autorouting', 'on');
add_line(modelName, 'PV23 Mux/1', 'PV2&3 plots/1', 'autorouting', 'on');
add_line(modelName, 'V_PV1_PS/1', 'Battery Mux/1', 'autorouting', 'on');
add_line(modelName, 'SOC Saturation/1', 'Battery Mux/2', 'autorouting', 'on');
add_line(modelName, 'Battery Mux/1', 'Battery plots/1', 'autorouting', 'on');

%% Offset-correction model from Figs. 6.28-6.34
add_block('simulink/User-Defined Functions/Fcn', [modelName '/PV1 Offset'], ...
    'Expr', '0.011*u^2 + 0.56*u - 64', 'Position', [820 520 980 560]);
add_block('simulink/User-Defined Functions/Fcn', [modelName '/PV23 Offset'], ...
    'Expr', '0.12*u^2 - 9*u + 140', 'Position', [820 585 980 625]);
add_block('simulink/Math Operations/Sum', [modelName '/P_PV1 Corrected Sum'], ...
    'Inputs', '++', 'Position', [1055 500 1090 555]);
add_block('simulink/Math Operations/Sum', [modelName '/P_PV23 Corrected Sum'], ...
    'Inputs', '++', 'Position', [1055 575 1090 630]);

add_line(modelName, 'temperature/1', 'PV1 Offset/1', 'autorouting', 'on');
add_line(modelName, 'temperature/1', 'PV23 Offset/1', 'autorouting', 'on');
add_line(modelName, 'P_PV1 Product/1', 'P_PV1 Corrected Sum/1', 'autorouting', 'on');
add_line(modelName, 'PV1 Offset/1', 'P_PV1 Corrected Sum/2', 'autorouting', 'on');
add_line(modelName, 'P_PV23 Product/1', 'P_PV23 Corrected Sum/1', 'autorouting', 'on');
add_line(modelName, 'PV23 Offset/1', 'P_PV23 Corrected Sum/2', 'autorouting', 'on');

addToWorkspace(modelName, 'P_PV1_corrected', [1140 510 1280 540]);
addToWorkspace(modelName, 'P_PV23_corrected', [1140 590 1280 620]);
addToWorkspace(modelName, 'PV1_power_offset', [1140 645 1280 675]);
addToWorkspace(modelName, 'PV23_power_offset', [1140 685 1280 715]);
add_line(modelName, 'P_PV1 Corrected Sum/1', 'P_PV1_corrected/1', 'autorouting', 'on');
add_line(modelName, 'P_PV23 Corrected Sum/1', 'P_PV23_corrected/1', 'autorouting', 'on');
add_line(modelName, 'PV1 Offset/1', 'PV1_power_offset/1', 'autorouting', 'on');
add_line(modelName, 'PV23 Offset/1', 'PV23_power_offset/1', 'autorouting', 'on');

%% Save, run, and plot
save_system(modelName);

try
    simOut = sim(modelName, 'StopTime', num2str(tStop));
catch ME
    warning("Initial simulation failed. Re-running with conservative Simscape settings.\n%s", ME.message);
    set_param(modelName, 'Solver', 'ode15s', 'MaxStep', '1');
    set_param([modelName '/Solver Configuration'], ...
        'UseLocalSolver', 'on', ...
        'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
        'LocalSolverSampleTime', num2str(Ts), ...
        'DoDC', 'on', ...
        'MaxNonlinIter', '10');
    simOut = sim(modelName, 'StopTime', num2str(tStop));
end

V_PV1_sim = getLoggedArray(simOut, 'V_PV1');
P_PV1_sim = getLoggedArray(simOut, 'P_PV1');
V_PV23_sim = getLoggedArray(simOut, 'V_PV23');
P_PV23_sim = getLoggedArray(simOut, 'P_PV23');
P_PV1_corr_sim = getLoggedArray(simOut, 'P_PV1_corrected');
P_PV23_corr_sim = getLoggedArray(simOut, 'P_PV23_corrected');

figure('Name','Voltage vs Time');
plot(V_PV1_sim(:,1), V_PV1_sim(:,2), 'LineWidth', 1.4); hold on;
plot(V_PV23_sim(:,1), V_PV23_sim(:,2), 'LineWidth', 1.4);
grid on; xlabel('Time [sec]'); ylabel('Voltage [V]');
title('Voltage vs Time');
legend('PV1', 'PV2 and PV3', 'Location', 'best');

figure('Name','Power vs Time');
plot(P_PV1_sim(:,1), P_PV1_sim(:,2), 'LineWidth', 1.4); hold on;
plot(P_PV23_sim(:,1), P_PV23_sim(:,2), 'LineWidth', 1.4);
grid on; xlabel('Time [sec]'); ylabel('Power [Watts]');
title('Power vs Time');
legend('PV1', 'PV2 and PV3', 'Location', 'best');

figure('Name','Corrected Power vs Time');
plot(P_PV1_sim(:,1), P_PV1_sim(:,2), '--', 'LineWidth', 1.0); hold on;
plot(P_PV1_corr_sim(:,1), P_PV1_corr_sim(:,2), 'LineWidth', 1.4);
plot(P_PV23_sim(:,1), P_PV23_sim(:,2), '--', 'LineWidth', 1.0);
plot(P_PV23_corr_sim(:,1), P_PV23_corr_sim(:,2), 'LineWidth', 1.4);
grid on; xlabel('Time [sec]'); ylabel('Power [Watts]');
title('Power vs Time with Temperature Offset');
legend('PV1 raw', 'PV1 corrected', 'PV2 and PV3 raw', 'PV2 and PV3 corrected', 'Location', 'best');

save_system(modelName);
fprintf('\nCreated, simulated, and saved %s.slx\n', modelName);

%% Helper functions
function pvBlock = buildPVSubsystem(modelName, pv, pos, solarCellPath, connectionPortPath)
    pvBlock = [modelName '/' pv.Name];
    add_block('simulink/Ports & Subsystems/Subsystem', pvBlock, 'Position', pos);
    cleanupDefaultSubsystemBlocks(pvBlock);

    add_block(connectionPortPath, [pvBlock '/Ir'], 'Port', '1', 'Side', 'Left', 'Position', [40 60 70 90]);
    add_block(connectionPortPath, [pvBlock '/p'],  'Port', '2', 'Side', 'Right', 'Position', [980 60 1010 90]);
    add_block(connectionPortPath, [pvBlock '/n'],  'Port', '3', 'Side', 'Right', 'Position', [980 390 1010 420]);

    mask = Simulink.Mask.create(pvBlock);
    mask.addParameter('Type','edit','Name','Isc','Prompt','Short circuit current','Value',num2str(pv.Isc));
    mask.addParameter('Type','edit','Name','Voc','Prompt','Open circuit voltage','Value',num2str(pv.Voc));
    mask.addParameter('Type','edit','Name','n','Prompt','Quality factor, N','Value','1.5');
    mask.addParameter('Type','edit','Name','Rs','Prompt','Series resistance','Value','0');
    mask.addParameter('Type','edit','Name','Ir0','Prompt','Reference irradiation','Value','1000');
    mask.addParameter('Type','edit','Name','T','Prompt','Temperature','Value',num2str(pv.Tcell));
    mask.Display = sprintf("disp('%s\\n%d cells in series')", pv.Name, pv.Cells);

    firstPlus = [];
    previousMinus = [];
    cellIndex = 0;
    orderedCells = serpentineOrder(pv.Rows, pv.Cols);

    for k = 1:size(orderedCells, 1)
        r = orderedCells(k, 1);
        c = orderedCells(k, 2);
        cellIndex = cellIndex + 1;
        x = 120 + (c-1)*90;
        y = 40 + (r-1)*80;
        cellBlock = sprintf('%s/Cell_%02d', pvBlock, cellIndex);
        add_block(solarCellPath, cellBlock, 'Position', [x y x+58 y+58]);

        set_param(cellBlock, ...
            'prm', 'ee.enum.sources.solar_cell_prm.datasheet', ...
            'Isc', 'Isc', ...
            'Voc', sprintf('Voc/%d', pv.Cells), ...
            'Ir0', 'Ir0', ...
            'ec', 'n', ...
            'Rs', sprintf('Rs/%d', pv.Cells), ...
            'N_series', '1', ...
            'N_parallel', '1', ...
            'Tmeas', 'T', ...
            'TFIXED', 'T');

        sp = solarPorts(cellBlock);
        if isempty(firstPlus)
            firstPlus = sp.p;
        else
            add_line(pvBlock, previousMinus, sp.p, 'autorouting', 'on');
        end
        previousMinus = sp.n;
        add_line(pvBlock, onlyConn([pvBlock '/Ir']), sp.ir, 'autorouting', 'on');
    end

    add_line(pvBlock, onlyConn([pvBlock '/p']), firstPlus, 'autorouting', 'on');
    add_line(pvBlock, previousMinus, onlyConn([pvBlock '/n']), 'autorouting', 'on');

    annotationText = sprintf('%s: Pmax=%g W, Vmp=%g V, Imp=%g A, Voc=%g V, Isc=%g A, bypass diode rating=%g A, fuse=%g A', ...
        pv.Name, pv.Pmax, pv.Vmp, pv.Imp, pv.Voc, pv.Isc, pv.BypassRatingA, pv.FuseA);
    ann = Simulink.Annotation(pvBlock, annotationText);
    ann.Position = [120 430 900 470];
end

function cleanupDefaultSubsystemBlocks(ss)
    defaults = {'In1','Out1'};
    for k = 1:numel(defaults)
        b = [ss '/' defaults{k}];
        if getSimulinkBlockHandle(b) > 0
            delete_block(b);
        end
    end
end

function order = serpentineOrder(rows, cols)
    order = zeros(rows*cols, 2);
    idx = 0;
    for c = 1:cols
        if mod(c, 2) == 1
            rowList = 1:rows;
        else
            rowList = rows:-1:1;
        end
        for r = rowList
            idx = idx + 1;
            order(idx, :) = [r c];
        end
    end
end

function ports = solarPorts(block)
    ph = get_param(block, 'PortHandles');
    ports.ir = ph.LConn(1);
    ports.p = ph.LConn(2);
    ports.n = ph.RConn(1);
end

function ports = currentSensorPorts(block)
    ph = get_param(block, 'PortHandles');
    ports.p = ph.LConn(1);
    ports.i = ph.RConn(1);
    ports.n = ph.RConn(2);
end

function ports = voltageSensorPorts(block)
    ph = get_param(block, 'PortHandles');
    ports.p = ph.LConn(1);
    ports.v = ph.RConn(1);
    ports.n = ph.RConn(2);
end

function ports = diodePorts(block)
    ph = get_param(block, 'PortHandles');
    ports.anode = ph.LConn(1);
    ports.cathode = ph.RConn(1);
end

function ports = batteryPorts(block)
    ph = get_param(block, 'PortHandles');
    ports.p = ph.LConn(1);
    ports.n = ph.RConn(1);
end

function ports = subsystemPorts(block)
    ph = get_param(block, 'PortHandles');
    allPorts = [ph.LConn(:); ph.RConn(:)];
    positions = zeros(numel(allPorts), 2);
    for k = 1:numel(allPorts)
        positions(k, :) = get_param(allPorts(k), 'Position');
    end
    [~, leftIdx] = min(positions(:,1));
    maxX = max(positions(:,1));
    rightIdx = find(abs(positions(:,1) - maxX) < 1e-9);
    [~, yOrder] = sort(positions(rightIdx,2));
    rightIdx = rightIdx(yOrder);
    ports.ir = allPorts(leftIdx);
    ports.p = allPorts(rightIdx(1));
    ports.n = allPorts(rightIdx(end));
end

function h = onlyConn(block)
    ph = get_param(block, 'PortHandles');
    h = [ph.LConn(:); ph.RConn(:)];
    h = h(1);
end

function addPSOut(modelName, signalName, pos, unitName)
    add_block('nesl_utility/PS-Simulink Converter', [modelName '/' signalName '_PS'], ...
        'Unit', unitName, 'Position', pos);
end

function addToWorkspace(modelName, varName, pos)
    add_block('simulink/Sinks/To Workspace', [modelName '/' varName], ...
        'VariableName', varName, ...
        'SaveFormat', 'Array', ...
        'Position', pos);
end

function h = psInput(block)
    ph = get_param(block, 'PortHandles');
    h = ph.LConn(1);
end

function data = getLoggedArray(simOut, varName)
    try
        data = simOut.get(varName);
    catch
        data = evalin('base', varName);
    end
    if isa(data, 'timeseries')
        data = [data.Time, data.Data(:)];
    elseif isvector(data)
        try
            t = simOut.get('tout');
        catch
            t = [];
        end
        if isempty(t) || numel(t) ~= numel(data)
            t = (0:numel(data)-1).';
        end
        data = [t(:), data(:)];
    elseif size(data, 2) == 1
        try
            t = simOut.get('tout');
        catch
            t = [];
        end
        if isempty(t) || size(data, 1) ~= numel(t)
            t = (0:size(data,1)-1).';
        end
        data = [t(:), data(:,1)];
    end
end