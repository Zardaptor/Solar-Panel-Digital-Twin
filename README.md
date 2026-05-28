Solar Panel Digital Twin
A MATLAB/Simulink and Simscape project that automatically builds, simulates, and saves a solar-panel digital twin model. The model represents three photovoltaic panels, irradiance and temperature inputs, electrical sensors, bypass diodes, battery charging behavior, and power/voltage analysis.

Project Overview
This project creates a Simulink/Simscape model named SolarPanelDigitalTwin_Auto. The MATLAB script programmatically places and wires the blocks, runs the simulation, logs key outputs, plots the results, and saves the final .slx model.

The project includes:

Automatic Simulink model generation
Three photovoltaic panel subsystems
Irradiance and temperature input profiles
Solar cell arrays built from datasheet-style parameters
Bypass diode modeling
Current and voltage sensing
12 V battery charging model
Estimated battery state of charge
PV power calculation
Temperature-offset corrected power analysis
Voltage and power plots
Main Files
src/
└── digital_twin.m

models/
└── SolarPanelDigitalTwin_Auto.slx

docs/
└── digital-twin-report.pdf
The docs/ file is optional. Upload it only if you want to include your project report/reference PDF.

Requirements
MATLAB
Simulink
Simscape
Simscape Electrical
How to Run
Open MATLAB and move into the repository folder:

cd path/to/Solar-Panel-Digital-Twin
Run the script:

run('src/digital_twin.m')
The script will:

Create a new Simulink model named SolarPanelDigitalTwin_Auto
Build the PV panel subsystems
Add sensors, bypass diodes, battery, and measurement blocks
Run the simulation for 6300 seconds
Plot voltage, power, and corrected power
Save the model as SolarPanelDigitalTwin_Auto.slx
You can also open the saved model directly:

open_system('models/SolarPanelDigitalTwin_Auto.slx')
PV Panel Configuration
The model contains three PV panels:

Panel	Cells	Layout	Pmax	Vmp	Imp	Voc	Isc
PV1	45	5 x 9	85 W	17 V	5.00 A	21.5 V	5.49 A
PV2	36	4 x 9	100 W	17 V	5.88 A	21.5 V	6.37 A
PV3	36	4 x 9	100 W	17 V	5.88 A	21.5 V	6.37 A
PV2 and PV3 are combined as a parallel PV bank in the measurement stage.

Simulation Inputs
The script can use workspace variables if they already exist:

time_s
irradiance_Wm2
temperature_C
If those variables are not present, the script automatically generates sample irradiance and temperature profiles.

Default simulation settings:

Stop time: 6300 s
Sample interval: 4 s
Solver: ode23t
Battery nominal voltage: 12 V
Battery capacity: 320 Ah
Initial SOC estimate: 60 percent
Logged Outputs
The model logs:

V_PV1
P_PV1
V_PV23
P_PV23
I_PV1
I_PV23
Battery_SOC_est
P_PV1_corrected
P_PV23_corrected
PV1_power_offset
PV23_power_offset
Applications
This project demonstrates how a digital twin can be used to study solar-panel behavior under changing irradiance and temperature conditions. It can be extended for solar monitoring, PV diagnostics, battery charging studies, and renewable-energy system simulation.
