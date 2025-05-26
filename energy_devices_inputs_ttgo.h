// Energy Device Inputs
double BasicEnergySourceInitialEnergyJ = 0; // Energy in J
double BasicEnergySupplyVoltageV = 3.7; //in V
double StandbyCurrentA = 52e-3; //in A
double TxCurrentA = 113e-3; //PYCOM 0.1125
double IdleCurrentA = 52e-3; //45e-6
double SleepCurrentA = 40.5e-6;
double RxCurrentA = 61e-3;
bool EnterSleepIfDepleted = false;
double TurnOnDuration = 0.3; //Seconds
double TurnOnCurrentA = 0.015;
double OffCurrentA = 0.0000055;
double capacitance= 1000; // mF
double voltageThLow = 0.5; // 1.8 V
double voltageThHigh = 0.8; // 3 V
double energyAwareSenderVoltageTh = 3.3; // V