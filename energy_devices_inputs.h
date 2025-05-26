// Energy Device Inputs
double BasicEnergySourceInitialEnergyJ = 10000; // Energy in J
double BasicEnergySupplyVoltageV = 3.3; //in V
double StandbyCurrentA = 0.0014; //in A
double TxCurrentA = 0.028; // in A
//double IdleCurrentA =  0.000007; //45e-6
//double SleepCurrentA = 0.0000015;

double IdleCurrentA =  0.000007; // 7e-6
double SleepCurrentA = 0.0000015; // 1.5e-6

double RxCurrentA = 0.0112;
bool EnterSleepIfDepleted = false;
double TurnOnDuration = 0.3; //Seconds
double TurnOnCurrentA = 0.015;
double OffCurrentA = 0.0000055;
double capacitance= 100; // mF
double voltageThLow = 0.5; //5454; // 1.8 V
double voltageThHigh = 0.7090; // 3 V
double energyAwareSenderVoltageTh = 2.5; // V