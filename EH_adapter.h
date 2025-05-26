#include <unistd.h>
#include <iostream>
#include <filesystem>
#include <stdio.h>
#include <stdlib.h>


struct updateReturnValues {
    double T;
    int TX_counter;
    bool en_TX_counter;
    double Estored_ref;

};

double T_Update(double Estored_av, double Estored_EWSD, double DeltaEstored_sleep, double Edev, double C, double Vmax, double voltageThLow, double energyAwareSenderVoltageTh, double T);
double T_Update2(double Estored, double Estored_min, double Estored_max, double DeltaEstored, double Edev, double C, double Vmax, double voltageThLow, double energyAwareSenderVoltageTh, double T);
//double T_Update3(double Estored,double Estored_, double Edev, double C, double Vmax, double voltageThLow, double T,int TX_counter);
updateReturnValues T_Update3(double Estored,double Estored_, double DeltaEstored_sleep, double Edev, double C, double Vmax, double voltageThLow, double T, int TX_counter, bool en_TX_counte, double Estored_ref);
