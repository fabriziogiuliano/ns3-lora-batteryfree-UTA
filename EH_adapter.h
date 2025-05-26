#include <unistd.h>
#include <iostream>
#include <filesystem>
#include <stdio.h>
#include <stdlib.h>
#include "EH_adapter.h"
#include <algorithm>
#include <math.h>
#include <iomanip>
#include <map>


double sigmoid(double x,double k,double x0, double L,double Tmin, double Tmax){
        return L*(Tmax-Tmin) / (1 + exp(-k*(x - x0))) + Tmin;
}


double sigmoid(double x,double k){
        return 4/ (1 + exp(-k*(x))) -2;
}

double f(double x,double k, double Tmin,double Tmax){        
        return (Tmax-Tmin)*exp(-k*x)+Tmin;
}

double x(double SoC,double k){
  return 2/ (1 + exp(-(k*SoC)));
}

double z(double H,double Hmax,double k){
  return 2/ (1 + exp(-(k-H/Hmax)));
}

double map_range(double inValue, double minInRange, double maxInRange, double minOutRange, double maxOutRange){
  double x = (inValue - minInRange) / (maxInRange - minInRange);
  return minOutRange + (maxOutRange - minOutRange) * x;
}

double T_Update_DDASA(double Estored, double DeltaEstored,double Edev, double C, double Vmax, double voltageThLow, double energyAwareSenderVoltageTh, double T){
  double Tmin = 1;
  double Tmax = 24;
  double Estored_max = 0.5 * C/1000.0 * Vmax * Vmax;
  //double E_th = 0.5 * C/1000.0 * voltageThLow * Vmax * voltageThLow * Vmax;


  double SoC,fnew,fcurr;   
  SoC = (Estored)/Estored_max; 
  //double H=1;
  //double Hmax=1;
  double k=-1;
  fcurr=1/(T/3600);
  double X = x(SoC,k);
  //fnew = fcurr*x(SoC,k)*z(H,Hmax,k);
  fnew = fcurr*X; //Ignore solar radiation coefficient
  double T_new = std::max(Tmin,std::min(Tmax,1/fnew));
  
  double rr = 1000;
  double T_new_sec = std::ceil(T_new * rr) / rr;
  T_new_sec = T_new*3600;
  bool en_print=true;

  if (en_print){
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    std::cout << "X="<<X<< "\t SoC="<<SoC<< "\t Edev="<<Edev<< "\t C="<<C<<"\tT="<<T<<"\t DeltaEstored="<<DeltaEstored<<"\t T_new [h]:" << T_new << "\t sec: " << T_new_sec << std::endl;  
  }

  return T_new_sec;
  
}


double T_Update_EASA(double Estored,double Edev, double C, double Vmax, double voltageThLow, double energyAwareSenderVoltageTh, double T){
  double Tmin = 1;
  double Tmax = 24;
  //Edev=0.442292; //hardcoded from Device specs, for 2B frame transmission TTGO
  Edev=0.00518609; // STM32???
  double Estored_max = 0.5 * C/1000.0 * Vmax * Vmax;
  double E_th = 0.5 * C/1000.0 * voltageThLow * Vmax * voltageThLow * Vmax;
  int n=5;
  double Xlevel = std::min(100.0,(n*Edev)/Estored_max*100);
  double Estored_perc = 100.0*((Estored-E_th)/Estored_max); // in %
  int m=1;
  double k = (1-pow((Xlevel-Estored_perc)/100,m));//funzione di costo
  k = (Estored_perc >= Xlevel ) ? 1 : k; 
  double f_sampling = 1.0/Tmin;
  double f_EASA = f_sampling * k;

  double T_new = std::max(Tmin,std::min(Tmax,1/f_EASA));

  double rr = 1000;
  T_new = std::round(T_new * rr) / rr;

  //convert to seconds
  double T_new_sec = T_new*3600;
  

  std::cout << "-----------------------------" << std::endl;
  std::cout << "Edev:" << Edev << std::endl;
  std::cout << "E_th:" << E_th << std::endl;
  std::cout << "Estored_max:" << Estored_max << std::endl;
  std::cout << "Xlevel[%]:" << Xlevel << std::endl;
  std::cout << "Estored[%]:" << Estored/Estored_max*100 << std::endl;
  std::cout << "k:" << k << std::endl;
  std::cout << "T_new [h]:" << T_new << std::endl;
  std::cout << "T_new [s]:" << T_new_sec << std::endl;
  std::cout << "-----------------------------" << std::endl;


  
  return T_new_sec;

}




/*
double  T_Update(double Estored_av, double DeltaEstored, double Edev, double C, double Vmax, double voltageThLow, double energyAwareSenderVoltageTh, double T){
  
  //double Estored_max = 0.5 * C/1000.0 * Vmax * Vmax;
  //double E_th = 0.5 * C/1000.0 * voltageThLow * Vmax * voltageThLow * Vmax;

  double T_new,k,vmin,vmax;   

  double nn = (Estored_av)/(Edev);
  //T_new = f(nn,1,Tmin,Tmax);
  k=1;
  vmin=2;
  vmax=0;
  double v = f(nn,k,vmin,vmax);
  T_new = T/3600/v;
  T_new = std::max(Tmin,std::min(Tmax,T_new));
  //T_new=24;
  double rr = 1000;
  double T_new_sec = std::ceil(T_new * rr) / rr;
  T_new_sec = T_new*3600;
  bool en_print=true;

  if (en_print){
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    std::cout << "Estored_av="<<Estored_av<< "\t Edev="<<Edev<<"\tnn="<<nn<<"\tv="<<v<<"\tT="<<T<<"\tT/v="<<T/3600/v<<"\t DeltaEstored="<<DeltaEstored<<"\t T_new [h]:" << T_new << "\t sec: " << T_new_sec << std::endl;  
  }
*/



double  T_Update_Fabrizio(double Estored_av, double Estored_EWSD, double DeltaEstored, double Edev, double C, double Vmax, double voltageThLow, double energyAwareSenderVoltageTh, double T){
  double Tmin = 1;
  double Tmax = 24;
  double T_new;
  double E_th = 0.5 * C/1000.0 * voltageThLow * voltageThLow * Vmax * Vmax ;
  double Estored_max = 0.5 * C/1000.0 * Vmax * Vmax;

/*
  if ((100.0<=C) && (C<=500.0)){
    optimism_coef=4; // factor to increase the expected transmissions with estimated energy
    prudence_coef=0.10; // reduce T of portion of hours to avoid energy estinguishing during negative DeltaEstored

  }else if ((50.0<=C) && (C<100.0)){
    optimism_coef=2; // factor to increase the expected transmissions with estimated energy
    prudence_coef=3.00; // reduce T of portion of hours to avoid energy estinguishing during negative DeltaEstored

  }else{
    optimism_coef=0.5; // factor to increase the expected transmissions with estimated energy
    prudence_coef=0; // reduce T of portion of hours to avoid energy estinguishing during negative DeltaEstored
  }
*/
/*
  if (C <=50){
    prudence_coef = 0.5*4;  //C=50
  }else{
    prudence_coef = Estored_max/DeltaEstored;  //C=100
  }
  */
  //double nn=optimism_coef/prudence_coef*(Estored_av-E_th)/Edev; //number of possibile transmissions for givene Estored versus Edev
  //double nn=std::round((Estored_av-E_th)/Edev); //number of possibile transmissions for given Estored versus Edev
  double nn=(Estored_av-E_th)/Edev; //number of possibile transmissions for given Estored versus Edev
  double gap = (Estored_av-E_th)/Estored_max; // varia tra 0 e 1. se è vicino a 
                                              //1 vuol dire che sono al massimo della carica, se è vicino a 0 vuol dire che sono vicino alla soglia
  T_new = Tmax/nn*gap;
  /*
  if (DeltaEstored >= 0){
    T_new = Tmax/nn;
  }else{
    T_new = Tmin*nn;
  }
  */
  T_new = std::max(Tmin,std::min(Tmax,T_new));

  
  double T_new_sec = T_new * 3600;
  bool en_print=false;
  if (en_print){
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    //std::cout << "Emax="<< Estored_max << "\tE_th"<< E_th << "\tEstored_av="<<Estored_av<< "\t Edev="<<Edev<< "\t nn="<<nn<< "\t C="<<C<< "\t optimism_coef="<<optimism_coef<< "\t prudence_coef="<<prudence_coef<<"\tT="<<T<<"\t DeltaEstored="<<DeltaEstored<<"\t T_new [h]:" << T_new << "\t sec: " << T_new_sec << std::endl;  
    std::cout << "Emax="<< Estored_max << "\tE_th"<< E_th << "\tEstored_av="<<Estored_av<<"\t DeltaEstored="<<DeltaEstored<< "\tgap="<<gap<< "\t Edev="<<Edev<< "\t nn="<<nn<< "\t C="<<C << "\tT="<<T<<"\t T_new [h]:" << T_new << "\t sec: " << T_new_sec << std::endl;  
  }



  return T_new_sec;
}


double T_Update(double Estored_av, double Estored_EWSD, double DeltaEstored, double Edev, double C, double Vmax, double voltageThLow, double energyAwareSenderVoltageTh, double T){
  //return T_Update_DDASA(Estored_av, DeltaEstored, Edev, C, Vmax, voltageThLow, energyAwareSenderVoltageTh, T); // preso dal paper ma non funziona
  return T_Update_Fabrizio(Estored_av, Estored_EWSD, DeltaEstored, Edev, C, Vmax, voltageThLow, energyAwareSenderVoltageTh, T);


  
}



updateReturnValues T_Update3(double Estored,double Estored_, double DeltaEstored_sleep, double Edev, double C, double Vmax, double voltageThLow, double T, int TX_counter, bool en_TX_counter,double Estored_ref){
  double Tmin = 1;
  double Tmax = 6;
  double Tday=24;
  double E_max = 0.5 * C/1000.0 * Vmax * Vmax;
  double E_th = 0.5 * C/1000.0 * voltageThLow * voltageThLow * Vmax * Vmax ;
  DeltaEstored_sleep=-0.01; //Delta stored for 1 hour HARDCODED FOR NOW
  //Edev=0.031; //Hardcoded!!!

  updateReturnValues ret;
  ret.TX_counter=TX_counter;
  ret.en_TX_counter=en_TX_counter;
  ret.Estored_ref = Estored_ref;
  ret.T = T;
  if (Edev <= 0)
    return ret;

  double T_new; 
  double nn;
  double deltaEstored = Estored - Estored_;
  double Tres = Tday - Tmin * (TX_counter);
  double Nres = Tres/Tmin;

  
  //if (TX_counter > 12)
  //  Edev+=0.005;

  //T_new = Tmin;    
  

  if (
      (Estored/E_max >=0.9)  // se ho tanta energia
      || ( (deltaEstored >=0) )  // OPPURE se ho un delta di energia positivo                                                
  ){
    T_new = Tmin;
    nn=0;
    if (ret.en_TX_counter==false){
      ret.TX_counter=0;
      ret.en_TX_counter=true;
    }
      
    ret.TX_counter++; //azzero il contatore di trasmissioni a Tmin e ci sommo il primo Tmin individuato :-)
    
  }else {
    // stop conteggio
    if (ret.en_TX_counter) ret.Estored_ref = Estored;
    ret.en_TX_counter=false;

    nn = (E_max - E_th + Nres*DeltaEstored_sleep)/Edev; 
    

    // n < 0 significa che l'energia totale di sleep è maggiore dell'energia accumulata, in questa condizione C è troppo sottodimensionato.
    // Workaround: cerco un valore alternativo di Nres che fa ottenere nn positivo.
    int Nres_mod = Nres;
    while (nn <0){
      Nres_mod--;
      nn = (Estored_ref - E_th + Nres_mod*DeltaEstored_sleep)/Edev; 
    }
    
    T_new = std::min(Tres/nn,Tres);
    

    T_new = std::max(Tmin,std::min(Tmax,T_new)); //Max delay of 12 hours!!
    //T_new = std::llround(T_new*4) / 4.0; //round to the nearest quarter of hour 
    T_new = std::llround(T_new); //round to the nearest hour 
    //T_new = std::llround(T_new*2) / 2.0; //round to the nearest half of hour 

  }


  double T_new_sec = int(T_new * 3600);


  ret.T = T_new_sec;

  bool en_print=false;
  if (en_print){
    std::cout << std::fixed;
    std::cout << std::setprecision(3);
    std::cout << "E_max="<< E_max<<"\tE_th="<< E_th<<"\tEdev="<< Edev<<"\tEstored="<< Estored<<"\tDeltaEstored_sleep="<< DeltaEstored_sleep<<"\tEstored_="<< Estored_<< "\tTX_counter="<< ret.TX_counter<< "\t en_TX_counter="<< ret.en_TX_counter <<"\tn="<< nn  << "\t C="<<C << "\tT="<<T<<"\t T_new [h]:" << T_new << "\t Tres: " << Tres << std::endl;
  } 

  return ret;

}

double T_Update2(double Estored_av, double Estored_min, double Estored_max, double DeltaEstored, double Edev, double C, double Vmax, double voltageThLow, double energyAwareSenderVoltageTh, double T){
  
  double Tmin = 1;
  double Tmax = 24;
  double T_new, nn;
  double E_th  = 0.5 * C/1000.0 * voltageThLow * voltageThLow * Vmax * Vmax ;
  double E_max = 0.5 * C/1000.0 * Vmax * Vmax;
  
  //nn=std::round((Estored_av+Estored_min-2*E_th)/(Edev));
  //nn=std::round(Estored_av/Edev);
  
  nn= 2*(Estored_av)/Edev ;
  double Tday = 24; //ore
  T_new = Tday/nn;
  T_new = std::max(Tmin,std::min(Tmax,T_new));

  //Analizzo energia minima del mio dataset. se l'energia minima è molto alta vuol dire che ho 
  double T_new_sec = T_new * 3600;
  bool en_print=false;
  if (en_print){
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    std::cout << "E_max="<< E_max<< "Estored_max="<< Estored_max << "\tE_th"<< E_th << "\tEstored_min="<<Estored_min<< "\t Edev="<<Edev<< "\t nn="<<nn<< "\t C="<<C << "\tT="<<T<<"\t T_new [h]:" << T_new << "\t sec: " << T_new_sec << std::endl;  
  }

  return T_new_sec;
}
