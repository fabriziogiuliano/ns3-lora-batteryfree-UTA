#include "ns3/callback.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/lora-device-address.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-phy.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include <algorithm>
#include <ctime>
#include "ns3/animation-interface.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/file-helper.h"
#include <sys/stat.h>
#include <sys/types.h>

#include <unistd.h>
#include <iostream>
#include <filesystem>
#include <stdio.h>
#include <stdlib.h>
#include <ns3/energy-aware-sender-helper.h>
#include <ns3/one-shot-sender-helper.h>
#include <ns3/capacitor-energy-source-helper.h>
#include <ns3/basic-energy-harvester-helper.h>
#include <ns3/variable-energy-harvester-helper.h>

//#include "ns3/mpi-interface.h"
//#include <mpi.h>


double Tmin = 1;
double Tmax = 24;

#include "EH_adapter.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("Energy-Harveseting-Capacitor");

// Network settings
uint nDevices = 0;
int nGateways = 2;
double simulationTime = 1*60*60;
int appPeriodSeconds = 1*60*60;
int runId = 1;
// Channel model
bool realisticChannelModel = false; // Channel model
bool enable_T_adapt=false;
// Energy Device Inputs
#include "energy_devices_inputs.h"
//#include "energy_devices_inputs_ttgo.h"

double E = BasicEnergySupplyVoltageV; // V


//double eh = 0.0001; // fixed EH value
double eh = -1; // for negative values use variable Solar Harvester from filenameHarvesterSun
std::string pathToInputFile = "/home/testbed5g/NS3-lorawan2/ns-3-dev-git/scratch/energy-harvesting-capacitor"; 
//std::string filenameHarvesterSun = "/energy_csv/solar_power_palermo_winter.csv";

// Network Inputs

double radius = 1000;
int packetSize = 2;

double eaMaxDesyncDelay = 1;
int confirmed = 0;
bool print = false; // Output control
std::string sender = "periodicSender";


//TODO: capire se servono e com si usano
std::string filenameRemainingVoltage = pathToInputFile  + "/remainingVoltage.txt";
std::string filenameEnergyConsumption = "energyConsumption.txt";
std::string filenameRemainingEnergy = "remainingEnergy.txt";


std::string filenameTx = pathToInputFile  +  "/deviceTx.txt";
std::string filenameEnoughEnergy = "energyEnoughForTx.txt"; // NOT USED!

std::string filenameHarvesterCloudy = "/outputixys_cloudy.csv";
bool remainingVoltageCallbackFirstCall = true;
bool energyConsumptionCallbackFirstCall = true;
bool enoughEnergyCallbackFirstCall = true;
bool stateChangeCallbackFirstCall = true;
bool TxCallbackFirstCall = true;
int generatedPacketsAPP = 0;

std::vector<int> packetsSent (6, 0);
std::vector<int> packetsReceived (6, 0);

std::stringstream buffer;
std::stringstream adr_buffer;

std::stringstream pkt_log_buffer;  
std::vector<std::vector<std::string>> gw_pos;
std::vector<std::vector<std::string>> dev_pos;
std::vector<std::vector<std::string>> adr_values;

int Estored_list_size = 100;
//Device Energy Memory
std::map<int,double> Estored_;
std::map<int,int> Estored_index;
std::map<int,int> TX_counter;
std::map<int,int> TX_counter_start;

std::map<int,std::map<int,double>> Estored_list;
std::map<int,double> Estored_EWMA;
std::map<int,double> Estored_EWSD;
std::map<int,double> Estored_tx;
std::map<int,double> Estored_tx_;
std::map<int,double> Estored_ref;


std::map<int,std::string> state;



std::map<int,double> DeltaEstored_sleep;
std::map<int,double> Estored_tx_max;

int Edev_list_size = 100;
std::map<int,std::map<int,double>> Edev_list;
std::map<int,int> Edev_index;

std::map<int,double> Estored_sleep;
std::map<int,double> Estored_sleep_max;
std::map<int,bool> is_sleep;
std::map<int,bool> is_turnon;
std::map<int,double> T_tx;
std::map<int,double> T_sleep;

std::string pv_l="75";
std::string pv_h="135"; 

int dr_fixed = 5; //SF12
bool custom_node_position=false;
bool enable_gw_position=false;
bool enable_pkt_log = true;
bool en_adr = false;

bool enable_energy_log = true;

bool is_confirmed = false;

std::vector<FileHelper> fileHelpers;
std::string output_dir = "output_files_test";
//std::string adr_conf = "SF_allocation/ADR.csv";
std::string adr_conf = "";

std::string season = "winter";

bool extract_adr = false;

NodeContainer endDevices;


void
OnRemainingEnergyChange (double oldRemainingEnergy, double remainingEnergy)
{
  // NS_LOG_DEBUG (Simulator::Now().GetSeconds() << " " << remainingEnergy);
}


void
OnDeviceEnergyConsumption (double oldvalue, double energyConsumption)
{
  const char *c = filenameEnergyConsumption.c_str ();
  std::ofstream outputFile;
  if (energyConsumptionCallbackFirstCall)
    {
      // Delete contents of the file as it is opened
      outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
      // Set the initial sleep state
      outputFile <<  "0 0" << std::endl;
      energyConsumptionCallbackFirstCall = false;
    }
  else
    {
    // Only append to the file
    outputFile.open (c, std::ofstream::out | std::ofstream::app);
    }

  outputFile << Simulator::Now ().GetSeconds () << " " << energyConsumption << std::endl;
  outputFile.close ();
}


void
OnRemainingVoltageChange (std::string context,
                          double oldRemainingVoltage, double remainingVoltage)
{

  // NS_LOG_DEBUG("Callback " << context);
  const char *c = filenameRemainingVoltage.c_str ();
  std::ofstream outputFile;
  if (remainingVoltageCallbackFirstCall)
    {
      // Delete contents of the file as it is opened
      outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
      remainingVoltageCallbackFirstCall = false;
    }
  else
    {
      // Only append to the file
      outputFile.open (c, std::ofstream::out | std::ofstream::app);
    }

  outputFile << context << " "
             << Simulator::Now ().GetMilliSeconds ()
             << " " << remainingVoltage
             << std::endl;
  outputFile.close ();
  
  /*
  if (context == "0"){
    std::cout << Simulator::Now().GetSeconds()<<std::endl;
  }
  */
  
}


void OnEnergyHarvested (double oldvalue, double totalEnergyHarvested)
{
  NS_LOG_DEBUG("Total energy harvested: " << totalEnergyHarvested);
  NS_LOG_DEBUG ("Energy harvested in this interval: " << totalEnergyHarvested - oldvalue);
}


void
EnergyDepletionCallback (void)
{
  NS_LOG_DEBUG("Energy depleted callback in main");
}

void
CheckEnoughEnergyCallback (uint32_t nodeId, Ptr<const Packet> packet,
                           Time time, bool boolValue)
{
  const char *c = filenameEnoughEnergy.c_str ();
  std::ofstream outputFile;
  if (enoughEnergyCallbackFirstCall)
    {
      // Delete contents of the file as it is opened
      outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
      enoughEnergyCallbackFirstCall = false;
    }
  else
    {
      // Only append to the file
      outputFile.open (c, std::ofstream::out | std::ofstream::app);
    }

  outputFile << Simulator::Now ().GetSeconds () << " " << boolValue << std::endl;
  outputFile.close ();

}

// get MAX
double getMax(std::map<int,double> aa, int s){
  double curr_max=0;
  for (int i=0;i<s;i++){
    if (aa[i] > curr_max){
      curr_max=aa[i];
    }
  }
  return curr_max;
}

// get MIN
double getMin(std::map<int,double> aa, int s){
  double curr_min=1e6;
  for (int i=0;i<s;i++){
    if ((aa[i] < curr_min) && (aa[i] > 0 )){
      curr_min=aa[i];
    }
  }
  return curr_min;
}
std::map<int,double> resetEstored(){
  std::map<int,double> aa;
  return aa;
}

// get Mean
double getMean(std::map<int,double> aa, int s){
  double curr_max=0;
  for (int i=0;i<s;i++){
    if (aa[i] > curr_max){
      curr_max=aa[i];
    }
  }
  return curr_max/s;
}


void OnEndDeviceStateChange (std::string context,
                        EndDeviceLoraPhy::State oldstatus,
                        EndDeviceLoraPhy::State status) {


  uint32_t systemId = stoi(context);
  Ptr<Node> node = endDevices.Get(systemId);
  Ptr<PeriodicSender> periodic_sender = node->GetApplication(0)->GetObject<PeriodicSender>(); // OK
  Ptr<EnergySource> energy_source = node->GetObject<EnergySourceContainer>()->Get(0)->GetObject<EnergySource>(); // OK
  int loss_flag=0;
  //double E_th;
  double Estored = energy_source->GetRemainingEnergy();
  
  double Edev=(Estored_tx_max[systemId] - Estored_sleep_max[systemId]);

 

  //double Edev=Estored_tx[systemId] - Estored_sleep[systemId];

/*
  Edev_list[systemId][Edev_index[systemId]] = Estored_tx[systemId] - Estored_sleep[systemId];
  Edev_index[systemId]+=1;
  Edev_index[systemId]%=Edev_list_size;
*/
  //double Edev_max = getMax(Edev_list[systemId],Edev_list_size);
  //double Edev_min = getMin(Edev_list[systemId],Edev_list_size);

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //-------------------------------
  //FIXED EDEV!!! for SF7 and packet size 200B
  //double Edev = 0.0; //getMax(Edev_list[systemId],Edev_list_size);
  //-------------------------------
  
  double alpha = 1; //alpha alto do importanza al presente, alpha basso do importanza al passato
  std::string state_str;



  /*
  Edev Dovrebbe essere la differenza tra l'energia nello stato TX e l'energia nello stato SLEEP. 
  Per rapidità nello studio, si decide di mappare questo valore hardcoded poiché dai dati raccolti ci sono delle cose non molto chiare
  Edev = 0.145705; per SF12, packetSize=50 e dati energetici di default del modello energy-capacitor. Noto Edev si può iniziare a ragionare 
  sull'algoritmo di regolazione del T
  */
  switch (status)
  {
  case EndDeviceLoraPhy::SLEEP: 
    
    state_str="SLEEP"; 

    
    if (!(is_sleep[systemId]) && !(is_turnon[systemId])){
    
      Estored_sleep_max[systemId]=std::max(Estored_sleep_max[systemId],Estored);
  
      //Edev=Estored_tx[systemId] - Estored_sleep[systemId]; // NON TOCCARE QUESTO, SERVE PER LA loss_flag!!!
      //Edev = Estored_tx_max[systemId] - Estored_sleep_max[systemId];

      Estored_sleep[systemId]=Estored;

      
    }
    is_turnon[systemId]=false;
    is_sleep[systemId]=true; //lock sleep check
    
    break;
  case EndDeviceLoraPhy::STANDBY: state_str="STANDBY"; break;
  case EndDeviceLoraPhy::TX: 

    /*
     //FORZO Edev perché NS-3 usa un modello di tipo corrente costante che non fa tornare i conti di energia e Edev stranamente si abbassa quando V si abbassa.
    if ((capacitance >= 40) && (capacitance < 50))
      Edev = 0.01;
    if ((capacitance >= 50) && (capacitance < 60))
      Edev = 0.015;
    if ((capacitance >= 60) && (capacitance < 80))
      Edev = 0.02;
    */
   Edev = 0.026;
   
   /*
    if (capacitance >= 40)
      Edev=0.02;
    if (capacitance >= 50)
      Edev=0.02;
    if (capacitance >= 60)
      Edev=0.02;
    if (capacitance >= 80)
      Edev = 0.026;
    */

    Estored_tx_[systemId]=Estored_tx[systemId];
    Estored_tx[systemId]=Estored;
    DeltaEstored_sleep[systemId]=Estored-Estored_[systemId];

    Estored_tx_max[systemId]=std::max(Estored_tx_max[systemId],Estored);
    state_str="TX";

    is_sleep[systemId]=false; //NOTE: this flag unlock sleep check it is used to compute and update T
    
    Estored_list[systemId][Estored_index[systemId]] = Estored;
    Estored_index[systemId]+=1;
    Estored_index[systemId]%=Estored_list_size;
    //Edev=Estored_tx[systemId] - Estored_sleep[systemId];
    
    //std::cout << "Estored:"<<Estored<<"\tEdev:"<<Edev<<"\tE_th:"<<E_th<<"\tEstored-Edev:"<<Estored-Edev<<"\tloss_flag:"<<loss_flag<<std::endl;
    Estored_EWMA [systemId] =  Estored_EWMA [systemId] * (1-alpha) + Estored_tx[systemId]  * alpha;

    Estored_EWSD [systemId] =  std::sqrt(
      Estored_EWSD [systemId]*(1-alpha) + 
      (Estored - Estored_EWMA [systemId])*(Estored - Estored_EWMA [systemId]) * alpha
      );

    
    if (enable_T_adapt){

      updateReturnValues ret =  T_Update3(
          Estored_tx [systemId],
          //Estored_EWMA [systemId],
          Estored_tx_ [systemId],
          DeltaEstored_sleep[systemId],
          Edev, 
          capacitance, 
          BasicEnergySupplyVoltageV, 
          voltageThLow, 
          periodic_sender->GetInterval().GetSeconds(),
          TX_counter[systemId],
          TX_counter_start[systemId],
          Estored_ref[systemId]
      );
      
      periodic_sender->SetInterval(Seconds(ret.T));
      TX_counter[systemId]=ret.TX_counter;
      TX_counter_start[systemId]=ret.en_TX_counter;
      Estored_ref[systemId]=ret.Estored_ref;
    }

    //E_th = 0.5 * capacitance/1000.0 * voltageThLow * voltageThLow * BasicEnergySupplyVoltageV * BasicEnergySupplyVoltageV;
    //loss_flag=((Estored_tx[systemId]-Edev) <= E_th)?1:0;

    break;
  case EndDeviceLoraPhy::RX: state_str="RX"; break;
  case EndDeviceLoraPhy::OFF: 
    if (state[systemId]=="TX"){
      loss_flag=1;
    }
    state_str="OFF"; 
   
    //std::cout << "E_stored in OFF:" << Estored << std::endl;
    periodic_sender->SetInterval(Seconds(appPeriodSeconds)); 
    break;
  case EndDeviceLoraPhy::TURNON: 
    is_turnon[systemId]=true;
    state_str="TURNON";

    //RESET values
    Estored_EWMA [systemId] = Estored;
    TX_counter_start[systemId] = true;
    TX_counter[systemId] = 0;
    Estored_list[systemId]= resetEstored();
    Estored_index[systemId]=0;
    
    Estored_[systemId] = Estored;
    periodic_sender->SetInterval(Seconds(appPeriodSeconds)); 
    break;

  case EndDeviceLoraPhy::IDLE: state_str="IDLE"; break;
  default: break;
  }
  state[systemId]=state_str;
  //Update Estored per transmission
  Estored_[systemId]=Estored;
  

  // Save log
  std::string SF_str  = (en_adr)?"ADR":("SF"+std::to_string(12-dr_fixed));
  std::string filenameState = output_dir+"/deviceStates-nDevices" + std::to_string(nDevices)+"-GW" + std::to_string(nGateways) + "-" + SF_str +"-C" + std::to_string(int(capacitance)) +"-"+season+".csv";
  const char *c = filenameState.c_str ();

  std::ofstream outputFile;
  
  if (stateChangeCallbackFirstCall){
    // Delete contents of the file as it is opened
    outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
    // Set the initial sleep state
    outputFile << "timestamp" << "," << "DEV" <<","<< "state"<< "," <<"energy"<<"," <<"energy_av"<<","<<"T"<<","<<"LOSS"<<std::endl;
    NS_LOG_DEBUG ("Append initial state inside the callback");
    stateChangeCallbackFirstCall = false;
  } else {
    // Only append to the file
    outputFile.open (c, std::ofstream::out | std::ofstream::app);
  }

  outputFile << Simulator::Now ().GetSeconds () << "," << context  << "," << state_str << "," << Estored<< "," << Estored_EWMA [systemId] << "," << periodic_sender->GetInterval().GetSeconds() << "," << loss_flag <<std::endl;
  outputFile.close ();
}




void
OnTransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  
  Ptr<Node> node = endDevices.Get(systemId);
  /*
  Ptr<MobilityModel> position = node->GetObject<MobilityModel> ();
  Vector m_position = position->GetPosition();
  std::cout << "m_position.x: " << m_position.x <<std::endl;
  std::cout << "m_position.y: " << m_position.y <<std::endl;
  std::cout << "m_position.z: " << m_position.z <<std::endl;
  */
  
  
  //std::cout << "["<< systemId <<"]:" <<"periodic_sender->GetInterval(): " << periodic_sender->GetInterval() << std::endl;

  LoraTag tag;
  packet->PeekPacketTag(tag);
  //std::cout << "TX packet->GetUid():" << packet->GetUid() << std::endl;
  
  Ptr<Packet> packetCopy = packet->Copy();
  LorawanMacHeader mHdr;
  packetCopy->RemoveHeader (mHdr);
  LoraFrameHeader fHdr;
  packetCopy->RemoveHeader (fHdr);
  Time currentTime = Simulator::Now();
  
  
  if (enable_pkt_log){
    //DEBUG show packet details
    
    //std::cout <<  currentTime.GetSeconds() <<","<< std::to_string(tag.GetSpreadingFactor()) <<",TX," << fHdr.GetFCnt() << "," << fHdr.GetAddress().GetNwkAddr()  << std::endl;
    
    /*
    Ptr<Node> object = endDevices.Get(fHdr.GetAddress().GetNwkAddr()-1864);
    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    Vector m_position = position->GetPosition();
    */

    std::stringstream vv; 
    pkt_log_buffer << currentTime.GetSeconds() <<","<< std::to_string(tag.GetSpreadingFactor()) <<",TX," << fHdr.GetFCnt() << "," << fHdr.GetAddress().GetNwkAddr() << ",0"<<"\n";

    
    //std::cout << fHdr.GetAddress().GetNwkAddr() << std::endl;
    //std::stringstream vv;   
    //vv << currentTime.GetSeconds() <<","<< std::to_string(tag.GetSpreadingFactor()) <<",TX," << fHdr.GetFCnt() << "," << fHdr.GetAddress().GetNwkAddr() << ",0"<<"\n";
  

    /*
    if (int(fHdr.GetAddress().GetNwkAddr()) == 1864)
      std::cout << "FIRST DEVICE:" << vv.str() << std::endl;


    if (int(fHdr.GetAddress().GetNwkAddr()) == 1864+nDevices-1)
      std::cout << "LAST DEVICE:" << vv.str() << std::endl;
    */

  }
  packetsSent.at(tag.GetSpreadingFactor()-7)++;
}

void OnPacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  
  NS_LOG_FUNCTION (packet << systemId);
  LoraTag tag;
  packet->PeekPacketTag(tag);
  //std::cout << "RX packet->GetUid():" << packet->GetUid() << std::endl;
  if (enable_pkt_log){
    Ptr<Packet> packetCopy = packet->Copy();
    LorawanMacHeader mHdr;
    packetCopy->RemoveHeader (mHdr);
    LoraFrameHeader fHdr;
    Time currentTime = Simulator::Now();
    packetCopy->RemoveHeader (fHdr);
    pkt_log_buffer << currentTime.GetSeconds() <<","<< std::to_string(tag.GetSpreadingFactor()) <<",RX," << fHdr.GetFCnt() << "," << fHdr.GetAddress().GetNwkAddr() <<  "\n";
    //packetsReceived.at(tag.GetSpreadingFactor()-7)++;
    //std::cout << Simulator::Now() <<" "<< systemId << " RSSI:" << tag.GetReceivePower() << std::endl;
  }
}

void OnPacketReceptionMACCallback (Ptr<Packet const> packet)
{
  
  LoraTag tag;
  packet->PeekPacketTag(tag);
  //std::cout << "RX packet->GetUid():" << packet->GetUid() << std::endl;
  if (enable_pkt_log){
    Ptr<Packet> packetCopy = packet->Copy();
    LorawanMacHeader mHdr;
    packetCopy->RemoveHeader (mHdr);
    LoraFrameHeader fHdr;
    Time currentTime = Simulator::Now();
    packetCopy->RemoveHeader (fHdr);
    pkt_log_buffer << currentTime.GetSeconds() <<","<< std::to_string(tag.GetSpreadingFactor()) <<",RX," << fHdr.GetFCnt() << "," << fHdr.GetAddress().GetNwkAddr() <<  "," << tag.GetReceivePower() << "\n";
    //packetsReceived.at(tag.GetSpreadingFactor()-7)++;
    
  }
}


void
OnEndDeviceTx (std::string context, EndDeviceLoraPhy::State oldstatus,
                        EndDeviceLoraPhy::State status)
{
  if (status == EndDeviceLoraPhy::TX)
    {
      NS_LOG_DEBUG("One transmission");
      const char *c = filenameTx.c_str ();
      std::ofstream outputFile;
      if (TxCallbackFirstCall)
        {
          outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
          TxCallbackFirstCall = false;
        }
      else
        {
          // Only append to the file
          outputFile.open (c, std::ofstream::out | std::ofstream::app);
        }
      outputFile << context << " " << Simulator::Now ().GetSeconds () << std::endl;
      outputFile.close ();
    }
}

  void OnGeneratedPacket (void)
  {
    // NS_LOG_DEBUG("Generated packet at APP level");
    // In the whole network
    generatedPacketsAPP = generatedPacketsAPP + 1;
    // NS_LOG_DEBUG ("Total number of generated APP packet = " << generatedPacketsAPP);
  }





  /**********
   *  MAIN  *
   **********/
int
main (int argc, char *argv[]) {

  ns3::Packet::EnablePrinting();
  ns3::PacketMetadata::Enable ();
  std::string interferenceMatrix = "goursaud";
  std::string nodes = "betweeness";
  fileHelpers.clear();

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("capacitance", "capacitance[mF]", capacitance);
  cmd.AddValue ("packetSize", "Packet Size [B]", packetSize);
  cmd.AddValue ("nGateways", "Number of end devices to include in the simulation", nGateways);
  cmd.AddValue ("simulationTime", "Simulation Time", simulationTime);
  cmd.AddValue ("appPeriodSeconds", "Duty Cycle", appPeriodSeconds);
  cmd.AddValue ("interferenceMatrix", "Interference matrix to use [aloha, goursaud]", interferenceMatrix);
  cmd.AddValue ("runId", "Experiment Run Id", runId);
  cmd.AddValue ("radius", "Radius of the deployment", radius);
  cmd.AddValue ("realisticChannelModel", "Enable Realistic Channel Model", realisticChannelModel);
  cmd.AddValue ("adr", "Enable ADR", en_adr);
  cmd.AddValue ("nodes", "nodes position", nodes);
  cmd.AddValue ("output_dir", "Experiment output directory", output_dir);
  cmd.AddValue ("extract_adr", "Extract ADR allocation (ONLY LogDistance Propagation)", extract_adr);
  cmd.AddValue ("adr_conf", "Use pre-allocated ADR", adr_conf);
  cmd.AddValue ("DR", "fixed DR", dr_fixed);
  cmd.AddValue ("season", "Season", season);
  cmd.AddValue ("confirmed", "Confirmed Frames", is_confirmed);
  cmd.AddValue ("pv_l", "PV len 1", pv_l);
  cmd.AddValue ("pv_h", "PV len 2", pv_h);
  

  

  cmd.Parse (argc, argv);

  if (appPeriodSeconds==-1){
    enable_T_adapt=true;
    appPeriodSeconds=1*60*60;
  }
    
    
  // Set up logging
  LogComponentEnable ("Energy-Harveseting-Capacitor", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraPacketTracker", LOG_LEVEL_ALL);(timestamp, 𝑃ℎ𝑎𝑟 𝑣𝑒𝑠𝑡𝑒𝑟 )
  LogComponentEnable ("CapacitorEnergySource", LOG_LEVEL_ALL);
  // LogComponentEnable ("CapacitorEnergySourceHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraRadioEnergyModel", LOG_LEVEL_ALL);
  // LogComponentEnable ("EnergyAwareSender", LOG_LEVEL_ALL);
  // LogComponentEnable ("EnergyAwareSenderHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("EnergyHarvester", LOG_LEVEL_ALL);
  // LogComponentEnable ("VariableEnergyHarvester", LOG_LEVEL_ALL);
  // LogComponentEnable ("EnergySource", LOG_LEVEL_ALL);
  // LogComponentEnable ("BasicEnergySource", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraChannel", LOG_LEVEL_INFO);
  // LogComponentEnable ("LoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable ("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable ("SimpleEndDeviceLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable ("GatewayLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable ("SimpleGatewayLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("GatewayLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LogicalLoraChannel", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraPhyHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LorawanMacHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("OneShotSenderHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("OneShotSender", LOG_LEVEL_ALL);
  // LogComponentEnable ("PeriodicSender", LOG_LEVEL_ALL);
  // LogComponentEnable ("LorawanMacHeader", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraFrameHeader", LOG_LEVEL_ALL);

  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);



  // Set SF
  Config::SetDefault ("ns3::EndDeviceLorawanMac::DataRate", UintegerValue (dr_fixed));
  // Set MAC behavior
  Config::SetDefault ("ns3::EndDeviceLorawanMac::MacTxIfEnergyOk", BooleanValue (false));


  std::string filenameHarvesterSun = "/energy_csv/solar_power_palermo_"+season+"_"+pv_l+"_"+pv_h+".csv";

  // Select harvester and input file
  bool enableVariableHarvester;
  std::string filenameHarvester;
  if (eh < 0) // variable
    {
      enableVariableHarvester = true;
      filenameHarvester = pathToInputFile + filenameHarvesterSun;
    }

  else // constant harvester
    {
      enableVariableHarvester = false;
    }

/*  
  MpiInterface::Enable(&argc, &argv);


  uint32_t systemId = MpiInterface::GetSystemId();
  uint32_t systemCount = MpiInterface::GetSize();
  // Create a set of nodes
  if (systemCount==1)  {
    endDevices.Create (nDevices);
  }else{ 
    if (systemId==0){ //use the first systemId to create the nodes
      std::cout << "===========================================" << std::endl;
      std::cout << "CREATE MPI nodes in different processors..." << std::endl;
      std::cout << "===========================================" << std::endl;
    }
    for (uint nn=0; nn<nDevices; nn++){
      if (systemId==0) 
        std::cout << "nn:" << nn << "systemCount" << systemCount << "curr n:" << nn % systemCount << std::endl;
      Ptr<Node> n = CreateObject<Node>(nn%systemCount);
      endDevices.Add(n);
    }
    
  }
*/
  endDevices.Create (nDevices);

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (1);

  // Creating a directory
  if (mkdir(output_dir.c_str(), 0777) != -1)
    std::cout << output_dir <<" Directory created" << std::endl;
      
  //else 
  //  std::cerr << "Warning :  " << output_dir << " " << "NOT CREATED" << std::endl;
  /*
  if (!extract_adr){
    if (mkdir((output_dir+"/battery-level").c_str(), 0777) == -1)
      std::cerr << "Warning :  " << output_dir+"/battery-level" << " " << strerror(errno) << std::endl;
    else std::cout << output_dir+"/battery-level"  <<" Directory created" << std::endl;
  }*/
  buffer << "nDevices,appPeriodSeconds,simulationTime,SF,totPacketsSent,totReceivedPackets\n";

  pkt_log_buffer<< "timestamp,SF,FLAG,Fcnt,nwkAddr,RSSI\n";

  if (interferenceMatrix == "aloha")
  {
    LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::ALOHA;
  }
  else if (interferenceMatrix == "goursaud")
  {
    LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::GOURSAUD;
  }

  /***********
   *  Setup  *
   ***********/
  std::vector<int> dr_list;
  if (en_adr){
      std::cout << "ADR!!!!" << std::endl;
      dr_list.push_back(-1); //ADR
  }else{
    std::cout << "NO ADR" << std::endl;
    dr_list.push_back(dr_fixed); //Use only DR=0 (SF12)

  //      //add all DR:
  //      for (int ii=0; ii<6; ii++){
  //        dr_list.push_back(ii);
  //      }

  }

  //int maxDevices=nDevices;
  // int nDevices_list[] = {100};
  std::vector<int> nDevices_list;
  //int nDevices_list[] = {85,65,45,25,5};
  if (nDevices==0){ //NO nDevices specified in args
    nDevices_list.push_back(85);
    nDevices_list.push_back(65);
    nDevices_list.push_back(45);
    nDevices_list.push_back(25);
    nDevices_list.push_back(5);
  }else{
    nDevices_list.push_back(nDevices); //use a specified value of nDevice
  }

  if (custom_node_position){
    //nDevices_list = {80,60,40,20};
    // OPEN NODE POSITION FILE
    std::vector<std::string> row;
    std::string line, word;

    /*
    char cwd[1024];
    getcwd(cwd, sizeof(cwd));
    printf("Current working dir: %s\n", cwd);
      std::system("echo -n '1. Current Directory is '; pwd");
    //system("mkdir temp");
    std::system("pwd");
    */
    //std::cout << "Current working directory: " << tmp << std::endl;

    //std::string centrality_table_file_name = "scratch/der-energy/conf/centrality_table_"+nodes+"-NS3-scalar.csv";
    std::string centrality_table_file_name = "scratch/der-energy/conf/ranking_links_big-NS3-scalar.csv"; //LoRa Network Topology
    std::cout << centrality_table_file_name << std::endl;
    std::fstream file (centrality_table_file_name, std::ios::in);

    if(file.is_open()) {
      while(getline(file, line)) {
        row.clear();
        std::stringstream str(line);
        while(getline(str, word, ';'))
          row.push_back(word);
        dev_pos.push_back(row);
      }
    } else
    {
        std::cout << "Could not open the file centrality_table_\n";
    }
    file.close();

   }

  if (enable_gw_position){
    // OPEN NODE POSITION FILE
      
    std::vector<std::string> row;
    std::string line, word;
    std::fstream file ("scratch/der-energy/conf/gw_position_big-NS3-scalar.txt", std::ios::in);
    if(file.is_open()) {
      while(getline(file, line)) {
        row.clear();
        std::stringstream str(line);
        while(getline(str, word, ';'))
          row.push_back(word);
        gw_pos.push_back(row);
      }
    } else std::cout << "Could not open the file_gw\n";
    file.close();
  }
  //for (int nDevices=1000;nDevices<=maxDevices;nDevices+=100){
  //
  
  for (int DR : dr_list){
      
    std::string DR_str = (DR==-1)?"ADR":std::to_string(DR);

    std::cout << "nDevices=" << nDevices << " DR=" << DR_str << std::endl;
    
    //reset data structure (useful for loop)
    int siz = packetsSent.size();
    //no memory allocating
    packetsSent.resize(0);
    packetsSent.resize(siz, 0);

    siz = packetsReceived.size();
    //no memory allocating
    packetsReceived.resize(0);
    packetsReceived.resize(siz, 0);

    // Mobility
    MobilityHelper mobility;
    if (!custom_node_position){
      mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (radius),"X", DoubleValue (0.0), "Y", DoubleValue (0.0));
    }
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      
    /************************
     *  Create the channel  *
     ************************/

    // Create the lora channel object
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
    loss->SetPathLossExponent (2.9);
    loss->SetReference (1, 14-(-18.5)); //Prx(1m) = -18.5dBm, max distance @SF12 = 12Km

    if (realisticChannelModel) {
      // Create the correlated shadowing component
      Ptr<CorrelatedShadowingPropagationLossModel> shadowing = CreateObject<CorrelatedShadowingPropagationLossModel> ();

      // Aggregate shadowing to the logdistance loss
      loss->SetNext (shadowing);

      // Add the effect to the channel propagation loss
      //Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

      //shadowing->SetNext (buildingLoss);
    }

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

    Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

    /************************
     *  Create the helpers  *
      ************************/

    // Create the LoraPhyHelper
    LoraPhyHelper phyHelper = LoraPhyHelper ();
    phyHelper.SetChannel (channel);

    // Create the LorawanMacHelper
    LorawanMacHelper macHelper = LorawanMacHelper ();
    macHelper.SetRegion (LorawanMacHelper::ALOHA);

    // Create the LoraHelper
    LoraHelper helper = LoraHelper ();
    helper.EnablePacketTracking (); // Output filename

    //Create the NetworkServerHelper
    NetworkServerHelper nsHelper = NetworkServerHelper ();

    //Create the ForwarderHelper
    ForwarderHelper forHelper = ForwarderHelper ();

    /************************
     *  Create End Devices  *
      ************************/
      
      
        
    // Assign a mobility model to each node
    //std::cout << "------------------------------------"<< std::endl;
    //std::cout << "systemId:" << systemId << std::endl;
    //std::cout << "systemCount:" << systemCount << std::endl;
    //std::cout << "------------------------------------"<< std::endl;
    //if (MpiInterface::GetSystemId()==1)
    mobility.Install (endDevices);
    // Make it so that nodes are at a certain height > 0
    for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility->GetPosition ();
      position.z = 1.2;
      mobility->SetPosition (position);
    }

    // SET CUSTOM NODE POSITION
    if (custom_node_position){
      uint index = 1;

      // iterate our nodes and print their position.
      for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End(); ++j) {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
        NS_ASSERT (position != 0);
        //Vector pos = position->GetPosition ();
        Vector m_position = position->GetPosition();
        //Vector m_velocity = position->GetVelocity();

        //          std::cout << "x=" << m_position.x << ", y=" << m_position.y << ", z=" << m_position.z << std::endl;
        //m_position.x = (atof(content[index][6].c_str())-493747);
        //m_position.y = ((atof(content[index][7].c_str())-1376302));

        //          m_position.x = (atof(content[index][2].c_str())-493747);
        //          m_position.y = (atof(content[index][3].c_str())-1376302);
        m_position.x = (atof(dev_pos[index][2].c_str()));
        m_position.y = (atof(dev_pos[index][3].c_str()));

        //std::cout << "x=" << m_position.x << ", y=" << m_position.y << ", z=" << m_position.z << std::endl << std::endl;


        //mob->SetVelocity(m_velocity);
        position->SetPosition(m_position);

        index = index + 1;
        if (index == nDevices+1){
          break;
        }
      }
    }
      
    // Create the LoraNetDevices of the end devices
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

    // Create the LoraNetDevices of the end devices
    macHelper.SetAddressGenerator (addrGen);
    phyHelper.SetDeviceType (LoraPhyHelper::ED);
    macHelper.SetDeviceType (LorawanMacHelper::ED_A);

    NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, endDevices);

    //helper.Install (phyHelper, macHelper, endDevices);

    // Now end devices are connected to the channel

    // Connect trace sources
    for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j) {
        Ptr<Node> node = *j;
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
      }

    /*********************
     *  Create Gateways  *
     *********************/

    // Create the gateway nodes (allocate them uniformely on the disc)
    NodeContainer gateways;
    gateways.Create (nGateways);

    //  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
    //  // Make it so that nodes are at a certain height > 0
    //  allocator->Add (Vector (0.0, 0.0, 15.0));
    //  mobility.SetPositionAllocator (allocator);
    mobility.Install (gateways);



   /**********************
   *  Handle buildings  *
   **********************/

    double xLength = 130;
    double deltaX = 32;
    double yLength = 64;
    double deltaY = 17;
    int gridWidth = 2 * radius / (xLength + deltaX);
    int gridHeight = 2 * radius / (yLength + deltaY);
    if (realisticChannelModel == false){
      gridWidth = 0;
      gridHeight = 0;
    }
    Ptr<GridBuildingAllocator> gridBuildingAllocator;
    gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
    gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (gridWidth));
    gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (xLength));
    gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (yLength));
    gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (deltaX));
    gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (deltaY));
    gridBuildingAllocator->SetAttribute ("Height", DoubleValue (6));
    gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (2));
    gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (4));
    gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (2));
    gridBuildingAllocator->SetAttribute (
        "MinX", DoubleValue (-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
    gridBuildingAllocator->SetAttribute (
        "MinY", DoubleValue (-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
    BuildingContainer bContainer = gridBuildingAllocator->Create (gridWidth * gridHeight);

    BuildingsHelper::Install (endDevices);
    BuildingsHelper::Install (gateways);













    int index = 1;
    
    if (enable_gw_position){
      // iterate our gateways and print their position.
      for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End(); ++j) {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
        NS_ASSERT (position != 0);
        Vector m_position = position->GetPosition();

        m_position.x = (atof(gw_pos[index][1].c_str()));
        m_position.y = (atof(gw_pos[index][2].c_str()));        
        m_position.z = (atof(gw_pos[index][3].c_str()));          

        position->SetPosition(m_position);
        index = index + 1;

      }
    }
    // Create a netdevice for each gateway
    phyHelper.SetDeviceType (LoraPhyHelper::GW);
    macHelper.SetDeviceType (LorawanMacHelper::GW);
    helper.Install (phyHelper, macHelper, gateways);

    NS_LOG_DEBUG ("Completed configuration");

    /*********************************************
     *  Install applications on the end devices  *
     *********************************************/

    Time appStopTime = Seconds (simulationTime);
    




    ApplicationContainer appContainer;

    if (sender == "energyAwareSender") {
      EnergyAwareSenderHelper appHelper;
      double energyTh = capacitance / 1000 * pow (energyAwareSenderVoltageTh, 2) / 2;
      NS_LOG_WARN (voltageThHigh * E);
      appHelper.SetAttribute("MaxDesyncDelay",DoubleValue(eaMaxDesyncDelay));
      appHelper.SetEnergyThreshold (energyTh);
      appHelper.SetMinInterval (Seconds (appPeriodSeconds));
      appHelper.SetPacketSize (packetSize);
      //appHelper.Install (endDevices);
      appContainer = appHelper.Install (endDevices);
    }
    else if (sender == "multipleShots") {
      OneShotSenderHelper appHelper;
      double sendTime = appPeriodSeconds;
      while (sendTime < simulationTime) {
        appHelper.SetSendTime (Seconds (sendTime));
        //appHelper.Install (endDevices);
        appHelper.SetAttribute ("PacketSize", IntegerValue (packetSize));
        sendTime = sendTime + appPeriodSeconds;
        appContainer = appHelper.Install (endDevices);
      }
    }
    else if (sender == "periodicSender") {
        PeriodicSenderHelper appHelper;
        appHelper.SetPeriod (Seconds (appPeriodSeconds));
        appHelper.SetPacketSize (packetSize);
        //appHelper.Install (endDevices);
        appContainer = appHelper.Install (endDevices);
    }




    /*
    PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
    appHelper.SetPeriod (Seconds (appPeriodSeconds));
    appHelper.SetPacketSize (packetSize);
    ApplicationContainer appContainer = appHelper.Install (endDevices);
    */

      


    ///////////////////
    // Packet tracker
    ///////////////////
    LoraPacketTracker &tracker = helper.GetPacketTracker ();


    /****************
    *  Simulation  *
    ****************/

    
    appContainer.Start (Seconds (0));
    appContainer.Stop (appStopTime);

    std::ofstream outputFile;
    // Delete contents of the file as it is opened
    outputFile.open ("durations.txt", std::ofstream::out | std::ofstream::trunc);
    for (uint8_t sf = 7; sf <= 12; sf++) {
      LoraTxParameters txParams;
      txParams.sf = sf;
      txParams.headerDisabled = 0;
      txParams.codingRate = 1;
      txParams.bandwidthHz = 125000;
      txParams.nPreamble = 8;
      txParams.crcEnabled = 1;
      //txParams.lowDataRateOptimizationEnabled =
      //    LoraPhy::GetTSym (txParams) > MilliSeconds (16) ? true : false;
      Ptr<Packet> pkt = Create<Packet> (packetSize);

      LoraFrameHeader frameHdr = LoraFrameHeader ();
      frameHdr.SetAsUplink ();
      frameHdr.SetFPort (1);
      frameHdr.SetAddress (LoraDeviceAddress ());
      frameHdr.SetAdr (0);
      frameHdr.SetAdrAckReq (0);
      frameHdr.SetFCnt (0);
      pkt->AddHeader (frameHdr);

      LorawanMacHeader macHdr = LorawanMacHeader ();
      if (is_confirmed){
        macHdr.SetMType (ns3::lorawan::LorawanMacHeader::CONFIRMED_DATA_UP);
      }else{
        macHdr.SetMType (ns3::lorawan::LorawanMacHeader::UNCONFIRMED_DATA_UP);
      }
      
      macHdr.SetMajor (1);
      pkt->AddHeader (macHdr);

      outputFile << LoraPhy::GetOnAirTime (pkt, txParams).GetMicroSeconds() << " ";
    }
    outputFile.close ();

    /**************************
     *  Create Network Server  *
    ***************************/

    // Create the NS node
    NodeContainer networkServer;
    networkServer.Create (1);

    // Create a NS for the network
    nsHelper.SetEndDevices (endDevices);
    nsHelper.SetGateways (gateways);
    nsHelper.Install (networkServer);


    Ptr<ListPositionAllocator> allocator2 = CreateObject<ListPositionAllocator> ();
    // Make it so that nodes are at a certain height > 0
    allocator2->Add (Vector (4000, -300, 0.0));
    mobility.SetPositionAllocator (allocator2);
    mobility.Install (networkServer);
    //Create a forwarder for each gateway
    forHelper.Install (gateways);

    // Install trace sources
    /*
    for (NodeContainer::Iterator node = gateways.Begin (); node != gateways.End(); node++) {
      (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
          "ReceivedPacket", MakeCallback (OnPacketReceptionCallback));
    }
    */
    for (NodeContainer::Iterator node = gateways.Begin (); node != gateways.End(); node++) {
      (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext (
          "ReceivedPacket", MakeCallback (OnPacketReceptionMACCallback));
    }

    // Install trace sources
    for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++) {
      (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
          "StartSending", MakeCallback (OnTransmissionCallback));
    }



    // ============================
    // SAVE ADR ALLOCATION AND EXIT
    // ============================
    if (extract_adr){
      std::vector<int> sfQuantity (7);
      sfQuantity = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
      std::cout << "SF allocation" << std::endl;
      for (uint32_t i = 0; i < 7; ++i)
      {
        std::cout<<"SF"<<std::to_string(i+7)<<": "<<std::to_string(sfQuantity[i]) <<" | ";
      }
      std::cout << std::endl;
      
      adr_buffer << "x,y,SF\n";
      //std::string output_adr = output_dir+"ADR-"+nodes+".csv";
      std::string output_adr = output_dir+"ADR-GW-"+std::to_string(nGateways)+".csv";
    
      for (NodeContainer::Iterator nn = endDevices.Begin (); nn != endDevices.End(); nn++) {
        Ptr<Node> object = *nn;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
        Vector m_position = position->GetPosition();
        Ptr<ClassAEndDeviceLorawanMac> mac = object->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac()->GetObject<ClassAEndDeviceLorawanMac> ();
        adr_buffer << m_position.x << "," << m_position.y <<","<< 12-int(mac->GetDataRate ()) << "\n";  

        std::cout <<"["<< nn-endDevices.Begin() <<"]" <<m_position.x << "," << m_position.y <<","<< 12-int(mac->GetDataRate ()) << std::endl;
      }
        
      std::cout << "--------------------------------" << std::endl;  
      std::cout << "SAVE ADR ALLOCATION " << output_adr << std::endl;
      std::cout << "--------------------------------" << std::endl;  
      std::ofstream outputFileADR(output_adr);

      if (outputFileADR.is_open()) {
          outputFileADR << adr_buffer.rdbuf();
          outputFileADR.close();
          std::cout << "Content written to file successfully!" << std::endl;
      } else {
          std::cerr << "Unable to open file for writing." << std::endl;
      }
        return 0;
    }

    if (en_adr){
      std::cout << "ENTRA QUAAAAAAAA -------------" <<std::endl;
      if (!adr_conf.empty()){
        // ============================
        // GET ADR ALLOCATION FROM FILE
        // ============================

        std::cout << "PRENDI DAL FILE DI CONFIGURAZIONE!!!" << std::endl;
        std::vector<std::string> row;
        std::string line, word;
        std::string filename = "scratch/der-energy/"+adr_conf;
        std::fstream file (filename, std::ios::in);

        if(file.is_open()) {
          while(getline(file, line)) {
            row.clear();
            std::stringstream str(line);
            while(getline(str, word, ','))
                    row.push_back(word);
            adr_values.push_back(row);
          }
        } else {
          std::cout << "Could not open the file adr_conf\n";
        }
        file.close();
        std::cout << "ESCI" << std::endl;
        //force Spreading factor
        for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++) {
          Ptr<Node> object = *node;
          Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
          Ptr<ClassAEndDeviceLorawanMac> mac = (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac()->GetObject<ClassAEndDeviceLorawanMac> ();

          //Search position in adr_values
          for (int i_adr=0;i_adr<86;i_adr++){
            //  std::cout << i_adr <<"," << adr_values[i_adr][0]<<"," << adr_values[i_adr][1]<<"," << adr_values[i_adr][2]<<"," << std::endl;

            double x = atof(adr_values[i_adr][0].c_str());
            double y = atof(adr_values[i_adr][1].c_str());
            int curr_DR = int(12 - atof(adr_values[i_adr][2].c_str()));
            double EPSILON = 0.0001;
            //std::cout << i_adr <<"," << std::endl;
            //std::cout << i_adr <<"," <<x << ","  << y<< "," << curr_DR << std::endl;
            if ((abs(position->GetPosition().x - x ) < EPSILON) && (abs(position->GetPosition().y - y ) < EPSILON)){
              mac->SetDataRate (curr_DR);  
              //  std::cout <<"["<< i_adr <<"]" <<"x:" << x<<" X:"<< position->GetPosition().x  <<" y: "<< y <<" Y:"<< position->GetPosition().y<< " DR:" << curr_DR << std::endl;
              std::cout << i_adr <<"," <<position->GetPosition().x << ","  << position->GetPosition().y<< "," << curr_DR << std::endl;
              break;
            }
          }
          //exit(1);

        }
      }else{
        // ============================
        //          ASSIGN SF 
        // ============================
        std::vector<int> sfQuantity (7);
        sfQuantity = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
        std::cout << "SF allocation" << std::endl;
        for (uint32_t i = 0; i < 7; ++i) {
          std::cout<<"SF"<<std::to_string(i+7)<<": "<<std::to_string(sfQuantity[i]) <<" | ";
        }
        std::cout << std::endl;
      }
    }else{
      //force Spreading factor
      for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++) {
        Ptr<Node> object = *node;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
        Ptr<ClassAEndDeviceLorawanMac> mac = (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac()->GetObject<ClassAEndDeviceLorawanMac> ();          
        mac->SetDataRate (DR);
      }
    }




    //for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++) {
    //
    //          Ptr<Node> object = *node;
    //          Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    //          //NS_ASSERT (position != 0);
    //          Vector m_position = position->GetPosition();
    //
    //          Ptr<ClassAEndDeviceLorawanMac> mac = (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac()->GetObject<ClassAEndDeviceLorawanMac> ();
    //          buffer << node - endDevices.Begin() << ":"  << m_position.x << ":" << m_position.y << ":" <<unsigned(mac->GetDataRate())<< ",";
    //      }



    //std::string anim_filename="output_files/lora-animation-nDevices"+std::to_string(n_devices)+"-SF"+std::to_string(12-DR) +"N"+std::to_string(n_devices)+".xml" ;
    std::string SF_str_xml  = (DR==-1)?"ADR":("SF"+std::to_string(12-DR));
    std::string anim_filename=output_dir+"/lora-animation-nDevices"+std::to_string(nDevices)+"-GW" + std::to_string(nGateways)+"-"+SF_str_xml + "-C" + std::to_string(int(capacitance))+".xml" ;
    AnimationInterface anim (anim_filename); // Mandatory
    for (uint32_t i = 0; i < endDevices.GetN (); ++i) {
      std::string ED_label = "ED-" + std::to_string(i);
      anim.UpdateNodeDescription (endDevices.Get (i), ED_label); // Optional
      anim.UpdateNodeColor (endDevices.Get (i), 255, 0, 0); // Optional
      anim.UpdateNodeSize (endDevices.Get (i) -> GetId (), 100, 100);
    }
    for (uint32_t i = 0; i < gateways.GetN (); ++i) {
      std::string GW_label = "GW-" + std::to_string(i);
      anim.UpdateNodeDescription (gateways.Get (i), GW_label); // Optional
      anim.UpdateNodeColor (gateways.Get (i), 0, 255, 0); // Optional
      anim.UpdateNodeSize (gateways.Get (i) -> GetId (), 100, 100);
    }

    for (uint32_t i = 0; i < networkServer.GetN (); ++i) {
      std::string GW_label = "NS-" + std::to_string(i);
      anim.UpdateNodeDescription (networkServer.Get (i), GW_label); // Optional
      anim.UpdateNodeColor (networkServer.Get (i), 0, 0, 255); // Optional
      anim.UpdateNodeSize (networkServer.Get (i) -> GetId (), 100, 100);
    }


    anim.EnablePacketMetadata (); // Optional
    anim.EnableWifiMacCounters (Seconds (0), Seconds (10)); //Optional
    anim.EnableWifiPhyCounters (Seconds (0), Seconds (10)); //Optional

    /************************
     * Install Energy Model *
      ************************/


    

    std::string rv;
    if (eh < 0){
      rv = "ns3::UniformRandomVariable[Min=0|Max=" + std::to_string (E) + "]";
      std::cout << rv << std::endl;

    } else {
      rv = "ns3::UniformRandomVariable[Min="+ std::to_string (E) +"|Max=" + std::to_string (E) + "]";
      std::cout << rv << std::endl;
    } 
    rv = "ns3::UniformRandomVariable[Min=0|Max=0]";

    Config::SetDefault ("ns3::CapacitorEnergySource::RandomInitialVoltage", StringValue (rv));

    CapacitorEnergySourceHelper capacitorHelper;
    capacitorHelper.Set ("Capacitance", DoubleValue (capacitance / 1000));
    capacitorHelper.Set ("CapacitorLowVoltageThreshold", DoubleValue (voltageThLow));
    capacitorHelper.Set ("CapacitorHighVoltageThreshold", DoubleValue (voltageThHigh));
    capacitorHelper.Set ("CapacitorMaxSupplyVoltageV", DoubleValue (BasicEnergySupplyVoltageV));
    // Assumption that the C does not reach full capacitance because of some
    // consumption in the OFF state
    // double Ioff = 0.0000055;
    // double RLoff = E/Ioff;
    // double ri = pow(E, 2)/eh;
    // double Req_off = RLoff;
    // double V0 = E;
    // if (eh != 0)
    // {
    //   Req_off = RLoff * ri / (RLoff + ri);
    //   V0 = E * Req_off / ri;
    //   }
    // NS_LOG_DEBUG ("Initial voltage [V]= " << V0 <<
    //               " RLoff " << RLoff <<
    //               " ri " << ri <<
    //               " R_eq_off " << Req_off);


    // Set initial voltage uniformy random in (0, E)
    capacitorHelper.Set ("RandomInitialVoltage", StringValue (rv));



    TimeValue ttt = Minutes(15);


    capacitorHelper.Set ("PeriodicVoltageUpdateInterval", ttt);
    // capacitorHelper.Set ("FilenameVoltageTracking", StringValue (filenameRemainingVoltage));

        
    LoraRadioEnergyModelHelper radioEnergy;
    radioEnergy.Set ("EnterSleepIfDepleted", BooleanValue (EnterSleepIfDepleted));
    radioEnergy.Set ("TurnOnDuration", TimeValue (Seconds (TurnOnDuration)));
    radioEnergy.Set ("TurnOnCurrentA", DoubleValue (TurnOnCurrentA));
    radioEnergy.Set ("TxCurrentA", DoubleValue (TxCurrentA));
    radioEnergy.Set ("IdleCurrentA", DoubleValue (IdleCurrentA));
    radioEnergy.Set ("RxCurrentA", DoubleValue (RxCurrentA));
    radioEnergy.Set ("SleepCurrentA", DoubleValue (SleepCurrentA));
    radioEnergy.Set ("StandbyCurrentA", DoubleValue (StandbyCurrentA));
    radioEnergy.Set ("OffCurrentA", DoubleValue (OffCurrentA));   
    radioEnergy.SetTxCurrentModel ("ns3::ConstantLoraTxCurrentModel", "TxCurrent", DoubleValue (TxCurrentA));   


    // install source on EDs' nodes
    
    // install device model
    //EnergySourceContainer sources;
    
    #define CAPACITOR
    //#define BASIC

    #ifdef CAPACITOR
      EnergySourceContainer sources = capacitorHelper.Install (endDevices);
    #endif

    #ifdef BASIC
      BasicEnergySourceHelper basicSourceHelper;
      basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (BasicEnergySourceInitialEnergyJ)); // Energy in J
      basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (BasicEnergySupplyVoltageV));
      EnergySourceContainer sources = basicSourceHelper.Install (endDevices);  
    #endif  

    DeviceEnergyModelContainer deviceModels = radioEnergy.Install (endDevicesNetDevices, sources);

    if (enableVariableHarvester) {
      // // Variable energy harvesting
      VariableEnergyHarvesterHelper variableEhHelper;
      variableEhHelper.Set ("Filename", StringValue (filenameHarvester));
      variableEhHelper.Set ("PeriodicHarvestedPowerUpdateInterval", ttt);
      variableEhHelper.Install (sources);
      
    }else{
      if (eh!=0){
        // Constant harvesting rate
        double meanPowerDensity = eh;
        double variancePowerDensity = 0.08; 
        //Basic Energy harvesting
        BasicEnergyHarvesterHelper harvesterHelper;
        harvesterHelper.Set ("PeriodicHarvestedPowerUpdateInterval", ttt);
        std::string power = "ns3::NormalRandomVariable[Mean=" +
        std::to_string (meanPowerDensity) +
        "|Variance=" + std::to_string (variancePowerDensity) +
        "|Bound=" + std::to_string(meanPowerDensity)+"]";
        harvesterHelper.Set ("HarvestablePower", StringValue (power));
        harvesterHelper.Install (sources);
      }
    }
   

    /**************
     * Get output *
    **************/


    // run for loop from 0 to vecSize

    for (uint j = 0; j < nDevices; ++j) {
      Ptr<Node> node = endDevices.Get (j);
      Names::Add ("Names/nodeApp"+std::to_string(j), node->GetApplication (0));
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<EnergySource> capacitorES;
      for (uint k = 0; k < nDevices; ++k) {
        uint32_t nodeId = sources.Get (k)->GetNode ()->GetId ();
        if (nodeId == j) {
          capacitorES = sources.Get(k);
        }
      }
      Ptr<EndDeviceLoraPhy> phy = loraNetDevice->GetPhy ()->GetObject<EndDeviceLoraPhy> ();
      Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
      // mac->TraceConnectWithoutContext ("EnoughEnergyToTx",
      //                                      MakeCallback (&CheckEnoughEnergyCallback));
      // std::string name = "Names/nodeEdLoraPhy/"
      //   << std::to_string(endDevices.Get(j)->GetId()) << "/";

      phy->TraceConnect ("EndDeviceState", std::to_string (j), MakeCallback (&OnEndDeviceStateChange));
      phy->TraceConnect ("EndDeviceState", std::to_string (j), MakeCallback (&OnEndDeviceTx));
      capacitorES -> TraceConnect("RemainingVoltage", std::to_string(j), MakeCallback (&OnRemainingVoltageChange));

      ns3::Config::ConnectWithoutContext ("/Names/nodeApp"+std::to_string(j)+"/GeneratedPacket", MakeCallback (&OnGeneratedPacket));
      // NS_LOG_DEBUG ("Tracesources connected for ED " << j);




      // INIT Estored_
      TX_counter_start[j] = true;
      TX_counter[j] = 0;
      state[j]="OFF";
      Estored_[j] = node->GetObject<EnergySourceContainer>()->Get(0)->GetObject<EnergySource>()->GetRemainingEnergy();
      Estored_tx[j] = node->GetObject<EnergySourceContainer>()->Get(0)->GetObject<EnergySource>()->GetRemainingEnergy();
      Estored_tx_[j] = node->GetObject<EnergySourceContainer>()->Get(0)->GetObject<EnergySource>()->GetRemainingEnergy();
      Estored_sleep[j] = node->GetObject<EnergySourceContainer>()->Get(0)->GetObject<EnergySource>()->GetRemainingEnergy();
      T_sleep[j] = 0;
      T_tx[j] = 0;
      is_sleep[j] = true;
    }


    if (enable_energy_log){
      for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
      {
        int curr_i = int(j-endDevices.Begin ());
        FileHelper fileHelper;
        std::string EnergySource_label = "/Names/EnergySource-" + std::to_string(curr_i);
        Names::Add (EnergySource_label, sources.Get (j-endDevices.Begin ()));
        std::string SF_str_1  = (DR==-1)?"ADR":("SF"+std::to_string(12-DR));
        //std::string filename_battery = output_dir+"/battery-level/DEV-"+ std::to_string(curr_i) + "-nDevices" + std::to_string(nDevices)+"-GW" + std::to_string(nGateways) + "-" + SF_str_1 + "-C" + std::to_string(int(capacitance));
        
        //std::cout << filename_battery << std::endl;

        //fileHelper.ConfigureFile (output_dir+"/battery-level/DEV-"+ std::to_string(curr_i) + "-nDevices" + std::to_string(nDevices)+"-GW" + std::to_string(nGateways) + "-" + SF_str_1 + "-C" + std::to_string(int(capacitance)), FileAggregator::SPACE_SEPARATED);


        //fileHelper.WriteProbe ("ns3::DoubleProbe", EnergySource_label+"/RemainingEnergy", "Output");
        //fileHelpers.push_back(fileHelper);

      }

    }


    ////////////////
    // Simulation //
    ////////////////
    Simulator::Stop (appStopTime + Hours (1.1));
    NS_LOG_INFO ("Running simulation...");
    Simulator::Run ();

    Simulator::Destroy ();

    /////////////////////////////
    // Print results to stdout //
    /////////////////////////////
    NS_LOG_INFO ("Computing performance metrics...");
    

    NS_LOG_INFO ("Printing total sent MAC-layer packets and successful MAC-layer packets");
    std::string tracker_output = tracker.CountMacPacketsGlobally (Seconds (0), appStopTime + Hours (1));

    replace(tracker_output.begin(), tracker_output.end(), ' ', ',');
    // nDevices,appPeriodSeconds,simulationTime,SF,totPacketsSent,totReceivedPackets
    std::string SF_str  = (DR==-1)?"ADR":("SF"+std::to_string(12-DR));
    std::cout << nDevices << "," << appPeriodSeconds << "," << simulationTime << ","  << SF_str <<  "," << tracker_output << std::endl;
    buffer <<    nDevices << "," << appPeriodSeconds << "," << simulationTime << ","  << SF_str <<  "," << tracker_output << "\n";
    //get SF for all devices and print


    std::string output_file="";
    if (realisticChannelModel)
      output_file = output_dir+"/results-nDevices"+std::to_string(nDevices)+"-GW" + std::to_string(nGateways) + "-" + SF_str + "-C" + std::to_string(int(capacitance)) + "-" + std::to_string(runId) +"-realch";
    else
      output_file = output_dir+"/results-nDevices"+std::to_string(nDevices)+"-GW" + std::to_string(nGateways) + "-" + SF_str + "-C" + std::to_string(int(capacitance)) + "-" + std::to_string(runId) + "";

    if (enable_pkt_log){
      std::cout << "--------------------------------" << std::endl;  
      std::cout << "SAVE PACKET DETAILS FILE TO " << output_file+"-"+season+"-PKT.csv" << std::endl;
      std::cout << "--------------------------------" << std::endl;  
      std::ofstream outputFilePKT(output_file+"-"+season+"-PKT.csv");

      if (outputFilePKT.is_open()) {
          outputFilePKT << pkt_log_buffer.rdbuf();
          outputFilePKT.close();
          std::cout << "Content written to file successfully!" << std::endl;
      } else {
          std::cerr << "ERROR: Unable to open file for writing." << std::endl;
      }

    }

  } // of DR

  std::cout << "FINISH!!!"<<std::endl;

  //SAVE BUFFER PKT TO FILE
  std::string output_file="";
  std::string SF_str_2  = (dr_list[0]==-1)?"ADR":("SF"+std::to_string(12-dr_list[0]));
  std::string nDevices_str  = (nDevices==0)?"":("nDevices"+std::to_string(nDevices));

  if (realisticChannelModel)
    //output_file = "output_files/results-nDevices"+std::to_string(nDevices)+"-GW" + std::to_string(nGateways) + "-" + std::to_string(runId) + "-realch";
    output_file = output_dir+"/results-"+nDevices_str+"-"+"GW" + std::to_string(nGateways) + "-" +SF_str_2+ "-C" + std::to_string(int(capacitance)) + "-" + std::to_string(runId) + "-"+season+"-realch";
  else
    //output_file = "output_files/results-nDevices"+std::to_string(nDevices)+"-GW" + std::to_string(nGateways) + "-" + std::to_string(runId) + "";
    output_file = output_dir+"/results-"+nDevices_str+"-"+"GW" + std::to_string(nGateways) + "-" +SF_str_2+ "-C" + std::to_string(int(capacitance)) + "-" + std::to_string(runId)+"-"+season;

  std::cout << "--------------------------------" << std::endl;
  std::cout << "SAVE STATS FILE TO " << output_file+".csv" << std::endl;
  std::cout << "--------------------------------" << std::endl;

  std::ofstream outputFile(output_file+".csv");
  if (outputFile.is_open()) {
      outputFile << buffer.rdbuf();
      outputFile.close();
      std::cout << "Content written to file successfully." << std::endl;
  } else {
      std::cerr << "Unable to open file for writing." << std::endl;
  }

  return 0;
}
