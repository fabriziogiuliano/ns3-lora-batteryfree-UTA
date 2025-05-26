# NS-3-lora-batteryfree-UTA

### Installation Instructions

1. **Set Up NS-3 Workspace**  
    ```bash
    mkdir NS3
    cd NS3
    ```

2. **Clone NS-3 Repository**  
    ```bash
    git clone https://github.com/nsnam/ns-3-dev-git.git
    cd ns-3-dev-git/
    git checkout ns-3.29
    ```

3. **Add LoRaWAN Module**  
    ```bash
    git clone https://github.com/signetlabdei/capacitor-ns3.git src/lorawan

    echo " =============================="
    echo " NOTE: To avoid compilation errors, in <ns3>/src/energy/model/energy-source.h, the private variables should be moved to protected."
    echo " =============================="

    ```
4. **Clone WDN-NS-3 Repository and add to scratch sources**
   ```bash
    git clone https://github.com/fabriziogiuliano/ns3-lora-batteryfree-UTA scratch/ns3-lora-batteryfree-UTA/    
    ```
5. **Create and activate Conda Environment for python2.7 to run waf in ns-3.29**  
    ```bash
    conda create -n ns3 python=2.7
    conda activate ns3
    ```
    
6. **BUILD**  
    ```bash
    ./waf configure
    ./waf build
    ```


7. **Apply custom patch for randomized packet sender**  
   ```bash
    cd ../../src/lorawan
    git apply ../../scratch/ns3-lora-batteryfree-UTA/lorawan_rnd_sender.patch
    ``` 


8. **Run experiments (with python3)**  
    ```bash
    #move to NS-3 directory to run "waf"
    ./waf --run ns3-lora-batteryfree-UTA --nGateways=1 --realisticChannelModel=true --appPeriodSeconds=-1 --packetSize=200 --capacitance=40 --simulationTime=1728000 --RngRun=10 --RngSeed=10 --runId=1 --nDevices=1000 --output_dir=EXPERIMENT_200_UNCONFIRMED_T_ADAPT --adr=0 --DR=5 --radius=1000 --season=winter --confirmed=0 --pv_l=75 --pv_h=135
    ```
