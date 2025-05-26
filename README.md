# NS-3-lora-batteryfree-UTA

### Installation Instructions

2. **Set Up NS-3 Workspace**  
    ```bash
    mkdir NS3
    cd NS3
    ```

3. **Clone NS-3 Repository**  
    ```bash
    git clone https://github.com/nsnam/ns-3-dev-git.git
    cd ns-3-dev-git/
    git checkout ns-3.29
    ```

4. **Add LoRaWAN Module**  
    ```bash
    git clone https://github.com/signetlabdei/capacitor-ns3.git src/lorawan

    echo " =============================="
    echo " NOTE: To avoid compilation errors, in <ns3>/src/energy/model/energy-source.h, the private variables should be moved to protected."
    echo " =============================="

    ```
5. **Clone WDN-NS-3 Repository and add to scratch sources**
   ```bash
    git clone https://github.com/fabriziogiuliano/ns3-lora-batteryfree-UTA scratch/wdn-ns-3/    
    ```
    

7. **BUILD**  
    ```bash
    cd ../../
    ./waf configure
    ./waf build
    ```


6. **Create and activate Conda Environment for python2.7 to run waf in ns-3.29**  
    ```bash
    conda create -n ns3 python=2.7
    conda activate ns3
    ```


7. **Apply custom patch for randomized packet sender**  
   ```bash
    cd ../../src/lorawan
    git apply ../../scratch/ns3-lora-batteryfree-UTA/lorawan_rnd_sender.patch
    ``` 


8. **Run experiments (with python3)**  
    ```bash
    TBD...
    ```
