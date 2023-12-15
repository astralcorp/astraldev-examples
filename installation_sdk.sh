#!/bin/bash
### INSTALL AND SELECT PYTHON 3.8
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt install -y python3.8
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1


### MAVSDK PART
sudo apt install -y python3-pip
sudo -H pip3 install --upgrade pip
pip3 install mavsdk
pip3 install --upgrade protobuf 
pip3 install geopy
pip3 install GitPython

### NODEJS PART
sudo apt install -y nodejs
sudo apt-get install -y nodejs-dev node-gyp libssl1.0-dev
sudo apt install -y npm
sudo apt-get install -y curl

if which curl >/dev/null; then
  curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.2/install.sh | bash
else
  echo "Error: curl command not found. Please install curl and try again."
fi

source ~/.nvm/nvm.sh
nvm install 12.14.1
nvm use 12.14.1
nvm alias default 12.14.1
echo $(node -v)
npm install pm2 -g

echo "sudo chmod 777 /dev/ttyACM0" >> ~/.bashrc
echo "sudo chown $USER:$USER /home/$USER/.pm2/rpc.sock /home/$USER/.pm2/pub.sock" >> ~/.bashrc
source ~/.bashrc

###INSTALLING SDK
cd ~/Desktop && git clone https://EminErkol:ATBB9zZ69xb73rfhhhXnVemHdh3V5A3EDD9D@bitbucket.org/astral-us/autonomous_drone.git


###HOTSPOT BUILD. PASSWORD OF THE HOTSPOT:1234567890
nmcli con add type wifi ifname wlan0 con-name astral-drones autoconnect no ssid astral-drones
nmcli con modify astral-drones 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
nmcli con modify astral-drones wifi-sec.key-mgmt wpa-psk
nmcli con modify astral-drones wifi-sec.psk "1234567890"

cd ~/Desktop/autonomous_drone/client_nano/drone/Wifi-connect/server/ && sudo chmod +x network_settings.sh
cd ~/Desktop/autonomous_drone/client_nano/drone/Wifi-connect/server/ && sudo chmod +x activate_hotspot.sh
###REMOVE SUDO PASSWORD
echo "$USER ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers

###DECREASE WAITING FOR NETWORK ON BOOT
sudo mkdir -p /etc/systemd/system/networking.service.d/
sudo bash -c 'echo -e "[Service]\nTimeoutStartSec=20sec" > /etc/systemd/system/networking.service.d/timeout.conf'
sudo systemctl daemon-reload

echo " Please close the current terminal and open new one to run the SDK."










