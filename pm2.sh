#!/bin/bash
pm2 start /home/$USER/Desktop/autonomous_drone/client_nano/drone/Wifi-connect/server/wifi_connection.js --cwd /home/$USER/Desktop/autonomous_drone/client_nano/drone/Wifi-connect/server/
pm2 save
pm2 startup
