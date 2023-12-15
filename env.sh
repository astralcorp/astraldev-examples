#!/bin/bash
cd ~/catkin_ws && catkin build
sudo env PATH=$PATH:/home/$USER/.nvm/versions/node/v12.14.1/bin /home/$USER/.nvm/versions/node/v12.14.1/lib/node_modules/pm2/bin/pm2 startup systemd -u $USER --hp /home/$USER

