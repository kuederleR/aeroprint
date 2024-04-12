#!/bin/bash

# Remove voxl aeroprint service and copy it from this directory.
cp ~/aeroprint/starling_setup/voxl-aeroprint-start.service /etc/systemd/system/voxl-aeroprint-start.service
cp ~/aeroprint/starling_setup/startup_commands.bash /etc/aeroprint/startup.bash
# Set appropriate services
sudo systemctl enable voxl-aeroprint-start

if [ "$STARLING_MODE" ==  "SIM" ]
then
  sudo systemctl disable voxl-px4;
  sudo systemctl disable voxl-qvio-server;
fi
if [ "$STARLING_MODE" == "FLIGHT" ]
then
  sudo systemctl enable voxl-px4;
  sudo systemctl enable voxl-qvio-server;
fi