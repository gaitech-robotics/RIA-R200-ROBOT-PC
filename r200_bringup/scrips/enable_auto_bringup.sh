#!/bin/bash

sudo systemctl enable r200_base.service 
sudo systemctl enable r200_teleop.service
sudo systemctl start r200_base.service
sudo systemctl start r200_teleop.service

reboot
