#!/bin/bash

sudo systemctl stop r200_base.service 
sudo systemctl stop r200_teleop.service 
sudo systemctl disable r200_base.service 
sudo systemctl disable r200_teleop.service

reboot

