# DAQ_System

# Project Showcase Link
https://eecs.engineering.oregonstate.edu/project-showcase/projects/?id=z2oxjPZD8s8oi1AW

# Project Overview
The purpose of the personal data acquisition prototype project is to design a data recorder prototype for a general consumer target market. This prototype was designed to be used with a go-kart. Sensors can be connected to the data recorder in a modular fashion. Sensor modules were developed on custom-designed PCBs that utilize STM32 microcontrollers. Each microcontroller sends data to a database hosted on a Raspberry Pi. The data can then be outputted to the user through an intuitive UI. The firmware for the microcontrollers was developed using the RUST programming language. The database used SQLite. The UI was made using the EGUI framework in RUST. Some notable achievements include the UI and firmware development using Rust, a cutting-edge secure programming language, and the modularity of the system which allows for a variety of sensor configurations based on the user's preferences. Key challenges revolved around the creation of custom STM32-based PCBs, suggesting one key area for future improvement.

## System setup guide

Materials: 
  - Raspberry Pi with monitor, keyboard, and mouse. Make sure Its micro SD card has more than 8GB of space

  - RS485 CAN Hat

  - “Sensor modules” 

.

Steps:

1. Use the RS485 CAN Hat on the Pi. Connect sensor modules.

2. Flash code to the Nucleo dev boards that are in charge of the sensors (files in the nucleo_firmware directory).

3. Set up the CAN Hat on the Pi:

  // Open the Raspberry Pi terminal and run the following commands:
  wget https://github.com/joan2937/lg/archive/master.zip 
  unzip master.zip
  cd lg-master
  sudo make install
  
  sudo apt-get update
  sudo apt-get install python-serial
  sudo pip install python-can 
    //if it gives an error, run: rm /usr/lib/python3.11/EXTERNALLY-MANAGED
  sudo nano /boot/config.txt
  //in the file, Modify/create:
  dtparam=spi=on
  dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000

  sudo reboot  //NOTE: you have to have the CAN hat connected before the Pi reboots
  
  //Finally, test that it knows the CAN is there:
  	dmesg | grep -i '\(can\|spi\)'

4. Copy the daq_gui_1 & logger_code directories to the Pi

5. Run the pi_can.py file in the logger_code directory to start the communication. You should see the data being sent in the terminal if it is working.

6. Go to the daq_gui_1 directory and run the daq_gui code (Use cargo run --release if it doesn’t work the normal way). It will take a long time to run the first time.

7. Done. It should work!
