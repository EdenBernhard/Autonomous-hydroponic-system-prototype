# AutomaticHydroponic_Prototype
Code for the automatisation of a hydroponic system prototype written in C/C++ (V 1.0)

The Code allows the surveillance of the pH-Value, the oxygen-Value and the conductivity-Value of the water plus an automatic adaption of the values for the optimal conditions to grow your plants.

Features of the current Version:
  - surveillance of pH, oxygen and conductivity via different sensors 
  - adaption of the pH-Value and conductivity-Value through controlled dosing pumps (ESP32)
  - Representation of the measured data on a local Website on a Raspberry Pi 4 via MQTT and NodeRed

Used Technologie:
  - Rapsberry Pi
  - 2x ESP32
  - pH-Sensor
  - oxygen-Sensor
  - conductivity-Sensor
  - 3x dosing pumps (3D printed)
  - 3x Stepper-Motor

Planned Features: 
  - Camera surveillance
  - adjusting of the wanted pH-, oxygen-, and conductivity-Value dependent on the plants to grow
  - adaption of the oxygen-Value through activating and deactivating the oxygen stone
  - interactive Website 
