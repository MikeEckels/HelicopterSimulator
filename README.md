# HelicopterSimulator
Simulates a helicopters controls over USB HID protocol, using a Teensy 3.6 and three different MPU6050 accelerometers. This is all implemented within a real helicopter frame.
The HID device is essentially emulating a traditional Joystick controller, enabiling the simulator to be used in games like X-Plane, and Microsoft Flight Simulator. A big
flatscreen TV can be placed infront of the cockpit to show the flight scenery of the game. Enabling FPV enhances the simulation.
 
 - [Pictures](https://github.com/MikeEckels/HelicopterSimulator/tree/master/Photos)
 - [3D Files](https://github.com/MikeEckels/HelicopterSimulator/tree/master/3DFiles)
 
 <p float="left">
  <img src="https://imgur.com/wLzb7uk.jpg" width="500" />
  <img src="https://imgur.com/SMUFr5e.jpg" width="290" />
 </p>
 
 # Electronics Overview
 The heart of the simulator is a Teensy 3.6 microcontroller. This interfaces with three different MPU6050 gyroscop/accelerometers over I2c. Everything was hand soldered
 onto a section of perfboard for simplicity.
  
 # Code Overveiew
 This project has two major functions in order to properly function as a simulator. First, it must read all sensors and process/filter thier readings. Next, the data is
 formatted, and output over USB as an HID compliant device. All sensors are fed through a simple first order Kalman filter to reduce noise.
 
 The simulator can also output debug information over a serial port.
 
 ## Debug Commands
  - FullPrint:True/False
    - Enables or disables printing all debug information
  - RawPrint:True/False
    - Enables or disables printing non-filtered data
  - TimePrint:True/False
    - Enables or disables printing a time stamp
