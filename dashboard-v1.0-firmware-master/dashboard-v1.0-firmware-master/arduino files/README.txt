Dash Robotics Dashboard v1.0 support
====================================

This contains support for the Dashboard v1.0
http://dashrobotics.com/products/dashboard-rdk

To add support for this board to the Arduino IDE, copy the contents into the 
matching folders within the Arduino directory.

====================================

Detailed description of contents:

	File: drivers/Dashboard_drivers/DashboardV1.inf
	Description: This file provides a Windows driver for the Dashboard
	Notes: This file is not digitally signed. Depending on the version of
		windows you are using, you may need to follow additional steps
		to get the driver to install. 

	File: hardware/arduino/boards.txt
	Description: This file provides configuration information for the 
		Dashboard.
	Notes: You can either replace the existing boards.txt or append the
		the information from the included file to the existing file.
		The former will only allow your copy of Arduino to program
		Dashboards (i.e. you will lose the ability to program other
		arduino devices, such as the Uno). The latter will preserve
		the Arduino IDE's ability to program other Arduino devices.

	File: hardware/arduino/variants/dashboard_v1/pins_arduino.h
	Description: pin definitions for the Dashboard

	File: hardware/arduino/bootloaders/caterina/Caterina-dashboardV1.hex
	Description: bootloader for Dashboard

	Files: libraries/Dashbot
	Description: library for Dashboard functions. Includes firmware that
		shipped with the robot.
	Note: The firmware with which the robot shipped is located at
		Dashbot/examples/DashFirmware_v1_0.ino

====================================

Directions for setup in Windows:

1) Download and install Arduino 1.0.5-r2 (www.arduino.cc)
2) Locate the location of your Arduino installation. In Windows, the default 
	location is C:\Program Files (x86)\Arduino

3) Copy drivers\Dashboard_drivers\DashboardV1.inf from the zip to 
	Arduino\drivers

4) Copy the contents of hardware\arduino\boards.txt from the zip file to the
	bottom of the Arduino\hardware\arduino\boards.txt file. This will 
	allow the board to show up as a Dashboard. You can simply overwrite 
	the boards.txt file in your Arduino directory with the version from
	the zip file will allow you program the Dashboard but no other boards
	(such as the Arduino Uno).

5) Copy hardware\arduino\bootloaders\caterina\Caterina-dashboardV1.hex from 
	the zip file to Arduino\hardware\arduino\bootloaders\caterina

6) Copy the Dashbot directory in libraries\ in the zip file to 
	Arduino\libraries 

7) Copy the dashboard_v1 directory in hardware\arduino\variants\from the
	zip file to Arduino\hardware\arduino\variants\.

8) Connect your Dashboard to your computer. If prompted for a driver, 
	manually direct your computer to 
	Arduino\drivers\Dashboard_drivers\DashboardV1.inf
	
	Note: this driver is not digitally signed. Depending on your operating 
	system, you may need to use additional steps to force your OS to install
	this driver. 

9) Restart Arduino IDE if it is open.

====================================

Directions for setup in Mac OS X:

1) Download and install Arduino 1.0.5-r2 (www.arduino.cc)
2) Locate the location of your Arduino installation. Right click (or 
	control-click) the Arduino application and choose “Show Package 
	Contents” to open the necessary directories in Finder.

3) Copy the contents of hardware/arduino/boards.txt from the zip file to the
	bottom of the Contents/Resources/Java/hardware/arduino/boards.txt file. 
	This will allow the board to show up as a Dashboard. You can simply 
	overwrite the boards.txt file in your Arduino directory with the 
	version from the zip file will allow you program the Dashboard but no 
	other boards (such as the Arduino Uno).

4) Copy hardware/arduino/bootloaders/caterina/Caterina-dashboardV1.hex from 
	the zip file to 
	Contents/Resources/Java/hardware/arduino/bootloaders/caterina

5) Copy the Dashbot directory in libraries/ in the zip file to 
	Contents/Resources/Java/libraries 

6) Copy the dashboard_v1 directory in hardware/arduino/variants/from the
	zip file to Contents/Resources/Java/hardware/arduino/variants/.

7) Restart Arduino IDE if it is open.

====================================

Directions for programming:

1) Open Arduino IDE.
2) Under Tools/Board, select Dash Robotics Dashboard v1.0
3) Under Tools/Serial Port, select your Dashboard. In Windows, it will appear as a COM port. In Mac OS or Linux, the device will appear as /dev/tty.usbmodem****, where each * is a digit that helps identify your particular Dashboard. 
4) Write code.
5) To upload code, go to file/upload.

Example code for Dash can be found by going to file/examples/DashBot/DashFirmware_v1_0
Uploading this code will restore your robot to factory settings

All of Dash's functions can be found in the DashBot library (DashBot.cpp)

====================================

Troubleshooting:

If you have trouble uploading code, try restarting your board using the small
push button. Wait 30 seconds, then re-select the COM port or /dev/tty.usbmodem**** for your device. Re-try uploading.
