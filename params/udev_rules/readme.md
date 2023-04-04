To ensure all sensors remain in the same Ports, set udev rules. 

Udev rules are written by the administrator in /etc/udev/rules.d/, and their file name has to end with “.rules.” 

To get sensor info in order to write these rules use: 
udevadm info --attribute-walk --name=/dev/ttyUSB0  (or similar port)
