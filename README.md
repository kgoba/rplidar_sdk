# SLAMTec's RPLidar SDK

Based on the official SDK Version 1.5.7 (initial commit).

## Building the SDK and examples

Run ```make``` in the ```sdk/``` folder.

## Using raw measurements for debugging

This functionality is an extension over the original SDK. This can be helpful for debugging Lidar problems (range, various material reflection properties, etc), since the motor will not be spinning.

Run ```./output/Linux/Release/raw_test /dev/<serial port>``` from the ```sdk/``` folder (on Windows, change correspondingly).

The program will initialize communication with RPLidar, and continuously measure distance without spinning the motor. Your output should look something like this:
```
RPLIDAR S/N: FDED9AF0C5E29DD2B6E39DF57E3E3116
Version: 1.5.7
Firmware Ver: 1.22
Hardware Rev: 3
RPLidar health status : OK. (errorcode: 0)
waiting for data...
Quality: 12h Angle: 352.95 Dist: 01463.75 
Quality: 12h Angle: 352.95 Dist: 01459.75 
Quality: 12h Angle: 353.02 Dist: 01460.00 
Quality: 16h Angle: 353.00 Dist: 01459.25 
Quality: 12h Angle: 352.98 Dist: 01457.75 
Quality: 12h Angle: 352.95 Dist: 01457.25 
Quality: 12h Angle: 352.97 Dist: 01459.50 
```

The Quality field is a raw 8-bit value returned by the firmware. Due to motor not spinning, it will lack the synchronization bit (bit 0). The reported scan angle can be ignored.
