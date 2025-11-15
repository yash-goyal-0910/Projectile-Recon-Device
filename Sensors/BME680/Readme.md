#### General Description



* BNO055 is a 9-axis absolute orientation sensor with accelerometer, gyroscope, and magnetometer embedded with a sensor fusion algorithm.
* It supports various power modes: Normal, Low Power, and Suspend.
* Communicates over I2C or UART.



#### **Power Requirements**



Operates at supply voltage: 3.3V typical.



##### **Power modes:**

* Normal mode: all sensors active.
* Low Power mode: only accelerometer active, wakes upon motion.
* Suspend mode: all sensors and MCU in sleep mode.



**Power consumption varies with mode, typical current consumption for normal operation is around 12mA.**



### **Operating Modes (Configurable via registers)**

* CONFIGMODE: sensor fusion off for configuration.



* Non-fusion modes: accelerometer only, magnetometer only, gyroscope only, and combinations of these.
  
* Fusion modes: IMU, COMPASS, M4G, NDOFFMCOFF, NDOF providing fused orientation data.



###### **Default operation after power-on is CONFIGMODE.**



### **Output Data**

* Output data formats include calibrated/uncompensated accelerometer, magnetometer, gyroscope data.
* Fusion output includes orientation as Euler angles (roll, pitch, yaw) or quaternions.
* Data output rates can be up to 100Hz in fusion modes.
* Data registers provide 16-bit signed values (LSB \& MSB).



#### **Communication Interface**

* I2C interface: typical uses standard 400kHz mode.
* UART interface available.
* Registers accessible via I2C/SPI, allowing configuration and data reading.



#### **Sensor Configuration**

* Accelerometer ranges: 2g, 4g, 8g, 16g.
* Gyroscope ranges: 125, 250, 500, 1000, 2000 dps.
* Magnetometer output data rate: 2Hz to 30Hz.
* Sensor fusion and calibration algorithms run internally.



#### **Voltage and Logic Levels**

* Power supply: 3.3V (voltage regulator needed if using 5V MCU like Atmega328P).
* I2C levels: 3.3V logic; level shifting required for direct interfacing with 5V logic Atmega328P.



#### **Interrupts and Pins**

* INT pin signals data ready and interrupt events.
* Supports motion interrupts, no-motion and high-g detection.
* INT pin logic level is 3.3V.



#### **Calibration Data**

* Sensor offset calibration can be done via writing calibration values to registers.
* Calibration status available via registers.



#### **Key Registers and Configuration**

* Power mode register (PWRMODE)
* Operation mode register (OPRMODE)
* Unit selection register (UNITSEL)
* Data output registers for accelerometer, magnetometer, gyroscope, Euler angles, quaternions.



#### **Practical Notes for Using with Atmega328P-PU**

* Use 3.3V supply for BNO055 or level shifters for I2C lines if Atmega328P runs at 5V.
* Implement I2C communication to read/write BNO055 registers.
* Configure power and operation modes by writing to respective registers.
* Read sensor fusion outputs such as Euler angles or quaternions for orientation.
* Use INT pin for efficient data readiness signaling.
* Manage sensor calibration through registers or external software commands.
* Pay attention to default sensor ranges and output rates for application requirements.



These points summarize the essential features, power requirements, output types, and interface considerations for integrating BNO055 with Atmega328P.











