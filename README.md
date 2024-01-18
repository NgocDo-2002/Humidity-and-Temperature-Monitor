# Outdoor Humidity and Temperature Monitor

The project was made with the purpose of learning and researching Embedded Systems Design. The scope of this project is to design and develop a temperature and humidity measurement system using DHT22 sensor, microcontroller STM32 L152-RE board and use MODBUS to establish master and slave communication. A simple circuit made to connect microcontroller, sensor together.

![image](https://github.com/NgocDo-2002/Humidity-and-Temperature-Monitor/assets/84715183/2fa01247-abcd-4627-bc99-6fcb32d8843a)

## Operating specifications

The communication between sensor and microcontroller is separated into 3 steps: When MCU send start signal, AM2302 change from standby-status to running-status. When MCU finishs sending the start signal, AM2302 will send response signal of 40-bit data that reflect the relative humidity and temperature to MCU.

Step 1: MCU send out start signal to AM2302 and AM2302 send response signal to MCU  
   MCU pull down in at least 1ms for start signal  
   MCU pull up wait 20-40us for the sensor's response  

Step 2: DHT22 send response signal to MCU  
   Sensor pull down in 80us then pull up in 80us to notify the host that the sensor is ready to send the data  
   If no response is received, it is possible that the sensor doesn't work properly  

Step 3: DHT22 send data to MCU by bits "0" or "1"  
   Every bit data is followed by 50us of low-voltage-level.  
   Then the sensor will pull up for a certain amount of time, which decides the bit is 0 or 1:  
     26~28us corresponding to bit 0.  
     70us corresponding to bit 1.  
