# seismoM5
SeismoM5 is an Earthquake Sensor Implementation on M5StickC, using its own MPU6886 accelerometer. Although MPU6886 in M5StickC is a bit on the noisy side; disabling Gyro and using an accel calibration helped a little. The properties of this project is:

- Calibrate MPU6886 accelerometer and use calibration results as offsets.
- Use DLPF - 5 Hz.
- Use MQTT and send data, only when an earthquake happens.
- Draw X,Y,Z accel results on M5stickC screen as graph, only when an earthquake happens.
- Show PGA always.
- Emergency alerts; Red Led and SPK HAT sound warning during an earthquake.

<p align="center">
<img src="pics/IMG_4012.jpg" width="382" height="200">
</p>
  
## Seismology

I am nowhere near being a Seismologist or I understand anything about. It is just that I live in an overly active Seismic zone. The idea came up for adding an earthquake sensor to the smart home; to shut gas valves off and open some rolling shutters for an escape route during an earthquake.

So i tried some signal algorithms to get the most effective earthquake trigger mechanism possible. Among these, STA/LTA method was very close to get rid of the noise and unwanted man made noises and peaks. However, the test results were not succesfull for me. So i opted out for a simple PGA calculation:

``
pga = sqrt(x_vector_mag * x_vector_mag + y_vector_mag * y_vector_mag + z_vector_mag * z_vector_mag)*scale_factor;
``

The triggering mechanism is, if PGA exceeds limit, that is an earthquake. PGA gives intensity too, but i do not know how reliable that is.

## Usage

This is a Platformio code. First of all, the user needs to fill WiFi and MQTT server details in main.cpp. Other parameters like Seismic, MPU calibration or screen properties can be changed for best fit after some tests and trials.

Button A of M5stickC toggles MQTT server connection. If bottom right corner of the screen shows MQTT, the server is connected.

Button B of M5stickC resets M5StickC for recalibration purposes.

<img src="pics/IMG_3999.jpg" width="245" height="150"> <img src="pics/IMG_4001.jpg" width="254" height="150"> <img src="pics/IMG_4003.jpg" width="250" height="150">


## MQTT

### Availability: m5seismo/status

"**online**" or "**offline**"

### State: m5seismo/state

**INIT_MPU:** Init MPU6886.

**WAIT:** Wait for 10 seconds, to get ready to keep M5StickC steady.

**CALIBRATION:** MPU6886 Accelerator find offsets and send to MPU registers for calibration. Might take up to 1 minute.

**LISTENING:** Everything is ready and listening for earthquakes.

**RESET:** Send a raw "RESET" message from anywhere to reset M5StickC for recalibration purposes.

**UPDATE:** Send a raw "UPDATE" message from anywhere to receive momentary Event message.

### Events: m5seismo/event

**{"x":"-85","y":"-1097","z":"16305","pga":"0.07"}**   <sup>Sample</sup>

Momentary x,y,z parameters and PGA in (g). Only send while an earthquake occurs.

PGA results can be compared with the values in:

[PGA Correlation with the Mercalli scale](https://en.wikipedia.org/wiki/Peak_ground_acceleration#Correlation_with_the_Mercalli_scale)

or

[Japan Meteorological Agency seismic intensity scale](https://en.wikipedia.org/wiki/Japan_Meteorological_Agency_seismic_intensity_scale#Scale_overview)

### Events: m5seismo/change_pga

Displays or changes PGA trigger value. The first time SeismoM5 works after the first upload, the default PGA Trigger value will be 0.025 g. It can be changed with sending a float value to this topic in raw format, like sending a raw MQTT message 0.050 to m5seismo/change_pga topic to make the trigger value 0.05 g. If the PGA Trigger is ever changed, SeismoM5 saves this value permanently and use it from then on even after you reset SeismoM5. On screen top right corner shows the PGA Trigger value.

## OTA Firmware Update

You can connect the IP Address of SeismoM5 for an OTA Firmware Update.

## Mounting

Seismologists mount accelerometer based seismic sensors to the lowest point of the structure, close to the ground as possible. However, since this is an amateur earthquake sensor, i mount it as high as possible in the house, on a wall. You can use the in-built magnets of M5StickC to attach it to a metal surface but as i experienced they are not so strong and may fall off during an earthquake. Double sided adhesive mounting tapes can also work, but make sure the wall paint is a stickable one. All in all, it should be mounted very firm and be careful about the x,y,z axis of the accelerometer; use a carpenters level if required. 

** Due to the screen position, x axis is used as z axis and z axis is used as x axis within the code.
