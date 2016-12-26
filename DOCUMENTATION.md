##AT command set
The settling of the board is performed through the USB connector using a serial terminal. Serial configuration is 115200bps, 8-bit of data, no parity and 1 stop byte. Each configuration frame must start with ":" (0x3A) and end with a carriage return (0x0D). They should follow this framing:

```
:<command>,<parameter1>,<parameter2>,<parameter3>,<...><CR>
```
The controller will respond to most of request with "OK" or "ERROR".

###SET
Allows to configure a pwm channel. This command respond with "OK" or "ERROR".

```
:SET,<channel>,<T_thres>,<D_min>,<D_max><CR>
```
* channel : channel identifier from 1 to 4
* T_thres : Threshold temperature in tenth of a celcius degree from 0 to 1000
* D_low : Low state PWM duty cycle from 0 to 100
* D_high : High state PWM duty cycle from 0 to 100

###TRACE
Displays the measured temperature on the terminal in tenth of a degree. The temperature is sent every 500ms until the user send a carriage return (0x0D).

```
:TRACE<CR>
```

###START
Allows the user to define the PWM duty cycle that is applied to every channel at the startup of the board. This command respond with "OK" or "ERROR".

```
:START,<D>,<duration><CR>
```
* D : Startup duty cycle from 0 to 100
* duration : startup duration in ms from 0 to 60000

###HYST
Allows the user to define the hysteresis value for each channel. This command respond with "OK" or "ERROR".

```
:HYST,<channel>,<hys><CR>
```

* channel : channel identifier from 1 to 4
* hys : hysteresis value in tenth of a degree from 5 to 50

###CONFIG
This command can be used to display an array containing the current configuration for all the channels. This command respond with the data followed by "OK" or "ERROR".

```
:CONFIG<CR>
```

Response format :

```
:CONFIG CH<n>: <T_thres>, <D_low>, <D_high>, <Hys>
```

Response example:
```
CONFIG CH1: 250, 0, 100, 10
CONFIG CH2: 250, 0, 100, 10
CONFIG CH3: 250, 0, 100, 10
CONFIG CH4: 250, 0, 100, 10
OK
```