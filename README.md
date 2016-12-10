# fanctrl

Software used on a STM32L031K6 microcontroller to monitor temperature and drive four fans with PWM signals. The digital regulator implemented is hysteretic. The system is configured through serial link and for each fan it is possible to set a trigger temperature at which the PWM ratio driving the fan will change. Four LEDs indicate the status of each fan.