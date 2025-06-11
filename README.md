HELLO! 
The project is for an STM32F411RE. 

I've used these as guide. 
For setting up FreeRTOS, 
  Udemy course. 
For SPI SD Card, 
  https://www.youtube.com/watch?v=acXPelSv2B4
For DS18b20 sensor, 
  https://www.youtube.com/watch?v=09C1dyXvSbg
For pH, and DO, 
  Just used ADC 

Currently, it has a lot of GPIOs, and EA task 
60Mhz due to SPI Timer configuration. 

Structure: 2 tasks, 
**First task**
  -ADC battery check 
  -If battery < (min_Val)
    transmit
  -else 
    if min < battery < lev_2
      set wakeup timer 6 hrs
    .
    .
    .
  proceed to turn on pumps 
  stop
  Temp 
  pH (uncalibrated)
  DO (uncalibrated)

  Transmit 
**2nd Task**
  Update_File
**Back to 1st task**
  dispose and clean

  IDLE SLeep. 
