You need to add a 4K7 resistor accross 3.3V and GPIO 2 for the DS18B20 sensors

![Relay_board_modification](https://github.com/Sonusss/ESP-Boiler-Controller/blob/main/hardware/LC-Tech_mod.jpg)

If you would like to use the << Burner Flame >> input you need to modify the ESP-01 board to replace RST pin with GPIO 4.

![ESP-01_modification](https://github.com/Sonusss/ESP-Boiler-Controller/blob/main/hardware/ESP-01_mod.jpg)

A bit tricky but... 
(You can see that my ESP8266 had some damage but still working)

Relays are used in the following order:
Relay 1:  Auxiliary contact (not part of any rules, must be directly driven from MQTT)
Relay 2:  Oil burner
Relay 3:  Heating circulating pump
Relay 4:  Hot water circulating pump
