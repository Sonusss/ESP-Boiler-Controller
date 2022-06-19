# ESP-Boiler-Controller

ESP8266 based oil burner controller with MQTT interface
Based on LC_Tech 4 relays board

The goal of this project is to replace old style controller on standard oil burners giving more control flexibility and interfacing with domotic control systems like Home-Assistant over MQTT.

It has 5x DS18B20 based temperature sensors for:
- Boiler temp
- Heating return temp
- Hot water tank temp
- home temp
- outside temp

It has 4 relays for:
- Burner
- Heating pump
- Hot water pump
- auxiliary output (Hot water loop pump in my case)

It has an optional auxiliary input for flame monitoring (if available on the burner)

All settings are made through MQTT and are stored in Flash Memory

Actually it needs an external thermostat contact like a standard oil burner heater (MQTT - not physical)

The final aim is to create a totally self contained controller just giving home temp target.
Having also a dynamic drive on the heating water temp.
