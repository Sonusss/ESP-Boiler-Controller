MQTT Topics:
oil_burner/state
oil_burner/cmd


Parameters:

  outlet_probe_idx = 0;
  return_probe_idx = 1;
  boiler_probe_idx = 2;
  inside_probe_idx = 3;
  outside_probe_idx = 4;
  heating = 0;                //Heating mode on/off  (0/1)
  heating_preset_temp = 70;   //Normal heating water temp 
  heating_high_limit = 90;    //Safety upper limit
  heating_low_limit = 50;     //Temperature when firing the flame
  heating_hysteresis = 2;     //not used (future use)
  heating_temp_factor = 100;  //not used (future use)
  heating_temp_bias = 10;     //not used (future use)
  adaptive_heating = 0;       //not used (future use)
  heating_target_temp = 20.5; //not used (future use)
  burner_min_time = 3;        //not used (minimal flame time to avoid scattering)
  boiler = 0;                 //Hot water generator on/off (0/1)
  boiler_priority = 1;        //Priority mode over heating
  boiler_preset_temp = 60;    //Target temp when generation hot water
  boiler_heating_temp = 75;   //Target temp of circulating water (minimum 5 degrees over boiler_preset_temp)
  boiler_high_limit = 90;     //Safety upper limit
  boiler_low_limit = 40;      //hot water generation is triggered when going below this temperature
  boiler_temp_factor = 100;   //not used
  boiler_temp_bias = 5;       //Circulating water should be warmer of x degrees in order to turn circulating pump on (efficiency security) 

  There is a permanent safety rule for the boiler temp.
  If the temperature goes over boiler_high_limit or heating_high_limit
  burner will be turned off and it triggers an MQTT Alarm message
  oil_burner/state {"event":{"alarm":"overtemp"}}
  However, I hardly suggest not bypassing the genuine thermal security of your boiler
  

Any of the above parameter can be settled with an MQTT message like:
Topic  oil_burner/cmd   Payload '{"heating_preset_temp": 63}'

Available commands:

oil/burner   {"save":1}       Save settings to flash
oil/burner   {"wipe":1}       Erase settings in flash usefull after upgrade
oil/burner   {"init":1}       Restore default settings - does not save to flash 
oil/burner   {"reboot":1}  
oil/burner   {"settings":1}   call MQTT settings message
oil/burner   {"probe":1}   call MQTT sensors message


  
  

  
  
