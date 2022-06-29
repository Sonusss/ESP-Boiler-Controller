One wire sensors are very sensitive to power supply quality.

There is a security mecanism in the code avoiding taking into account bad reading (-127.0) but of course this affect the control precision quality as values are not updated efficiently.

If you see frequent bad reading in the "sensor" MQTT payload like:
{"sensors":{"T_probe0":"59.3","T_probe1":***"-127.0"***,"T_probe2":"58.6","T_probe3":"21.7","T_probe4":***"-127.0"***}}
then change your power supply.

A very good test is to use a battery, if the problem is out on bettery power than you know your PSU is most probably too noisy.

Personnally my first power supply was too noisy and I spent a lot of time trying to debug my code while the battery test pointed out a PSU problem

With my actual PSU I have zero bad reading over hours...
