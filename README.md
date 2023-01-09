# IOT_humidity_project
## Structure and nodes
- Sensor (DHT11 temperature and humidity)
- Actuator (home grade dehumidifier, with small mods)
- Dashboard (for viewing data history and override and set configurations of the actuator)

![Project overview in a Flowchart](Sensorside/Images/IOT_project_overview.png)

## Links to other documentation and repos
- github dashboard: https://github.com/VerbruggenD/IOT_project_dashboard
- github psoc: https://github.com/VerbruggenD/IOT_humidity_project
- dockerhub dashboard: https://hub.docker.com/repository/docker/dieterverbruggen/iot_dashboard/general
- google drive folder: https://drive.google.com/drive/folders/1XOHH8MmAhiO1N2XTn9_pA3xuGM-f5_lv?usp=sharing

## Future works:
For The low power it is possible to have the sensor powered by a IO-pin of the PSOC and switching the sensor off. Also implementing the standard FREERTOS idle settings makes sure the RTOS goes to sleep and deepsleep after a set amount of time.

For the application obviously it would be nice to have an interface node to set the threshold of the `AUTO`. And further develop the other aspects of home HVAC functionality like heating and cooling. Hereby expanding the capabilities of this smart home application.