# Device Side
## Introduction
As an actuator we chose to use a dehumidifier, this device is internally controlled by a relay that is connected to the switch of the watercontainer so that it automatically switches off when the container is full or when the container is not present. We added a mosfet in series with the switch so that the switch can override te mosfet. The mosfet is directly controlled by a data pin on the PSOC6 microcontroller.

## Project Flowchart
![Project overview in a Flowchart](/IOT_project_overview.png)

## MQTT and different topics
We used MQTT to communicate between our devices, in total we used 4 different topics to post different information:
1. sensor/temperature:

    >This is the topic where the temprerature values from the sensor are posted.

2. sensor/humidity:

    >This is the topic where the humidity values from the sensor are posted and the dehumidifier is subscribed to this topic to get the current humidity.

3. sensor/config:

    >This is the topic where the treshold can be updated to set the autmatic function of the dehumidifier.

4. sensor/control:

    >This is the override topic where the dehumidifier is subscribed to to get the on/off/auto command from the dashboard.

When the dehumidifier is first booted up, the treshold value is set to 50%, this means that when the humidity rises above 50%, the dehumidifier will be turned on. When the config topic is used to set a new value by just posting a raw number between 0 and 100, the dehumidifier automatically updates the device state to use the new setting. There is also the possibility to override the automatic program and manually control the dehumidifier by posting `TURN ON` or `TURN OFF` on the control topic. The dehumidifier will then not work automatically anymore untill there is a new post on the control topic saying `AUTO` after which the dehumidifier goes directly into the state corresponding to the most recent information.