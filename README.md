# stm32-weather-sensor
Outdoor air sensor using STM32WB55 and Home Assistant Zigbee integration

## Overview
The main purpose of this project is to experiment with low power modes of stm32 microcontroller while building a Zigbee sleepy end-device that can work off the grid on solar power.

## Hardware
The project is again based on [WeAct STM32WB55-Core board](https://github.com/WeActStudio/WeActStudio.STM32WB55CoreBoard "Github link: STM32WB55 Core Board").
This is a compact board that still has all of the bare necessities:
- LDO regulator, according to schematics this is ME6231A33M3G, 3-18V input, 500mA output;
- HSE oscillator 32MHz;
- LSE crystal 32.678kHz;
- Reverse voltage protection MOSFet;
- GPIO controlled (PE4) blue LED;
- PCB antenna.

As for the sensor, I have considered BME280 or BMP280 coupled with AHT20. Both sensors can purchased from Aliexpress already soldered on breakout board. The difference between the two sensors is minimal,  but in the batch I got BMP280+AHT20 seemed to produce sligtly more accurate temperature and humidity measurements, so it went into the final design.

Along with sensors I have picked on Aliexpress a small outdoor light, solar powered and with replaceable battery. It serves both as the power source and enclosure, along with its normal function as an outdoor light. There is enough room inside its casing to host the controller board. The sensor must be placed outside the case though, since the light may get very warm under the sun.

To monitor battery charge level I have added a simple voltage divider. It is easy to implement, but divider's downside is that it continuously discharges battery in the background. I used 2 fairly high value resistors, 1M and 330K to keep the discharge current small. Experimenting with ADC I have found that it takes ~50ms of sampling time to accurately measure battery voltage. For this project this is acceptable.


## Zigbee setup
Since this sensor is battery powered it should be configured as a sleepy end device. The firmware implements 1 endpoint and 4 measurement clusters:

- Temperature
- Humidity
- Pressure
- Voltage

Battery voltage is reported using AC voltage attributes, since ZHA does not support DC voltage measurements.

Pairing STM32WB55 with ZHA is somewhat tricky. STM's implementation of SED involves aggressive power optimizations. As soon as SED joins Zigbee network its firmware switches to low-power mode with 30 second poll intervals by default. However, ZHA expects to receive device's configuration, such list device's clusters and attributes and their values and eventually times out with incomplete device setup in Home Assistant.

One way to fix this behavior that I have found to work most of the time (still not every time, unfortunately) is to temporarily turn on fast-polling mode for a few minutes immediately after joining the network for the first time.

There are few other small compatibility issues that are better taken care of by the ZHA quirk file.

Information about the connection is stored in persistence storage, so next time the controller is restared network re-joining happens automatically.

