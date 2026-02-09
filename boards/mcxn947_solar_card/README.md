# Oresat MCXN Solar Card

This repository contains the board files for the Oresat MCXN947 solar card.

It currently enables flexcomm4 UART for the console, I2C0 with an INA226 and two TMP101NA,
DAC1 to an LT1618, and CAN. GPIOs are used for the heartbeat LED, INA226, two TMP101NA alert lines, and the LT1618_ENABLE line.

Follow the [README installation instructions](https://github.com/plskeggs/oresat-zephyr-common) for installation.
