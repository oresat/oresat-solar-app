# oresat-solar-app

OreSat Zephyr app for the solar card.


# Building and flashing
Ensure you are in the `solar` directory (`cd src/oresat/firmware/apps/solar`) prior to building.

> NOTE:
>   The `mcxn947_solar_card` is the default. It normally does not need to be specified as shown below,
>   except for the important exception of enabling MCUboot. In that case, it is mandatory for Zephyr 4.2.0.

| Board         | Build Example                                         |
| ------------- | ----------------------------------------------------- |
| mcxn947_solar_card | `west build -p always -b mcxn947_solar_card/mcxn947/cpu0 -DCONFIG_MCUBOOT_ALLOWED=n` |
| mcxn947_solar_card with shell | `west build -p always -b mcxn947_solar_card/mcxn947/cpu0 -- -DEXTRA_CONF_FILE=overlay_shell.conf -DCONFIG_MCUBOOT_ALLOWED=n` |
| mcxn947_solar_card with MCUboot | `west build -p always -b mcxn947_solar_card/mcxn947/cpu0 --sysbuild -- -DBOARD_ROOT=$PWD` |

> NOTE: the section below only gives general instructions. Specific steps below (like for setting the CAN node id) are self-contained
>   in the section.

# Building and flashing without the bootloader
This runs the build and the flash faster when working in the lab.
However, this will of course not be able to be remotely updated over
CAN.

## First build

For the first build, or when previously running with the bootloader:
```
$ west build -p
$ west flash --erase -r pyocd
```

> NOTE: the `--erase` option will lose the settings data that stores a
>  non-default CAN node id.

## Subsequent builds:
```
$ west build
$ west flash -r pyocd

```

# Setting the CAN node id
This can be done through a terminal with the Zephyr shell enabled.

This requires three steps in practice:
1. Build and flash with the `overlay_shell.conf` configuration file applied
1. Enter the CAN node id in the terminal
1. Build and flash again without the `overlay_shell.conf` without erasing the chip

## Enabling the shell
Do a build with the `overlay_shell.conf` file as a parameter to `west build`:
```
$ west build -p always -b mcxn947_solar_card/mcxn947/cpu0 -- -DEXTRA_CONF_FILE=overlay_shell.conf -DCONFIG_MCUBOOT_ALLOWED=n
```
Flash as explained earlier.

## Using the shell
The terminal will continue to show whatever log messages are currently enabled.
But it will also show, at the bottom of the scroll, a prompt:
```
uart:~$ nodeid
```
Enter `help` for help.
Enter `nodeid`<enter> to see the current nodeid.
Change the node id by entering `nodeid <N>` where N is the desired node id in decimal.

## Rebuilding to remove shell
Build again, using whatever options you need as explained in the document.

> NOTE: do not use the `--erase` parameter with `west flash` or you will lose the change you
>   just made with the shell.

# Building and flashing with bootloader
```
$ west build -p -b mcxn947_solar_card/mcxn947/cpu0 --sysbuild -- -DBOARD_ROOT=$PWD
$ west flash --erase -r pyocd
```
This will build two binaries: one for the bootloader, and one for the application.
The flashing step will be done automatically in two parts to flash these two binaries.

> NOTE: The app flashing step will run slowly: this is normal.

## CAN flashing script
The `flash_canopen.py` script can be used to test flashing over CAN on the desktop.

```
	 $python3 flash_canopen.py --help
usage: flash_canopen.py [-h] [--serial-port SERIAL_PORT] [--channel CHANNEL] [--bitrate BITRATE] [--node-id NODE_ID] [--bin BIN_PATH] [--block-transfer] [--download-buffer-size DOWNLOAD_BUFFER_SIZE]
						 [--status-timeout STATUS_TIMEOUT] [--bootup-timeout BOOTUP_TIMEOUT] [--sdo-timeout SDO_TIMEOUT] [--sdo-retries SDO_RETRIES] [--confirm] [--no-confirm] [--request-crc]
						 [--throttle-delay THROTTLE_DELAY] [--debug]

options:
  -h, --help            show this help message and exit
  --serial-port SERIAL_PORT
  --channel CHANNEL
  --bitrate BITRATE
  --node-id NODE_ID
  --bin BIN_PATH
  --block-transfer
  --download-buffer-size DOWNLOAD_BUFFER_SIZE
  --status-timeout STATUS_TIMEOUT
  --bootup-timeout BOOTUP_TIMEOUT
  --sdo-timeout SDO_TIMEOUT
  --sdo-retries SDO_RETRIES
  --confirm
  --no-confirm
  --request-crc
  --throttle-delay THROTTLE_DELAY
  --debug               Enable verbose CAN logging
```

This example flashes the mag app firmware to a CANable adapter on ttyACM1, where the mag card is running
the default CAN id of 124.
```
$ python3 flash_canopen.py --serial-port /dev/ttyACM1 --bin  build/mag/zephyr/zephyr.signed.bin --node-id 124
```
> NOTE: The CAN id must be changed to the proper one for this card before the
>   C3 can access it.
