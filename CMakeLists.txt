# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(BOARD "mcxn947_solar_card/mcxn947/cpu0")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(oresat_mcxn947_solar)

target_sources(app PRIVATE
        src/main.c
        drivers/sensor/ti/tmp101/tmp101.c
    )

# Compile in various demos depending on Kconfig values.
target_sources_ifdef(CONFIG_BLINK       app PRIVATE src/blink.c)
target_sources_ifdef(CONFIG_I2C_SENSORS app PRIVATE src/i2c_sensor.c)
target_sources_ifdef(CONFIG_DAC         app PRIVATE src/dac.c)

