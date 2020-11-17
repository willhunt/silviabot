#!/bin/sh

# Compile
arduino-cli compile --fqbn arduino:avr:uno silvia_micro

# Upload
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 silvia_micro --verbose

printf "Done  \n"