#!/bin/bash
cd /CodeDroneDIY/src/
platformio run -t upload --upload-port /dev/ttyACM0 -v
