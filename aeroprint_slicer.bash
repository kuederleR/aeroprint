#!/bin/bash

# Input .stl file path
stl_file = "$1"

# Output G-code file path
gcode_file = "output.gcode"

# Convert .stl to G-code using PrusaSlicer
prusa-slicer --export-gcode "$stl_file" --output "$gcode_file"

# Upload G-code to PrusaLink
curl -X POST -H "Content-Type: application/json" \
    -d "{\"filename\":\"$gcode_file\",\"printer\":\"your_printer_name\"}| \
    "http://192.168.8.161/api/files/local"
