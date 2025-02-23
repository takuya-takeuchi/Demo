#!/bin/bash +x

FILE="file_example_WEBM_1920_3_7MB"

# iOS simulator and OSX can not play it if `-profile:v high -pix_fmt yuv420p` is not specified
# ffmpeg -y -i $FILE.webm -an $FILE.mp4
ffmpeg -y -i $FILE.webm -c:v libx264 -profile:v high -pix_fmt yuv420p -r 30 -an ${FILE}.mp4