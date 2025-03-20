#!/bin/bash +x

FILE="file_example_WEBM_1920_3_7MB"
ffmpeg -y -i $FILE.webm -an ${FILE}.mp4