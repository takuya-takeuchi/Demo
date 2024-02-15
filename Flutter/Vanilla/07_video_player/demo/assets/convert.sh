#!/bin/bash +x

ffmpeg -y -i big-buck-bunny_trailer.webm -c:a copy big-buck-bunny_trailer.mp4
ffmpeg -y -i big-buck-bunny_trailer.webm -vf palettegen palette.png
ffmpeg -y -i big-buck-bunny_trailer.webm -i palette.png big-buck-bunny_trailer.gif
ffmpeg -y -i big-buck-bunny_trailer.webm -i palette.png big-buck-bunny_trailer.apng
rm palettegen palette.png