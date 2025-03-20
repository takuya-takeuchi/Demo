#!/bin/bash +x

ffmpeg -y -i ios.mov  -vf "palettegen" palette.png
ffmpeg -y -i ios.mov -i palette.png -filter_complex "[0:v]fps=5,scale=iw/2:ih/2[v];[v][1:v]paletteuse" ios.gif
ffmpeg -y -i android.mp4  -vf "palettegen" palette.png
ffmpeg -y -i android.mp4 -i palette.png -filter_complex "[0:v]fps=5,scale=iw/2:ih/2[v];[v][1:v]paletteuse" android.gif
rm palette.png