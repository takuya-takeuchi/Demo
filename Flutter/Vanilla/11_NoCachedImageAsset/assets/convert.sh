#!/bin/bash +x

ffmpeg -c:v libvpx-vp9 -i "Animation_1709733079844.webm" -c:v libwebp -y -vf "fps=10, split[a][b]; [a]palettegen[c]; [b][c]paletteuse" "Animation_1709733079844.webp"