#!/bin/sh -e

if [ $# -ne 1 ]; then
  echo "Pass password of running user to execute command as root" 1>&2
  exit 1
fi

echo $1 | sudo apt install -y xrdp

D=/usr/share/ubuntu:/usr/local/share:/usr/share:/var/lib/snapd/desktop
cat <<EOF > ~/.xsessionrc
export GNOME_SHELL_SESSION_MODE=ubuntu
export XDG_CURRENT_DESKTOP=ubuntu:GNOME
export XDG_DATA_DIRS=${D}
export XDG_CONFIG_DIRS=/etc/xdg/xdg-ubuntu:/etc/xdg
EOF