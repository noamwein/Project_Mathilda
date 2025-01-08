# Configuring RPI

## Setup VNC Server

Install the VNC server on rpi:

``
sudo apt update
sudo apt install realvnc-vnc-server realvnc-vnc-viewer
``

Enable VNC:

``
sudo raspi-config
``

Reboot to apply changes:

``
sudo reboot
``