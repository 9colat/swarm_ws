#! /bin/bash

#we need to replace any thing inside <oftheseparts>

echo "ACTION==\"add\", ATTRS{idVendor}==\"<yourvendorid>\", ATTRS{idProduct}==\"<yourproductid>\", ENV{XAUTHORITY}=\"/home/<user>/.Xauthority\", ENV{DISPLAY}=\":0\", OWNER=\"<user>\", RUN+=\"/usr/local/bin/usb-<yourdevice>-in_udev\"" > /etc/udev/rules.d/00-usb-<yourdevice>.rule

echo "#!/bin/bash" > /usr/local/bin/usb-<yourdevice>-in_udev
echo "/usr/local/bin/usb-<yourdevice>-in &" > /usr/local/bin/usb-<yourdevice>-in_udev

echo "#!/bin/bash" > /usr/local/bin/usb-<yourdevice>-in
echo "sleep 1" > /usr/local/bin/usb-<yourdevice>-in
echo "#code here" > /usr/local/bin/usb-<yourdevice>-in
