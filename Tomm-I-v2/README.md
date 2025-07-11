
# Tommi-I-v2


## Fresh SD card install

### Step 1. Copy base system image to SD card
If you do not already have it, download the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) program. Select the device and the default 64bit operating system. Insert a new microSD card (or one you that can be erased) and click `NEXT`.

<img src="../docs/images/rpi_imager01.png" width="400">

A window will pop up giving you the option of customizing the installation by setting a username/password for the default account as well as WiFi setup. Edit these as appropriate so the device can immediately join the network when it first starts. 

<img src="../docs/images/rpi_imager02.png" width="400">

Save any changes and accept that it is OK to erase the SD card.


### Step 2: Boot rpi and configure system

Once the OS is written, eject the SD card and move it to the rpi. Power on the RPi. It will take a few minutes to boot up. You will need to find the IP address assigned to the rpi either by accessing your WiFi router by plugging an HDMI cable into the rpi to see the address when the graphical display comes up. IT will show in the upper right corner temporarily so if you miss it you will need to also plug in a keyboard and mouse to navigate to it.

One you have the IP address, you can ssh to it from a rempote computer. Once logged in, run the rspi-config command to complete the system setup. For example:

~~~bash
ssh pi@192.168.1.4
sudo raspi-config
~~~

From this menu configure the following setting. Note that a reboot will be 
required when finished.

~~~
Interface Options -> VNC enabled -> yes
Interface Options -> SPI -> enable -> yes
Interface Options -> I2C enabled -> yes
Interface Options -> Serial Port -> login shell accessible over serial -> no
Interface Options -> Serial Port -> serial port hardware enabled -> yes
Performance Options -> P3 Fan -> Temp Control -> Yes
Performance Options -> P3 Fan -> GPIO pin -> 12
Performance Options -> P3 Fan -> Temp in Celcius -> 80
~~~

### Step 3: Download/Install software
After reboot you should be able to access the device via VNC using the IP address, user, and password configured for the system. This is recommended since it gives easy access to the arduino GUI interface. 


Open a terminal and install/download the needed software. The first step will upgrade all packages and may take a little while.

~~~bash
# Upgrade OS packages
sudo apt -y upgrade

# Install some system packages including command line arduino
# (install "arduino" to install full GUI too)
sudo apt install -y arduino-builder

# Install platformio
python3 -m venv venv
source venv/bin/activate
pip install platformio
pip install adafruit-circuitpython-ssd1306 pillow
pip install adafruit-circuitpython-servokit
pip install zmq pyyaml numpy
pip install rpi.gpio

# Download the Tomm-I software
git clone https://github.com/faustus123/Tomm-I

# Build and upload the firmware
cd Tomm-I/Tomm-I-v2/Firmware
platformio run --target upload
~~~

### Step 4: Automatically start robot status daemon

The robot status daemon is what continually reads the robot state from
the arduino mega into the raspberry pi and then makes it available to
other programs running on the pi. It is also responsible for updating 
the onboard display on the robot so you can read the IP address, battery
life, etc... without having to connect it to another device.

Have the daemon start automatically upon reboot by placing an `@reboot`
line in the crontab of the default account. An example crontab that does
this is in the robot_statusd directory and can be installed like this:

~~~bash
crontab ~/Tomm-I/Tomm-I-v2/robot_statusd/robot_statusd.crontab
~~~

### Step 5: VNC screen resolution and RobotStatus GUI

The screen resolution for the VNC desktop is kind of hidden. To change it, connect via VNC
then go to the raspberry menu and select: 

Preference -> Screen Configuration

This will bring up a Screen Layout Editor which will show the name of th display. On mine it was "NOOP-1".

Layout -> Screens -> NOOP-1 -> Resolution -> 1920x1080


It is also useful to link the RobotStatus script to the Desktop so that it can be double-clicked to start. Do this with:

~~~bash
ln -s ~/Tomm-I/Tomm-I-v2/Monitor/RobotStatus ~/Desktop/
~~~

Test this by rebooting the rpi and seeing that the display (eventually)
comes up.
