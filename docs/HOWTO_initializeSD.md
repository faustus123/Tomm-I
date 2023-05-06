
Some instructions for setting up a new microSD card.

Use Raspberry Pi Imager program and select:

Raspberry Pi OS (other) -> Raspberry Pi OS (64-bit)

You'll need to connect a keyboard, mouse, and monitor to
get the network setup. Plug everything in and fire it up.
Here are some settings to use:

- I flipped the switch to make reduced desktop size so it
  fits better on the TV. The picture is still clipped, but
  it is more usable.

- I still use the old default user=pi pass=raspberry values
  and just ignore the complaints
  
- For the network I use "Hill-Lawrence_EXT".
  NOTE: You will be able to see the IP address on the on-robot
  display screen once the script is set up. However, if you
  need to find it otherwise, Then you will need to go to either
  the TPLink app on my phone or look up the IP address for the
  TPLink device on the computer and point my browser there to
  see the IP address assigned to the raspberry PI.

- After it does the initial updates and reboots, run

  "sudo raspi-config"

  Apply the following settings:

  Display Options -> VNC Resolution -> 1920x1080
  Interface Options -> SSH -> enabled
  Interface Options -> VNC -> enabled
  Interface Options -> I2C -> enabled
  Interface Options -> Serial -> disable login shell over serial
  Interface Options -> Serial -> enable serial port hardware
  Performance Options -> Fan -> Temperature Control -> enable pin 18 at 80 degrees

  NOTE: You must reboot after changing the settings since the /dev/ttyS0
        device won't be there until the serial option was enabled.

- Install various software pieces. Note that the Add/Remove software tool
  is really slow at parsing the package list so avoid it if possible.
  NOTE: I downloaded VScode because it seemed a version was not available
  through apt, but later realized it was.
  NOTE: I don't actually need both code and arduino. Either will work.
  If using code, install the C++ and PlatformIO extensions.
  
  sudo apt install code
  sudo apt install spyder
  sudo apt install arduino
  sudo apt install cmake
  sudo pip install Adafruit_GPIO
  sudo pip install Adafruit_SSD1306
  sudo pip install adafruit-circuitpython-servokit
  sudo pip install pyzmq

- Checkout the custom software:

  git clone https://github.com/faustus123/Tomm-I
  
  
- Add the following crontab:

  @reboot python3 /home/pi/Tomm-I/Tomm-I-v2/robot_statusd/robot_statusd.py


TROUBLESHOOTING:
--------------------
- I ran into a strange issue where the VNC desktop would start up OK but
  the menus would be short and the terminal windows would be very small.
  Rebooting did not seem to help. Logging out and back in did though. I
  was able to do this through the active VNC session.


