# Intro

Acquires acceleration data (x,y,z axis) from MMA8653FC sensor.
Buffers data locally (data frames).
Forwards acquired data to serial-USB per frame.
Inserts tokens into serial byte stream between frames.

# Build
Add this project to the node-apps project under apps directory.

Standard build options apply, check the main [README](../../README.md).
Additionally the device address can be set at compile time, see
[the next chapter](#device_address_/_signature) for details.

# Platforms
Can be built for tsb0 and smnt-mb (both tested). Builds for tsb0
out of the box (ie using a standard node-apps project main branch).

Building for smnt-mb requires adding a submodule to node-apps project:
  1. Add submodule https://github.com/thinnect/smenete-platforms to zoo. 

# LEDs
For smnt-mb platform 
  *  blinking blue LED indicates frame transfer.
  *  green LED indicates errors; off - no errors; on - error has occured
