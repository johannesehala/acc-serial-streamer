# RadioThroughputTest

Sends lots of bytes over radio.
Receives said bytes over radio.
Checks for lost messages on receiver side.
Forwards received bytes to serial-USB.
Inserts tokens into serial byte stream.

# Build
Add this project to the node-apps project under apps directory.

Standard build options apply, check the main [README](../../README.md).
Additionally the device address can be set at compile time, see
[the next chapter](#device_address_/_signature) for details.

# Platforms
Can be built for tsb0 and smnt-mb (both tested). Builds for tsb0
out of the box (ie using a standard node-apps project main branch).

# TODO tsb0 can't build if thinnect.smenete-platforms is not added to zoo

Building for smnt-mb requires three changes to node-apps project:
  1. Add submodule https://github.com/thinnect/smenete-platforms to zoo. 
  2. submodule zoo/thinnect.node-platform HEAD must be moved to git commit 2aa4ab7
  3. submodule zoo/thinnect.dev-platforms HEAD must be moved to git commit 7308efd.

# Device address / signature

Devices are expected to carry a device signature using the format
specified in https://github.com/thinnect/euisiggen and
https://github.com/thinnect/device-signature. The device then derives a short
network address using the lowest 2 bytes of the EUI64 contained in the
signature.

If the device does not have a signature, then the application will
initialize the radio with the value defined with DEFAULT_AM_ADDR. For example
to set the address to 0xABCD in the firmware for a Thunderboard Sense 2, make
can be called `make thunderboard2 DEFAULT_AM_ADDR=0xABCD`. It is necessary to
call `make clean` manually when changing the `DEFAULT_AM_ADDR` value as the
buildsystem is unable to recognize changes of environment variables.
