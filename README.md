DJI Osmo Pocket camera tilt control by PWM signal.
--------------------------------------------------

You need DJI OSMO Pocket Controller Wheel, Arduino Pro Mini 8 Mhz 3.3v, precision resistors 511 Omh and 255 Omh

DJI OSMO Pocket Controller Wheel contains an analog resistor in the form of a wheel positioned in the middle position by springs.
When you move the wheel the voltage on the center wire of the resistor changes then the digital part converts it to the respective commands for the camera.

The main idea is to change the control voltage externally by connecting the center wire of the resistor to a kind of DAC, so providing different output voltage we can control the tilt of the camera in the same way as we move the control wheel.

The circuit: arduino-tilt.ewb (EWB512) or arduino-tilt.png

BE CAREFUL! The colors of the wiring inside the Controller Wheel might have opposite meaning (at least that was with my camera) - like red wire is GND, black - VCC! Check it twice before connecting!
THE ARDUINO BOARD SHOULD BE POWERED FROM THE CONTROLLER WHEEL ONLY! Otherwise the voltage readings won't be accurate!

The main problem here is that we can't obtain the current tilt value from the camera, so we need to calculate it precisely in order to model camera movement by the program.
To do so we need a "camera profile" - the dependency between control voltage and speed of the camera movement.
I've already prepared such a profile (see dji-ctrl.xlsx). Hope it will work for every camera and you wouldn't need to do any adjustments of the values.

The rest is up to the code.
The main idea of the algorithm: use timer interrupts to periodically recalculate the camera's tilt position according to the current control voltage.
The code also contains a mavlink part which is only for requesting control parameters (speed and smoothness of the movement) from Ardupilot(Pixhawk) controllers and can be omitted if you don't need this.

You can contact me dr.nick.patrick at gmail com if you have any questions.
