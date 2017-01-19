# iot-Project
A low power, low cost Home Automation project

This code consists the fucntionality to switch-on the LEDs on-board in
accordance with the Light and Temperature sensors. This incorporates DMA
to collect the values from the Temperature sensor. Employs LEUART to
send the values to atmel in predefined time intervals - defined by
LETIMER0 onduty.
The entire code is in low power mode (E3 and E2)

The circular buffer code is deviced to handle the corrupt data - which
occurs during the data sharing between interrupts.
The TSL2561 inclues the library required to configure the light sensor,
the communication protocol employed by the light sensor is I2C

The Atmel application consists of the BLE stack to send the sensor
values recieved from the Lepoard Gecko through LEEUART. Atmel recieves
the values into a dma, thus making the entire design and architecture a
low energy module
