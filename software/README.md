The software must:

Interface with LMT01 sensors via PIO
Interface with character display via I2C
Interface with SD card over SPI

Always:
Update the screen 10 times per second (if data has changed)

On startup:
Allow the user to configure where values will be sent (Display, SD card, USB UART, or a combination thereof)
Which of the 8 sensors to record/display (bonus points: Determine how many sensors are connected automagically)
Set sampling/recording rates (e.g. 1x, 2x, 4x, 8x per second per channel)
Gracefully interface with an SD card (card detected?, write locked?, formatted?, free space on card? etc.)

During logging:
Achieve at least 8x samples per second per sensor (using the PIO blocks)
Send values to the display, SD card and/or USB Serial as necessary
Error gracefully if SD card fills up, etc.
Option to gracefully end data logging (ensure SD card can be ejected safely etc.)

