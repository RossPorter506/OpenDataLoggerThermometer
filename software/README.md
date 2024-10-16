The software must:

Interface with LMT01 sensors via PIO
Interface with character display via I2C
Interface with SD card over SPI

Always:
Update the screen if buttons are pressed, or if new sensor data is read

On startup:
1) Allow the user to configure where values will be sent (Display, SD card, USB UART, or a combination thereof)
1b) If UART selected: Configure baud, parity, stop bits
1c) If SD selected: Gracefully interface with an SD card (card detected?, write locked?, formatted?, free space on card? etc.)
2) Which of the 8 sensors to record/display (bonus points: Determine how many sensors are connected automatically and set missing disconnected channels to 'no')
3) Set sampling/recording rates (e.g. 1-9 samples per second per channel)

During logging:
Achieve at least 8x samples per second per sensor (using the PIO blocks)
Send values to the display, SD card and/or USB Serial as necessary
Error gracefully if SD card fills up, etc.
Option to gracefully end data logging (ensure SD card can be ejected safely etc.)

Pseudocode:

```
array[8] sensor_values
array[1024] sensor_values_buffer // Buffer writes to SD card
ISR(timer) {
    ready_to_read_sensors = true
    // Optionally wake MCU up if in power save
}

button_action = none
ISR(next_button) {
    button_action = next
    // Optionally wake MCU up if in power save
}
ISR(select_button) {
    button_action = select
    // Optionally wake MCU up if in power save
}

main() {
    while configuring { // Keep this loop tight
        if button_action != none {
            update_state(button_action) // Figure out what the screen should look like
            redraw_screen(sensor_values1) // This should only take a few milliseconds(?)
            button_action = none
        }
        // Optionally sleep until buttons or timers wake MCU up
    }
    while logging { // Keep this loop tight
        if ready_to_read_sensors {
            sensor_values = read_sensors() // Reading 8 registers, no time at all
            restart_sensors() // toggle GPIO
            set_timer()
            sensor_values_buffer.append(sensor_values)
            if sensor_values_buffer.full() {
                write_to_sd = true
            }
            ready_to_read_sensors = false
            new_sensor_values = true
        }
        if new_sensor_values or button_action != none {
            update_state(button_action) // Figure out what the screen should look like, should be quick
            redraw_screen(sensor_values1)   // I2c in either normal or fast mode (check adapter). 
                                            //This should only take a few milliseconds(?)
            button_action = none
            new_sensor_values = false
        }
        if write_to_sd {
            // SD card SPI should support up to ~20MHz so this should be quick
            write_buffer_to_sd(sensor_values_buffer)
            empty_buffer(sensor_values_buffer)
            write_to_sd = false
        }
        // Optionally sleep until buttons or timers wake MCU up
    }
}
```

PIO program: See if https://github.com/GitJer/Some_RPI-Pico_stuff/tree/main/count_pulses_with_pause is suitable

My first attempt:
```
.autopull 32
.autopush 32

.wrap_target
MOV Y ! NULL // Set Y to 0xFFFFFFFF. We count downwards since we can only decrement. Could do this outside the PIO with pio_sm_exec if needed

loop:
JMP !OSRE respond // CPU sends us dummy data to FIFO when it wants the count. Check if we've received anything (from autopull).
WAIT 1 PIN 0 // Wait for high on the 0th pin as configured in PINCTRL_IN_BASE
WAIT 0 PIN 0 // Wait for low  on the 0th pin as configured in PINCTRL_IN_BASE
JMP Y-- loop // Decrement Y and loop. We assume Y never reaches zero, i.e. the CPU will send up something before then.

respond:
OUT NULL 32 // Discard output shift register data we just autopulled. Could do this outside the PIO with pio_sm_exec if needed
IN Y 32 // Push pulse count to ISR. Autopush will push to FIFO. CPU must negate to undo the inversion from counting downwards

.wrap
```
