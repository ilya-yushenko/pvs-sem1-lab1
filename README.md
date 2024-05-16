# pvs-sem1-lab1

## Task

1. Diodes LD1, LD2, LD3, LD4 should blink with a frequency of 1 Hz and a phase shift of 180° for neighboring diodes. By pressing the B1 button, the phase shift should become 90°, pressing it again returns the previous mode of operation.
2. The blinking frequency can be changed using commands via the UART interface. The command has the following form: “F=x.x” where x is a decimal number (change range from 0.1 Hz to 9.9 Hz). The command must be case-insensitive and return an execution result or an error message to the PC in case of invalid command or data.

## Completed

### 1. Implement LED blinking logic with phase shift

- This commit adds the logic to control the blinking of four LEDs (LD1, LD2, LD3, LD4) with a phase shift between adjacent LEDs. The LEDs will blink at a frequency of 1 Hz, and each LED will be turned on for four timer ticks (4 × 250 ms = 1000 ms).
- The initial phase shift between adjacent LEDs is set to 180 degrees (500 ms). When the button B1 is pressed, the phase shift is toggled between 180 degrees and 90 degrees.
- The code uses the HAL library and interrupt handlers for the timer (TIM3) and button (B1) interrupts. The timer is configured to generate interrupts every 250 ms, and the LED states are updated and written to the respective GPIO pins in the timer interrupt handler.
- The phase shift calculation and LED state updates are performed in the `HAL_TIM_PeriodElapsedCallback` function, while the phase shift toggle  logic is implemented in the `HAL_GPIO_EXTI_Callback` function.

### 2. Implement dynamic frequency change via UART commands

- Added support for changing the blinking frequency at runtime
- The frequency can be set by sending a "F=x.x" command via UART
- The value x.x is a decimal number between 0.1 Hz and 9.9 Hz
- The command is case-insensitive
- Upon successful frequency change, the result is printed to the PC
- Error messages are printed for invalid commands or data
