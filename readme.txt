Steps
1) Get SPI working with sensor using provided HAL rust SPI library
2) Program my own SPI driver using 4 GPIO pins in rust


Notes:
1) Semihosting using SWD protocol allows you to send stdio information to host for debugging on ARM devices. This replaces the JTAG debugger on other embedded devices.
2) STM32F103 chip uses thumbv7m-none-eabi for cross-compilation
- cargo build --target thumbv7m-none-eabi
- cargo run
- openocd -f interface/stlink-v2-1.cfg -f target/stm32f1x.cfg
- then in gdb: monitor arm semihosting enable
- then in gdb: target extended-remote :3333
- 
