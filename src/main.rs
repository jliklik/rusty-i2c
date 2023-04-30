// src/main.rs
//
// std and main are not available for bare metal software
#![no_std]
#![no_main]

use cortex_m_rt::entry; // The runtime
use cortex_m_semihosting::hprintln; // semi-hosting allows us to print stm32 info on host computer
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text}, // get submodules Baseline and Text from module text
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{ // STM32F1 hardware abstraction layer crate
    i2c::{BlockingI2c, DutyCycle, Mode}, // These are modules inside the crate
    delay::Delay, 
    pac, // pac stands for peripheral access crate, or "PAC" 
    prelude::*, // a prelude is a collection of names that are automatically brought into scope of
                // every module in a crate
    stm32
}; 

#[allow(unused_imports)]
use panic_halt; // When a panic occurs, stop the microcontroller

#[entry]
fn main() -> ! {
    
    // Say hello world
    hprintln!("Hello world!").unwrap();

    // Get handles to the hardware objects. These functions can only be called once
    // so that the borrowchecker can enisure you don't reconfigure something by accident
    let dp = pac::Peripherals::take().unwrap(); // device peripherals
    let cp = cortex_m::Peripherals::take().unwrap(); // core peripherals

    let mut flash = dp.FLASH.constrain(); // grab pointer to reset clock/control register
    let mut rcc = dp.RCC.constrain(); // grab pointer to RCC

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2); // split GPIO B - this is the STM32 rust crate's
                                                  // way of configuring the GPIO - it assigns one
                                                  // field for each pin. Wow. Convenient.

//    let scl: gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl); // Configure as alternate function for I2C scl
//    let sda: gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl); // Configure as alternate function for I2C sda

    loop{}

}
