// src/main.rs
// std and main are not available for bare metal software
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use panic_halt as _;
use stm32f1xx_hal::{
    i2c::{BlockingI2c, Mode},
    pac, 
    prelude::*
}; // STM32F1 hardware abstraction layer crate

mod bme280; 

#[entry]
fn main() -> ! {

    hprintln!("Starting program");

    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut afio = dp.AFIO.constrain();
    let mut gpiob = dp.GPIOB.split();

    let scl = 
        gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = 
        gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Standard { frequency: (300.kHz()) },
        clocks,
        1000,
        10,
        1000,
        1000
    );

    // See the following pages for reference:
    // However, keep in mind that the chip that we actually have is the BMP280, not the BMP180
    // https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
    // https://apollolabsblog.hashnode.dev/stm32f4-embedded-rust-at-the-hal-i2c-temperature-pressure-sensing-with-bmp180
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
    // https://docs.rs/bme280-rs/latest/src/bme280_rs/bme280.rs.html#87-102 

    let mut sensor = bme280::Bme280::new(i2c);

    hprintln!("Read configs: \r");

    // write and then read
    // - tell sensor which register we would like to read from, then receive the value
    sensor.init(bme280::Bme280Resolution::StandardRes, bme280::Bme280Resolution::StandardRes);
    sensor.read_configs();
    hprintln!("chip_id: {}", sensor.config.chip_id);
    // if sensor.config.dig_t1 != sensor.read_dig_t1() {
    //     panic!("I2C reading is incorrect! Check register addresses in library?")
    // }
    hprintln!("temp: {}", sensor.read_temperature());
    hprintln!("pres: {}", sensor.read_pressure());

    loop {

    }

}
