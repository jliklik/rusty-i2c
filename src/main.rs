// src/main.rs
// std and main are not available for bare metal software
#![no_std]
#![no_main]

use cortex_m;
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

    let cp = cortex_m::Peripherals::take().unwrap();
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

    let delay = cp.SYST.delay(&clocks);

    let mut sensor = bme280::Bme280::new(i2c, delay);

    sensor.init(bme280::BME280_RES_CONFIG_WEATHER_MONITORING);
    sensor.read_configs();
    hprintln!("chip_id: {}", sensor.config.chip_id);
    let (temp, pres, humd) = sensor.read_environment();
    hprintln!("temp: {}, pres: {}, humd: {}", temp, pres, humd);

    loop {

    }

}
