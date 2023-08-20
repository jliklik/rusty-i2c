// src/main.rs
// std and main are not available for bare metal software
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use panic_halt as _;
// use cortex_m::asm;
use stm32f1xx_hal::{
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac, 
    prelude::*
}; // STM32F1 hardware abstraction layer crate

// fn echo(serial: &mut Serial<USART1, (Pin<'A',9,Alternate>, Pin<'A',10>)>) {
//     let sent = b'X';
//     let received = block!(serial.rx.read()).unwrap();
//     hprintln!("Received: {}", received);
//     block!(serial.tx.write(sent)).unwrap();  
// }

#[entry]
fn main() -> ! {

    hprintln!("Starting program");

    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut afio = dp.AFIO.constrain();
    
    // Get GPIO
    let mut gpiob = dp.GPIOB.split();

    let scl = 
        gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = 
        gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    let mut i2c = BlockingI2c::i2c1(
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
    // https://docs.rs/bme280-rs/latest/src/bme280_rs/bme280.rs.html#87-102 

    const BME180_CHIP_ID: u8 = 0x60; // The knockoff BME280 has the BME180 id which is 0x60
    const BME180_ADDR: u8 = 0x76;
    // const BMP180_ADDR: u8 = 0x77;
    const REG_ID_ADDR: u8 = 0xD0;
    const REG_RESET_ADDR: u8 = 0xE0; // Writing 0xB6 to this register will power on reset the device
    const REG_STATUS_ADDR: u8 = 0xF3; // status contains two bits which indicates status (image update or measuring statuses)
    const REG_CTRL_MEAS_ADDR: u8 = 0xF4; // Controls oversampling of the temp data and power mode
    const REG_CONFIG_ADDR: u8 = 0xF5; // Sets rate, filter and interface options of the device
    const REG_PRESS_ADDR: u8 = 0xF7; // pressure measurement: _msb, _lsb, _xlsb (depending on resolution)
    const REG_TEMP_ADDR: u8 = 0xFA; // temperature measurement:  _msb, _lsb, _xlsb (depending on resolution)
    const REG_DIG_T1_ADDR: u8 = 0x88; 
    const REG_DIG_T2_ADDR: u8 = 0x8A; 
    const REG_DIG_T3_ADDR: u8 = 0x8C; 

    let mut rx_buffer: [u8; 2] = [0; 2];
    let mut rx_word: i16;

    hprintln!("Read configs: \r");

    // write and then read
    i2c.write_read(BME180_ADDR, &[REG_ID_ADDR], &mut rx_buffer).unwrap();
    if rx_buffer[0] == BME180_CHIP_ID {
        hprintln!("Device ID is {}\r", rx_buffer[0]);
    } else {
        hprintln!("Device ID cannot be detected \r");
    }

    // Read MC
    i2c.write_read(BME180_ADDR, &[REG_PRESS_ADDR], &mut rx_buffer)
    .unwrap();
    rx_word = ((rx_buffer[0] as i16) << 8) | rx_buffer[1] as i16;
    hprintln!("pressure = {} \r", rx_word);
    let adc_press: i16 = rx_word;

    // Read MD
    i2c.write_read(BME180_ADDR, &[REG_TEMP_ADDR], &mut rx_buffer)
    .unwrap();
    rx_word = ((rx_buffer[0] as i16) << 8) | rx_buffer[1] as i16;
    hprintln!("temp = {} \r", rx_word);
    let adc_temp: i16 = rx_word;

    // Convert temp
    //let temp: f32 = bmp280_compensate_t_double(adc_temp, dig_t1, dig_t2, dig_t3);

    loop {

    }

}

fn bmp280_compensate_t_double(adc_temp: i16, dig_t1: i16, dig_t2: i16, dig_t3: i16) -> f32{
    let var1 = (((adc_temp as f32)/16384.0 - ((dig_t1 as f32)/1024.0))) * (dig_t2 as f32);
    let var2 = (((adc_temp as f32)/131072.0 - (dig_t1 as f32)/8192.0) * ((dig_t2 as f32)/131072.0 - (dig_t1 as f32)/8192.0)) * (dig_t3 as f32);
    let t_fine = (var1 + var2) / (5120.0);
    t_fine
}