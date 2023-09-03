use cortex_m_semihosting::hprintln;
use embedded_hal::blocking::i2c::{Write, Read, WriteRead};
use core::fmt::Debug;

/* CHIP CONSTANTS */
pub const BME280_CHIP_ID: u8 = 0x60; // BME180 ID 0x60 - temp, press and humidity sensor
pub const _BMP280_CHIP_ID: u8 = 0x58; // BMP180 ID 0x60 - only temp and press sensor
pub const RESET_VAL: u8 = 0xB6;

/* ADDRESSES */
pub const BME280_ADDR: u8 = 0x76; // I2C address
pub const REG_ID_ADDR: u8 = 0xD0; // chip ID
pub const REG_RESET_ADDR: u8 = 0xE0; 
// Writing 0xB6 to REG_RESET_ADDR will power on reset the device
pub const _REG_STATUS_ADDR: u8 = 0xF3; 
// _REG_STATUS_ADDR contains two bits which indicates status (image update or measuring statuses)
pub const REG_CTRL_MEAS_ADDR: u8 = 0xF4; 
// REG_CTRL_MEAS_ADDR controls oversampling of the temp data and power mode
pub const _REG_CONFIG_ADDR: u8 = 0xF5; 
// _REG_CONFIG_ADDR sets rate, filter and interface options of the device
pub const REG_PRES_ADDR: u8 = 0xF7; // pressure measurement: _msb, _lsb, _xlsb (depending on resolution)
pub const REG_TEMP_ADDR: u8 = 0xFA; // temperature measurement:  _msb, _lsb, _xlsb (depending on resolution)
pub const REG_DIG_T1_ADDR: u8 = 0x88;
pub const REG_DIG_T2_ADDR: u8 = 0x8A; 
pub const REG_DIG_T3_ADDR: u8 = 0x8C; 
pub const REG_DIG_P1_ADDR: u8 = 0x8E; 
pub const REG_DIG_P2_ADDR: u8 = 0x90; 
pub const REG_DIG_P3_ADDR: u8 = 0x92; 
pub const REG_DIG_P4_ADDR: u8 = 0x94; 
pub const REG_DIG_P5_ADDR: u8 = 0x96; 
pub const REG_DIG_P6_ADDR: u8 = 0x98; 
pub const REG_DIG_P7_ADDR: u8 = 0x9A; 
pub const REG_DIG_P8_ADDR: u8 = 0x9C;
pub const REG_DIG_P9_ADDR: u8 = 0x9E;

/* BITMASKS */
pub const OSRS_P_MASK: u8 = 0b00011100;
pub const OSRS_T_MASK: u8 = 0b11100000;

// Architecture 1: use traits, multiple Bme280 structs
// trait Bme280 {
//     fn new() -> Self;
//     fn read_u8() -> u8;
//     fn read_u16() -> u16;
// }

// then after defining trait, implement these:
// struct Bme280_pins67 which implements trait Bme280
// struct Bme280_pins89 which implements trait Bme280
// struct Bme280_pins1011 which implements trait Bme280
// but the issue is that there are many repeated functions - only new() is unique

// can we not instead make a Bme280 generic type?
// new() function does not allow us to 

// Architecture 2: Generic struct
// Generic struct - pass in I2C instead of creating it inside here - too difficult
// and not much benefit to create i2c object in this module
pub struct Bme280<I2cT, Delay> {
    pub i2c_addr: u8,
    i2c: I2cT,
    delay: Delay,
    pub config: Bme280Configs
}

pub struct Bme280Configs {
    pub chip_id: u8,
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,
    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,
    pub osrs_p: Bme280Resolution,
    pub osrs_t: Bme280Resolution
}

impl Default for Bme280Configs {
    fn default() -> Self {
        Bme280Configs {
            chip_id: 0,
            dig_t1: 0,
            dig_t2: 0,
            dig_t3: 0,
            dig_p1: 0,
            dig_p2: 0,
            dig_p3: 0,
            dig_p4: 0,
            dig_p5: 0,
            dig_p6: 0,
            dig_p7: 0,
            dig_p8: 0,
            dig_p9: 0,
            osrs_p: Bme280Resolution::Skip,
            osrs_t: Bme280Resolution::Skip
        }
    }
}

#[repr(u8)] // allow us to cast enums to u8s
#[derive(Copy, Clone)]
pub enum Bme280Resolution {
    Skip = 0,
    UltraLowPower = 1,
    LowPower = 2,
    StandardRes = 4,
    HighRes = 8,
    UltraHighRes = 16
}


impl Bme280Resolution {
    pub fn reverse(val: u8) -> Self {
        match val {
            0 => Bme280Resolution::Skip,
            1 => Bme280Resolution::UltraLowPower,
            2 => Bme280Resolution::LowPower,
            4 => Bme280Resolution::StandardRes,
            8 => Bme280Resolution::HighRes,
            _ => Bme280Resolution::UltraHighRes
        }
    }
}

#[repr(u8)]
pub enum Bme280Mode {
    Sleep = 0,
    Forced = 1,
    Normal = 3,
}

// Specific implementation
impl<I2cT, Delay> Bme280<I2cT, Delay>
where 
    <I2cT as WriteRead>::Error: Debug,
    <I2cT as Write>::Error: Debug,
    <I2cT as Read>::Error: Debug,
    I2cT: embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead + 
          embedded_hal::prelude::_embedded_hal_blocking_i2c_Read +
          embedded_hal::prelude::_embedded_hal_blocking_i2c_Write,
    Delay: embedded_hal::blocking::delay::DelayMs<u32>
{
    pub fn new(i2c: I2cT, delay: Delay) -> Self {
        Bme280 {
            i2c_addr: BME280_ADDR,
            i2c: i2c,
            delay: delay,
            config: Bme280Configs::default()
        }   
    }

    pub fn init(&mut self, temp_res: Bme280Resolution, pres_res: Bme280Resolution) {
        self.reset();
        self.delay.delay_ms(500_u32);
        self.set_power_mode(Bme280Mode::Normal);
        self.delay.delay_ms(500_u32);
        self.write_temp_res(temp_res);
        self.delay.delay_ms(500_u32);
        self.write_pres_res(pres_res);
        self.delay.delay_ms(500_u32);
    
        // Debug
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        hprintln!("control reg after init: {}", rx_buffer[0]);
    }

    pub fn reset(&mut self) {
        self.write_u8(REG_RESET_ADDR, RESET_VAL)
    }

    pub fn read_configs(&mut self) {
        self.config = Bme280Configs {
            chip_id: self.read_u8(REG_ID_ADDR),
            dig_t1: self.read_u16(REG_DIG_T1_ADDR),
            dig_t2: self.read_i16(REG_DIG_T2_ADDR),
            dig_t3: self.read_i16(REG_DIG_T3_ADDR),
            dig_p1: self.read_u16(REG_DIG_P1_ADDR),
            dig_p2: self.read_i16(REG_DIG_P2_ADDR),
            dig_p3: self.read_i16(REG_DIG_P3_ADDR),
            dig_p4: self.read_i16(REG_DIG_P4_ADDR),
            dig_p5: self.read_i16(REG_DIG_P5_ADDR),
            dig_p6: self.read_i16(REG_DIG_P6_ADDR),
            dig_p7: self.read_i16(REG_DIG_P7_ADDR),
            dig_p8: self.read_i16(REG_DIG_P8_ADDR),
            dig_p9: self.read_i16(REG_DIG_P9_ADDR),
            osrs_p: self.read_pres_res(),
            osrs_t: self.read_temp_res()
        };
        if self.config.chip_id != BME280_CHIP_ID {
            panic!("Actual chip id: {}, expected chip id: {}", self.config.chip_id, BME280_CHIP_ID);
        }
    }

    pub fn set_power_mode(&mut self, mode: Bme280Mode) {
        let mode: u8 = mode as u8;
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        let existing_vals = !(OSRS_P_MASK | OSRS_T_MASK) & rx_buffer[0];
        let write_val = (mode) | existing_vals;
        //hprintln!("write_val: {}", write_val);
        self.write_u8(REG_CTRL_MEAS_ADDR, write_val);
    }


    pub fn read_pres_res(&mut self) -> Bme280Resolution {
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        Bme280Resolution::reverse((rx_buffer[0] & OSRS_P_MASK) >> 2 as u8)
    }

    pub fn read_temp_res(&mut self) -> Bme280Resolution {
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        Bme280Resolution::reverse((rx_buffer[0] & OSRS_T_MASK) >> 5 as u8)
    }

    pub fn write_temp_res(&mut self, temp_res: Bme280Resolution) {
        let temp_res = temp_res as u8;
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        //hprintln!("write temp existing control reg: {}", rx_buffer[0]);
        let existing_vals = !OSRS_T_MASK & rx_buffer[0];
        let write_val = (temp_res << 5) | existing_vals;
        self.write_u8(REG_CTRL_MEAS_ADDR, write_val);
    }

    pub fn write_pres_res(&mut self, pres_res: Bme280Resolution) {
        let pres_res: u8 = pres_res as u8;
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        let existing_vals = !OSRS_P_MASK & rx_buffer[0];
        //hprintln!("write pres existing control reg: {}", rx_buffer[0]);
        let write_val = (pres_res << 2) | existing_vals;
        //hprintln!("pres write_val: {}", write_val);
        self.write_u8(REG_CTRL_MEAS_ADDR, write_val);
    }

    pub fn write_u8(&mut self, addr: u8, value: u8) {
        let mut rx_buffer: [u8; 2] = [addr, value];
        self.i2c.write(self.i2c_addr, &mut rx_buffer).unwrap();
    }

    pub fn read_i8(&mut self, addr: u8) -> i8 {
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[addr], &mut rx_buffer).unwrap();
        return rx_buffer[0] as i8;
    }

    pub fn read_u8(&mut self, addr: u8) -> u8 {
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[addr], &mut rx_buffer).unwrap();
        return rx_buffer[0];
    }

    // i2c.read: If we use a buffer of length 2, it will read the addresses starting at [addr]
    // eg. i2c.read(self.i2c_addr, &[0x88], &mut rx_buffer)
    // will return value of reg 0x88 in rx_buffer[0], value of reg 0x89 in rx_buffer[1]
    pub fn read_i16(&mut self, addr: u8) -> i16 {
        let mut rx_buffer: [u8; 2] = [0; 2];
        self.i2c.write_read(self.i2c_addr, &[addr], &mut rx_buffer).unwrap();
        return i16::from_le_bytes(rx_buffer);
    }

    pub fn read_u16(&mut self, addr: u8) -> u16 {
        let mut rx_buffer: [u8; 2] = [0; 2];
        self.i2c.write_read(self.i2c_addr, &[addr], &mut rx_buffer).unwrap();
        // hprintln!("rx_buffer[0]: {}", rx_buffer[0]);
        // hprintln!("rx_buffer[1]: {}", rx_buffer[1]);
        return u16::from_le_bytes(rx_buffer);
    }

    // TODO: convert this to unit test (compare with reading dig_t1 using read_u8 with buffer length 2)
    pub fn read_dig_t1(&mut self) -> u16 {
        let mut rx_buffer: [u8; 1] = [0; 1];
        self.i2c.write_read(self.i2c_addr, &[0x88], &mut rx_buffer).unwrap(); //lsb
        let d1 = rx_buffer[0];
        //hprintln!("read dig_t1: rx_buffer[0]: {}", rx_buffer[0]);
        self.i2c.write_read(self.i2c_addr, &[0x89], &mut rx_buffer).unwrap(); //msb
        let d2 = rx_buffer[0];
        //hprintln!("read dig_t1: rx_buffer[0]: {}", rx_buffer[0]);
        let mut rx_buffer_2: [u8; 2] = [0; 2];
        rx_buffer_2[0] = d1; //lsb
        rx_buffer_2[1] = d2; //msb
        return u16::from_le_bytes(rx_buffer_2);
    }

    pub fn read_temperature(&mut self) -> f32 {
        let adc_temp = self.read_temp_adc();
        hprintln!("adc_temp: {}", adc_temp);
        let (_t_fine, temp) = self.compensate_t_double(adc_temp);
        return temp as f32; // celsius
    }

    pub fn read_temp_adc(&mut self) -> i32 {
        let mut rx_buffer: [u8; 3] = [0; 3];
        self.i2c.write_read(self.i2c_addr, &[REG_TEMP_ADDR], &mut rx_buffer).unwrap();
        let mut msb = rx_buffer[0] as u64;
        let mut lsb = rx_buffer[1] as u64;
        let mut xlsb = rx_buffer[2] as u64;
        if msb == 0x80 && lsb == 0x00 && xlsb == 0x0 {
            // TODO: Bad news - turned off!!
        }
        msb = msb << 16;
        lsb = lsb << 8;
        xlsb = xlsb;
        let mut sum = msb + lsb + xlsb;
        sum = sum >> 4;
        sum as i32
    }

    fn compensate_t_double(&self, adc_temp: i32) -> (f64, i32) {
        let adc_t_64: f64 = adc_temp as f64;
        let dig_t1_64: f64 = self.config.dig_t1 as f64;
        let dig_t2_64: f64 = self.config.dig_t2 as f64;
        let dig_t3_64: f64 = self.config.dig_t3 as f64;
        // Test values to make sure it is working - TODO: conver this to unit test
        // adc_t_64 = 519888.0;
        // dig_t1_64 = 27504.0;
        // dig_t2_64 = 26435.0;
        // dig_t3_64 = -1000.0;
        let var1 = (adc_t_64/16384.0 - dig_t1_64/1024.0) * dig_t2_64;
        let var2 = ((adc_t_64/131072.0 - dig_t1_64/8192.0)) * ((adc_t_64/131072.0 - dig_t1_64/8192.0)) * dig_t3_64;
        let t_fine = var1 + var2;
        return (t_fine, (t_fine / 5120.0) as i32);
    }

    pub fn read_pressure(&mut self) -> f32 {
        let adc_pres= self.read_pres_adc();
        hprintln!("adc_pres: {}", adc_pres);
        let adc_temp = self.read_temp_adc();
        let (t_fine, _temp) = self.compensate_t_double(adc_temp);
        let pres = self.compensate_p_double(t_fine, adc_pres);
        return pres as f32; // Pa
    }

    pub fn read_pres_adc(&mut self) -> i32 {
        let mut rx_buffer: [u8; 3] = [0; 3];
        self.i2c.write_read(self.i2c_addr, &[REG_PRES_ADDR], &mut rx_buffer).unwrap();
        let mut msb = rx_buffer[0] as u64;
        let mut lsb = rx_buffer[1] as u64;
        let mut xlsb = rx_buffer[2] as u64;
        if msb == 0x80 && lsb == 0x00 && xlsb == 0x0 {
            // TODO: Bad news - turned off!!
        }
        // hprintln!("pres msb: {}", msb);
        // hprintln!("pres lsb: {}", lsb);
        // hprintln!("pres xlsb: {}", xlsb);
        msb = msb << 16;
        lsb = lsb << 8;
        xlsb = xlsb;
        let mut sum = msb + lsb + xlsb;
        sum = sum >> 4;
        sum as i32
    }

    fn compensate_p_double(&mut self, t_fine: f64, adc_pres: i32) -> i32 {
        let adc_p_64: f64 = adc_pres as f64;
        hprintln!("adc_p_64: {}", adc_p_64);
        let dig_p1_64 = self.config.dig_p1 as f64;
        let dig_p2_64 = self.config.dig_p2 as f64;
        let dig_p3_64 = self.config.dig_p3 as f64;
        let dig_p4_64 = self.config.dig_p4 as f64;
        let dig_p5_64 = self.config.dig_p5 as f64;
        let dig_p6_64 = self.config.dig_p6 as f64;
        let dig_p7_64 = self.config.dig_p7 as f64;
        let dig_p8_64 = self.config.dig_p8 as f64;
        let dig_p9_64 = self.config.dig_p9 as f64;
        // let dig_p1_64: f64 = 36477.0;
        // let dig_p2_64: f64 = -10685.0;
        // let dig_p3_64: f64 = 3024.0;
        // let dig_p4_64: f64 = 2855.0;
        // let dig_p5_64: f64 = 140.0;
        // let dig_p6_64: f64 = -7.0;
        // let dig_p7_64: f64 = 15500.0;
        // let dig_p8_64: f64 = -14600.0;
        // let dig_p9_64: f64 = 6000.0;

        let mut var1 = t_fine/2.0 - 64000.0;
        let mut var2 = var1 * var1 * dig_p6_64 / 32768.0;
        var2 = var2 + var1 * dig_p5_64 * 2.0;
        var2 = var2 / 4.0 + dig_p4_64 * 65536.0;
        var1 = (dig_p3_64 * var1 * var1 / 524288.0) + (dig_p2_64 * var1 / 524288.0);
        var1 = (1.0 + var1 / 32768.0) * dig_p1_64;
        let mut p = 1048576.0 - adc_p_64;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = (dig_p9_64) * p * p / 2147483648.0;
        var2 = p * dig_p8_64 / 32768.0;
        p = p + (var1 + var2 + dig_p7_64) / 16.0;
        hprintln!("p: {}", p);
        return p as i32;
    }

}

