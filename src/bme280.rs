/* BME280 driver
 *
 * Rev.         Author          Date                Notes
 * ===          ======          ====                =====
 * 0.1          Jian Lik Ng     2023-09-03          Initial Release
 * 
 * See links for reference:
 * https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
 * https://apollolabsblog.hashnode.dev/stm32f4-embedded-rust-at-the-hal-i2c-temperature-pressure-sensing-with-bmp180
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
 * https://docs.rs/bme280-rs/latest/src/bme280_rs/bme280.rs.html#87-102 
 */

use cortex_m_semihosting::hprintln;
use embedded_hal::blocking::i2c::{Write, Read, WriteRead};
use core::fmt::Debug;

/* CHIP CONSTANTS */
pub const BME280_CHIP_ID:       u8 = 0x60;  // BME280 ID 0x60 - temp, press and humidity sensor
pub const _BMP280_CHIP_ID:      u8 = 0x58;  // BMP280 ID 0x60 - only temp and press sensor
pub const RESET_VAL:            u8 = 0xB6;

/* ADDRESSES */
pub const BME280_ADDR:          u8 = 0x76;  // I2C address
pub const REG_ID_ADDR:          u8 = 0xD0;  // chip ID
pub const REG_RESET_ADDR:       u8 = 0xE0;  // Writing 0xB6 to REG_RESET_ADDR will power on reset the device

pub const REG_CTRL_HUM_ADDR:    u8 = 0xF2;  // Changes to this register only become effective after write operation to CTRL_MEAS
pub const _REG_STATUS_ADDR:     u8 = 0xF3;  // _REG_STATUS_ADDR contains two bits which indicates status (image update or measuring statuses)
pub const REG_CTRL_MEAS_ADDR:   u8 = 0xF4;  // REG_CTRL_MEAS_ADDR controls oversampling of the temp data and power mode
pub const _REG_CONFIG_ADDR:     u8 = 0xF5;  // _REG_CONFIG_ADDR sets rate, filter and interface options of the device
pub const REG_PRES_ADDR:        u8 = 0xF7;  // pressure measurement: _msb, _lsb, _xlsb (depending on resolution)
pub const REG_TEMP_ADDR:        u8 = 0xFA;  // temperature measurement:  _msb, _lsb, _xlsb (depending on resolution)
pub const REG_DIG_T1_ADDR:      u8 = 0x88;
pub const REG_DIG_T2_ADDR:      u8 = 0x8A; 
pub const REG_DIG_T3_ADDR:      u8 = 0x8C; 
pub const REG_DIG_P1_ADDR:      u8 = 0x8E; 
pub const REG_DIG_P2_ADDR:      u8 = 0x90; 
pub const REG_DIG_P3_ADDR:      u8 = 0x92; 
pub const REG_DIG_P4_ADDR:      u8 = 0x94; 
pub const REG_DIG_P5_ADDR:      u8 = 0x96; 
pub const REG_DIG_P6_ADDR:      u8 = 0x98; 
pub const REG_DIG_P7_ADDR:      u8 = 0x9A; 
pub const REG_DIG_P8_ADDR:      u8 = 0x9C;
pub const REG_DIG_P9_ADDR:      u8 = 0x9E;
pub const REG_DIG_H1_ADDR:      u8 = 0xA1;
pub const REG_DIG_H2_ADDR:      u8 = 0xE1;
pub const REG_DIG_H3_ADDR:      u8 = 0xE3;
pub const REG_DIG_H4_ADDR:      u8 = 0xE4;
pub const REG_DIG_H5_ADDR:      u8 = 0xE5;
pub const REG_DIG_H6_ADDR:      u8 = 0xE7;

/* BITMASKS */
pub const OSRS_P_MASK:          u8 = 0b00011100;
pub const OSRS_T_MASK:          u8 = 0b11100000;
pub const OSRS_H_MASK:          u8 = 0b00000111;
pub const DIG_H4_MASK:          u16 = 0x0FFF;
pub const DIG_H5_MASK:          u16 = 0xFFF0;

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
    pub dig_h1: u8,
    pub dig_h2: i16,
    pub dig_h3: u8,
    pub dig_h4: i16, 
    pub dig_h5: i16, 
    pub dig_h6: u8,
    pub osrs_p: Bme280Resolution,
    pub osrs_t: Bme280Resolution,
    pub osrs_h: Bme280Resolution,
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
            dig_h1: 0,
            dig_h2: 0,
            dig_h3: 0,
            dig_h4: 0, 
            dig_h5: 0, 
            dig_h6: 0,
            osrs_p: Bme280Resolution::Skip,
            osrs_t: Bme280Resolution::Skip,
            osrs_h: Bme280Resolution::Skip
        }
    }
}

#[repr(u8)] // allow us to cast enums to u8s
#[derive(Copy, Clone)]
pub enum Bme280Resolution {
    Skip = 0b000,
    UltraLowPower = 0b001,
    LowPower = 0b010,
    StandardRes = 0b011,
    HighRes = 0b100,
    UltraHighRes = 0b101
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

pub struct Bme280ResolutionConfig{
    pub temp_res: Bme280Resolution,
    pub pres_res: Bme280Resolution,
    pub humd_res: Bme280Resolution
}

pub const BME280_RES_CONFIG_WEATHER_MONITORING: Bme280ResolutionConfig = Bme280ResolutionConfig{
    temp_res: Bme280Resolution::UltraLowPower, 
    pres_res: Bme280Resolution::UltraLowPower, 
    humd_res: Bme280Resolution::UltraLowPower
};

pub const BME280_RES_CONFIG_HUMIDITY_SENSING: Bme280ResolutionConfig = Bme280ResolutionConfig{
    temp_res: Bme280Resolution::UltraLowPower, 
    pres_res: Bme280Resolution::Skip, 
    humd_res: Bme280Resolution::UltraLowPower,
};

pub const BME280_RES_CONFIG_INDOOR_NAVIGATION: Bme280ResolutionConfig = Bme280ResolutionConfig{
    temp_res: Bme280Resolution::LowPower, 
    pres_res: Bme280Resolution::UltraHighRes,
    humd_res: Bme280Resolution::UltraLowPower
};
pub const BME280_RES_CONFIG_GAMING: Bme280ResolutionConfig =  Bme280ResolutionConfig{
    temp_res: Bme280Resolution::UltraLowPower, 
    pres_res: Bme280Resolution::StandardRes, 
    humd_res: Bme280Resolution::Skip
};


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

    pub fn init(&mut self, res_config: Bme280ResolutionConfig) {
        self.reset();
        self.delay.delay_ms(250_u32);
        self.set_ctrl_meas(Bme280Mode::Normal, res_config);
        self.delay.delay_ms(250_u32);
    
        // Debug
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        hprintln!("control reg after init: {}", rx_buffer[0]);
    }

    pub fn reset(&mut self) {
        self.write_u8(REG_RESET_ADDR, RESET_VAL)
    }

    pub fn read_configs(&mut self) {

        let (_mode, osrs_p, osrs_t, osrs_h) = self.read_ctrl_meas();
        let (dig_h1, dig_h2, dig_h3, dig_h4, dig_h5, dig_h6) = self.read_humd_config();

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
            dig_h1: dig_h1,
            dig_h2: dig_h2,
            dig_h3: dig_h3,
            dig_h4: dig_h4,
            dig_h5: dig_h5,
            dig_h6: dig_h6,
            osrs_p: osrs_p,
            osrs_t: osrs_t,
            osrs_h: osrs_h
        };
        if self.config.chip_id != BME280_CHIP_ID {
            panic!("Actual chip id: {}, expected chip id: {}", self.config.chip_id, BME280_CHIP_ID);
        }
    }

    pub fn read_ctrl_meas(&mut self) -> (u8, Bme280Resolution, Bme280Resolution, Bme280Resolution) 
    {
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        let pres_res = Bme280Resolution::reverse((rx_buffer[0] & OSRS_P_MASK) >> 2 as u8);
        let temp_res = Bme280Resolution::reverse((rx_buffer[0] & OSRS_T_MASK) >> 5 as u8);
        let mode = (rx_buffer[0] & !(OSRS_T_MASK | OSRS_P_MASK)) as u8;
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_HUM_ADDR], &mut rx_buffer).unwrap();
        let humd_res = Bme280Resolution::reverse((rx_buffer[0] & OSRS_H_MASK) as u8);
        (mode, pres_res, temp_res, humd_res)
    }

    pub fn read_humd_config(&mut self) -> (u8, i16, u8, i16, i16, u8) 
    {
        let dig_h1: u8 = self.read_u8(REG_DIG_H1_ADDR);
        let dig_h2: i16 = self.read_i16(REG_DIG_H2_ADDR);
        let dig_h3: u8 = self.read_u8(REG_DIG_H3_ADDR);
        let dig_h4: u16 = self.read_u16(REG_DIG_H4_ADDR);
        let dig_h4: i16 = (dig_h4 & DIG_H4_MASK) as i16;
        let dig_h5: u16 = self.read_u16(REG_DIG_H5_ADDR);
        let dig_h5: i16 = ((dig_h5 & DIG_H5_MASK) >> 4) as i16;
        let dig_h6: u8 = self.read_u8(REG_DIG_H6_ADDR);
        (dig_h1, dig_h2, dig_h3, dig_h4, dig_h5, dig_h6)
    }

    pub fn set_ctrl_meas(&mut self, mode: Bme280Mode, res_config: Bme280ResolutionConfig)
    {
        let mode: u8 = mode as u8;
        let mut temp_res = res_config.temp_res as u8;
        let mut pres_res = res_config.pres_res as u8;
        let humd_res = res_config.humd_res as u8;
        self.write_u8(REG_CTRL_HUM_ADDR, humd_res);
        let write_val = mode + (pres_res << 2) + (temp_res << 5);
        self.write_u8(REG_CTRL_MEAS_ADDR, write_val);
    }

    pub fn set_power_mode(&mut self, mode: Bme280Mode) {
        let mode: u8 = mode as u8;
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        let existing_vals = !(OSRS_P_MASK | OSRS_T_MASK) & rx_buffer[0];
        let write_val = (mode) | existing_vals;
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

    pub fn write_pres_res(&mut self, pres_res: Bme280Resolution) {
        let pres_res: u8 = pres_res as u8;
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        let existing_vals = !OSRS_P_MASK & rx_buffer[0];
        let write_val = (pres_res << 2) | existing_vals;
        self.write_u8(REG_CTRL_MEAS_ADDR, write_val);
    }

    pub fn write_temp_res(&mut self, temp_res: Bme280Resolution) {
        let temp_res = temp_res as u8;
        let mut rx_buffer: [u8; 1] = [0];
        self.i2c.write_read(self.i2c_addr, &[REG_CTRL_MEAS_ADDR], &mut rx_buffer).unwrap();
        let existing_vals = !OSRS_T_MASK & rx_buffer[0];
        let write_val = (temp_res << 5) | existing_vals;
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
        return u16::from_le_bytes(rx_buffer);
    }

    // TODO: convert this to unit test (compare with reading dig_t1 using read_u8 with buffer length 2)
    pub fn read_dig_t1(&mut self) -> u16 {
        let mut rx_buffer: [u8; 1] = [0; 1];
        self.i2c.write_read(self.i2c_addr, &[0x88], &mut rx_buffer).unwrap(); //lsb
        let d1 = rx_buffer[0];
        self.i2c.write_read(self.i2c_addr, &[0x89], &mut rx_buffer).unwrap(); //msb
        let d2 = rx_buffer[0];
        let mut rx_buffer_2: [u8; 2] = [0; 2];
        rx_buffer_2[0] = d1; //lsb
        rx_buffer_2[1] = d2; //msb
        return u16::from_le_bytes(rx_buffer_2);
    }

    pub fn read_environment(&mut self) -> (i32, u32, u32) {
        let (temp_adc, pres_adc, humd_adc) = self.read_adc();
        let temp_adc = temp_adc.unwrap();
        let pres_adc = pres_adc.unwrap();
        let humd_adc = humd_adc.unwrap();
        let (t_fine, temp) = self.compensate_t(temp_adc).unwrap();
        let pres = self.compensate_p(t_fine, pres_adc).unwrap();
        let humd = self.compensate_h(t_fine, humd_adc).unwrap();
        (temp, pres, humd)
    }

    // TODO: Convert test values to unit test
    // adc_t_64 = 519888.0;
    // dig_t1_64 = 27504.0;
    // dig_t2_64 = 26435.0;
    // dig_t3_64 = -1000.0;
    fn compensate_t(&self, temp_adc: i32) -> Option<(i32, i32)> {
        let dig_t1 = self.config.dig_t1 as i32;
        let dig_t2 = self.config.dig_t2 as i32;
        let dig_t3: i32 = self.config.dig_t3 as i32;
        let var1 = (((temp_adc >> 3) - (dig_t1 << 1)) * dig_t2) >> 11;
        let var2 = ((((temp_adc >> 4) - dig_t1) * ((temp_adc >> 4) - (dig_t1)) >> 12) * dig_t3) >> 14;
        let t_fine = var1 + var2;
        let t = (t_fine * 5 + 128) >> 8;
        Some((t_fine, t))
    }

    // TODO: Convert test values to unit test
    // let dig_p1_64: f64 = 36477.0;
    // let dig_p2_64: f64 = -10685.0;
    // let dig_p3_64: f64 = 3024.0;
    // let dig_p4_64: f64 = 2855.0;
    // let dig_p5_64: f64 = 140.0;
    // let dig_p6_64: f64 = -7.0;
    // let dig_p7_64: f64 = 15500.0;
    // let dig_p8_64: f64 = -14600.0;
    // let dig_p9_64: f64 = 6000.0;
    fn compensate_p(&mut self, t_fine: i32, adc_pres: i32) -> Option<u32> {
        let dig_p1 = self.config.dig_p1 as i32;
        let dig_p2 = self.config.dig_p2 as i32;
        let dig_p3 = self.config.dig_p3 as i32;
        let dig_p4 = self.config.dig_p4 as i32;
        let dig_p5 = self.config.dig_p5 as i32;
        let dig_p6 = self.config.dig_p6 as i32;
        let dig_p7 = self.config.dig_p7 as i32;
        let dig_p8 = self.config.dig_p8 as i32;
        let dig_p9 = self.config.dig_p9 as i32;

        let mut var1 = (t_fine >> 1) - 64000;
        let mut var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * dig_p6;
        var2 = var2 + ((var1 * dig_p5) << 1);
        var2 = (var2 >> 2) + (dig_p4 << 16);
        var1 = (((dig_p3 * ((var1 >> 2) * (var1 >> 2) >> 13)) >> 3) + ((dig_p2 * var1) >> 1)) >> 18;
        var1 = ((32768 + var1) * (dig_p1)) >> 15;
        if var1 == 0 {
            return None;
        }
        let mut p = (((1048576 - adc_pres) - (var2 >> 12)) * 3125) as u32;
        if p < 0x80000000 {
            p = (p << 1) / (var1 as u32);
        }
        else {
            p = p / (var1 as u32) * 2;
        }
        var1 = (dig_p9 * (((p >> 3) * (p >> 3)) >> 13) as i32) >> 12;
        var2 = (((p >> 2) as i32) * dig_p8) >> 13;
        p = ((p as i32) + ((var1 + var2 + dig_p7) >> 4)) as u32;
        Some(p)
    }

    fn compensate_h(&mut self, t_fine: i32, adc_humd: i32) -> Option<u32> {
        let dig_h1 = self.config.dig_h1 as i32;
        let dig_h2 = self.config.dig_h2 as i32;
        let dig_h3 = self.config.dig_h3 as i32;
        let dig_h4 = self.config.dig_h4 as i32;
        let dig_h5 = self.config.dig_h5 as i32;
        let dig_h6 = self.config.dig_h6 as i32;

        let mut v_x1_u32r: i32 = (t_fine as i32) - 76800;
        v_x1_u32r = 
            (((adc_humd << 14) - ((dig_h4 << 20) - (dig_h5 * v_x1_u32r)) + 16384) >> 15) * 
            (
                (
                    (
                        (
                            (
                                ((v_x1_u32r * dig_h6) >> 10) * 
                                (((v_x1_u32r * dig_h3) >> 11) + 32768)
                            ) >> 10
                        ) + 2097152
                    ) * dig_h2 + 8192
                ) >> 14
            );
        let temp1 = (v_x1_u32r >> 15) as i64; // unfortunately stm32f1xx crashes at next line if we use i32
        v_x1_u32r = ((v_x1_u32r as i64)- ((((temp1 * temp1) >> 7) * (dig_h1 as i64)) >> 4)) as i32;
        v_x1_u32r = if v_x1_u32r < 0 { 0 } else { v_x1_u32r };
        v_x1_u32r = if v_x1_u32r > 419430400 { 419430400 } else { v_x1_u32r };
        Some((v_x1_u32r >> 12) as u32)
    }

    pub fn read_adc(&mut self) -> (Option<i32>, Option<i32>, Option<i32>) {
        let mut rx_buffer: [u8; 8] = [0; 8]; // 3 bytes for pres, 3 bytes for temp, 3 bytes for humidity
        self.i2c.write_read(self.i2c_addr, &[REG_PRES_ADDR], &mut rx_buffer).unwrap();
        let pres_msb  = rx_buffer[0] as u64;
        let pres_lsb  = rx_buffer[1] as u64;
        let pres_xlsb = rx_buffer[2] as u64;
        let pres_adc = 
            if pres_msb == 0x80 && pres_lsb == 0x00 && pres_xlsb == 0x0 {
                None // Sleeping or skipped
            } else {
                let sum = ((pres_msb << 16) + (pres_lsb << 8) + (pres_xlsb)) >> 4; // need brackets around bitshift or will crash
                Some(sum as i32)
            };
        let temp_msb  = rx_buffer[3] as u64;
        let temp_lsb  = rx_buffer[4] as u64;
        let temp_xlsb = rx_buffer[5] as u64;
        let temp_adc = 
            if temp_msb == 0x80 && temp_lsb == 0x00 && temp_xlsb == 0x0 {
                None // Sleeping or skipped
            } else {
                let sum = ((temp_msb << 16) + (temp_lsb << 8) + (temp_xlsb)) >> 4; // need brackets around bitshift or will crash
                Some(sum as i32)
            };
        let humd_msb  = rx_buffer[6] as u64;
        let humd_lsb  = rx_buffer[7] as u64;
        let humd_adc = 
            if humd_msb == 0x80 && humd_lsb == 0x00 {
                None // Sleeping or skipped
            } else {
                let sum = (humd_msb << 8) + humd_lsb;
                Some(sum as i32)
            };
        (temp_adc, pres_adc, humd_adc)
    }

}

