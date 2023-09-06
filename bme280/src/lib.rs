#![cfg_attr(not(test), no_std)]

pub use bme280::{
    Bme280, 
    Bme280Configs,
    Bme280Resolution,
    Bme280ResolutionConfig,
    BME280_RES_CONFIG_WEATHER_MONITORING,
    BME280_RES_CONFIG_HUMIDITY_SENSING,
    BME280_RES_CONFIG_INDOOR_NAVIGATION,
    BME280_RES_CONFIG_GAMING
};

pub mod bme280;
