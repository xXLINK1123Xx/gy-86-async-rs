use defmt::Format;

pub const I2C_ADDRESS: u8 = 0x1E;

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub struct HMC5883L_CONFIG;

impl HMC5883L_CONFIG {
    pub const ADDR: u8 = 0x1E;
    pub const REG_CONFIG_A: u8 = 0x00;
    pub const REG_CONFIG_B: u8 = 0x01;
    pub const REG_MODE: u8 = 0x02;
    pub const REG_TEMP_OUTPUT_MSB: u8 = 0x31;
    pub const REG_OUT_X_M: u8 = 0x03;
    pub const REG_OUT_X_L: u8 = 0x04;
    pub const REG_OUT_Z_M: u8 = 0x05;
    pub const REG_OUT_Z_L: u8 = 0x06;
    pub const REG_OUT_Y_M: u8 = 0x07;
    pub const REG_OUT_Y_L: u8 = 0x08;
    pub const REG_STATUS: u8 = 0x09;
    pub const REG_IDENT_A: u8 = 0x0A;
    pub const REG_IDENT_B: u8 = 0x0B;
    pub const REG_IDENT_C: u8 = 0x0C;
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Compass {
    North = 0,
    South = 1,
    West = 2,
    East = 3,
    Up = 4,
    Down = 5,
}

pub const COMPASS_HORIZONTAL_X_NORTH: u8 =
    (((Compass::North as u8) << 6) | ((Compass::West as u8) << 3) | Compass::Up as u8) << 5;
pub const COMPASS_HORIZONTAL_Y_NORTH: u8 =
    (((Compass::East as u8) << 6) | ((Compass::North as u8) << 3) | Compass::Up as u8) << 5;
pub const COMPASS_VERTICAL_X_EAST: u8 =
    (((Compass::East as u8) << 6) | ((Compass::Up as u8) << 3) | (Compass::West as u8)) << 5;
pub const COMPASS_VERTICAL_Y_WEST: u8 =
    (((Compass::Up as u8) << 6) | ((Compass::West as u8) << 3) | (Compass::West as u8)) << 5;
