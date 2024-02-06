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

/// Errors in this crate
#[derive(Debug, Format)]
pub enum HMC5883Error<CommE> {
    /// Sensor communication error
    Comm(CommE),

    /// Sensor reading out of range
    OutOfRange,

    /// Configuration reads invalid
    Configuration,

    /// Unrecognized chip ID
    UnknownChipId,
}

/// Gain settings ( in LSb/Gauss )
/// One tesla (T) is equal to 104 gauss
#[repr(u8)]
pub enum GainSetting {
    ///± 0.88 Ga  / 0.73 (mGa/LSb)
    Gain1370 = 0b00000000,
    ///± 1.30 Ga  / 0.92 (mGa/LSb)
    Gain1090 = 0b00100000,
    ///± 1.90 Ga  / 1.22 (mGa/LSb)
    Gain0820 = 0b01000000,
    ///± 2.50 Ga  / 1.52 (mGa/LSb)
    Gain0660 = 0b01100000,
    ///± 4.00 Ga  / 2.27 (mGa/LSb)
    Gain0440 = 0b10000000,
    ///± 4.70 Ga  / 2.56 (mGa/LSb)
    Gain0390 = 0b10100000,
    ///± 5.60 Ga  / 3.03 (mGa/LSb)
    Gain0330 = 0b11000000,
    ///± 8.10 Ga  / 4.35 (mGa/LSb)
    Gain0230 = 0b11100000,
}

/// Output Data Rate settings in Hz
#[repr(u8)]
pub enum OdrSetting {
    Odr0_75Hz = 0b000,
    Odr1_5Hz = 0b001,
    Odr3_0Hz = 0b010,
    Odr7_5Hz = 0b011,
    Odr15_0Hz = 0b100,
    Odr30_0Hz = 0b110,
    Odr220_0Hz = 0b111,
}

/// Configuring sample averaging
#[repr(u8)]
pub enum SampleAvgSetting {
    AvgSamples1 = 0b00,
    AvgSamples2 = 0b01,
    AvgSamples4 = 0b10,
    /// Average 8 samples
    AvgSamples8 = 0b11,
}

/// Measurement mode settings
#[repr(u8)]
pub enum MeasurementModeSetting {
    NormalMode = 0b00,
    /// Positive bias current
    PositiveBias = 0b01,
}
