use defmt::debug;
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

use crate::reg_data::hmc5883::*;

#[derive(Debug)]
pub struct HMC5983<I2C> {
    i2c: I2C,
}

impl<I2C, CommE> HMC5983<I2C>
where
    I2C: I2c<Error = CommE>,
    CommE: core::fmt::Debug,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    pub async fn init(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<(), HMC5883Error<CommE>> {
        self.reset(delay_source).await
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), HMC5883Error<CommE>> {
        let write_buf = [reg, val];

        self.i2c
            .write(I2C_ADDRESS, &write_buf)
            .await
            .map_err(HMC5883Error::Comm)?;
        Ok(())
    }

    async fn read_block(
        &mut self,
        reg: u8,
        recv_buf: &mut [u8],
    ) -> Result<(), HMC5883Error<CommE>> {
        let cmd_buf = [reg];

        self.i2c
            .write_read(I2C_ADDRESS, &cmd_buf, recv_buf)
            .await
            .map_err(HMC5883Error::Comm)?;

        Ok(())
    }

    async fn reset(&mut self, delay_source: &mut impl DelayNs) -> Result<(), HMC5883Error<CommE>> {
        //wakeup the chip
        for reg in 0x00..0x0D {
            let _val = self.read_reg(reg).await?;
        }

        const EXPECTED_PROD_ID_A: u8 = 72; //'H';
        const EXPECTED_PROD_ID_B: u8 = 52; //'4';
        const EXPECTED_PROD_ID_C: u8 = 51; //'3';
                                           //compare product ID against known product ID
                                           //read the product identifiers
        let mut buf = [0u8; 3];
        self.read_block(REG_ID_A, &mut buf).await?;
        if buf[0] != EXPECTED_PROD_ID_A
            || buf[1] != EXPECTED_PROD_ID_B
            || buf[2] != EXPECTED_PROD_ID_C
        {
            return Err(HMC5883Error::UnknownChipId);
        }

        self.set_all_config_a(
            MeasurementModeSetting::NormalMode,
            OdrSetting::Odr30_0Hz,
            SampleAvgSetting::AvgSamples8,
            true,
        )
        .await?;

        self.set_gain(GainSetting::Gain0820).await?;
        // (Continuous-measurement mode)
        self.write_reg(REG_CONFIG_C, MeasurementModeSetting::NormalMode as u8)
            .await?;
        let _ = delay_source.delay_ms(100).await;

        Ok(())
    }

    /// Set the mag gain, which determines the range
    pub async fn set_gain(&mut self, gain: GainSetting) -> Result<(), HMC5883Error<CommE>> {
        let gain_val: u8 = gain as u8;
        self.write_reg(REG_CONFIG_B, gain_val).await?;

        let confirm_val = self.read_reg(REG_CONFIG_B).await?;
        if confirm_val != gain_val {
            debug!("gain bad: expected {} got {}", gain_val, confirm_val);
            return Err(HMC5883Error::Configuration);
        }
        Ok(())
    }

    /// Set all of the Config A register settings
    pub async fn set_all_config_a(
        &mut self,
        mode: MeasurementModeSetting,
        odr: OdrSetting,
        averaging: SampleAvgSetting,
        temp_enabled: bool,
    ) -> Result<(), HMC5883Error<CommE>> {
        let new_val = (if temp_enabled { 1 << 7 } else { 0 })
            & ((averaging as u8) << 6)
            & ((odr as u8) << 4)
            & ((mode as u8) << 2);
        self.write_reg(REG_CONFIG_A, new_val).await
    }

    /// Read a single register
    async fn read_reg(&mut self, reg: u8) -> Result<u8, HMC5883Error<CommE>> {
        let mut buf = [0u8; 1];
        self.read_block(reg, &mut buf).await?;

        Ok(buf[0])
    }

    /// Verify that a magnetometer reading is within the expected range.
    fn reading_in_range(sample: &[i16; 3]) -> bool {
        /// Maximum Dynamic Range for X and Y axes (micro Teslas)
        const MDR_XY_AXES: i16 = 1600;
        /// Maximum Dynamic Range for Z axis (micro Teslas)
        const MDR_Z_AXIS: i16 = 2500;
        /// Resolution (micro Teslas per LSB)
        const RESO_PER_BIT: f32 = 0.3;
        const MAX_VAL_XY: i16 = (((MDR_XY_AXES as f32) / RESO_PER_BIT) as i16) + 1;
        const MAX_VAL_Z: i16 = (((MDR_Z_AXIS as f32) / RESO_PER_BIT) as i16) + 1;

        sample[0].abs() < MAX_VAL_XY && sample[1].abs() < MAX_VAL_XY && sample[2].abs() < MAX_VAL_Z
    }

    /// Combine high and low bytes of i16 mag value
    fn raw_reading_to_i16(buf: &[u8], idx: usize) -> i16 {
        let val: i16 = (buf[idx] as i16) << 8 | buf[idx + 1] as i16;
        val
    }

    pub async fn get_mag_vector(&mut self) -> Result<[i16; 3], HMC5883Error<CommE>> {
        const XYZ_DATA_LEN: usize = 6;
        let mut buf = [0u8; XYZ_DATA_LEN];
        //get the actual mag data from the sensor
        self.read_block(REG_MAG_DATA_START, &mut buf).await?;
        let sample_i16 = [
            Self::raw_reading_to_i16(&buf, 0),
            Self::raw_reading_to_i16(&buf, 2),
            Self::raw_reading_to_i16(&buf, 4),
        ];

        if !Self::reading_in_range(&sample_i16) {
            debug!("bad reading?");

            return Err(HMC5883Error::OutOfRange);
        }

        //TODO do cross-axis flow calibration?
        Ok(sample_i16)
    }

    /// Read temperature from device
    /// Result is degrees Celsius
    pub async fn get_temperature(&mut self) -> Result<i16, HMC5883Error<CommE>> {
        const TEMP_DATA_LEN: usize = 2;
        let mut buf = [0; TEMP_DATA_LEN];
        self.read_block(REG_TEMP_OUTPUT_MSB, &mut buf).await?;

        //TODO datasheet is not clear whether the temp can go negative
        // Temperature=(MSB*2^8+LSB)/(2^4*8)+25in C
        let celsius = (((buf[0] as i16) * 256) + (buf[1] as i16)) / 128 + 25;
        Ok(celsius)
    }
}

const REG_CONFIG_A: u8 = 0x00;
const REG_CONFIG_B: u8 = 0x01;
const REG_CONFIG_C: u8 = 0x02;

/// X-axis output value register
const REG_DATA_X: u8 = 0x03;
// Y-axis output value register
// const REG_DATA_Y:u8 = 0x05;
// Z-axis output value register
// const REG_DATA_Z:u8	= 0x07;

// const REG_STATUS:u8 = 0x09;

/// Register to read out all three dimensions of mag data
const REG_MAG_DATA_START: u8 = REG_DATA_X;

/// Identification Register A
const REG_ID_A: u8 = 0x0A;
// Identification Register B
// const REG_ID_B: u8 = 0x0B;
// Identification Register C
// const REG_ID_C: u8 = 0x0C;

/// Temperature outputs, HMC5983
const REG_TEMP_OUTPUT_MSB: u8 = 0x31;
// const REG_TEMP_OUTPUT_LSB: u8 = 0x32;

// Status Register 2
// const REG_STATUS2: u8 = 0x09;
