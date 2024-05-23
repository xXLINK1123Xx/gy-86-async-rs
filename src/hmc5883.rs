use core::f32::consts::PI;

use defmt::{debug, Format};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use nalgebra::Vector3;

use crate::reg_data::hmc5883::*;

#[derive(Debug)]
pub struct HMC5983<I2C, D> {
    i2c: I2C,
    delay: D,
    operation_mode: OperatingModeSetting,
    measurement_mode: MeasurementModeSetting,
    odr: OdrSetting,
    averaging: SampleAvgSetting,
    temp_enabled: bool,
    gain: GainSetting,
}

impl<I2C, D, CommE> HMC5983<I2C, D>
where
    I2C: I2c<Error = CommE>,
    CommE: core::fmt::Debug,
    D: DelayNs,
{
    /// create new driver with default config
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self {
            i2c,
            delay,
            measurement_mode: MeasurementModeSetting::NormalMode,
            operation_mode: OperatingModeSetting::SingleMeasurement,
            averaging: SampleAvgSetting::AvgSamples8,
            temp_enabled: false,
            odr: OdrSetting::Odr30_0Hz,
            gain: GainSetting::Gain0820,
        }
    }

    /// create new driver with custom config
    pub fn new_with_config(
        i2c: I2C,
        delay: D,
        measurement_mode: MeasurementModeSetting,
        operation_mode: OperatingModeSetting,
        averaging: SampleAvgSetting,
        odr: OdrSetting,
        gain: GainSetting,
        temp_enabled: bool,
    ) -> Self {
        Self {
            i2c,
            delay,
            measurement_mode,
            operation_mode,
            averaging,
            temp_enabled,
            odr,
            gain,
        }
    }

    pub async fn init(&mut self) -> Result<(), HMC5883Error<CommE>> {
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
        self.read_block(HMC5883L_CONFIG::REG_IDENT_A, &mut buf)
            .await?;
        if buf[0] != EXPECTED_PROD_ID_A
            || buf[1] != EXPECTED_PROD_ID_B
            || buf[2] != EXPECTED_PROD_ID_C
        {
            return Err(HMC5883Error::UnknownChipId);
        }

        self.write_current_config().await
    }

    pub async fn get_mag_vector(&mut self) -> Result<Vector3<i16>, HMC5883Error<CommE>> {
        const XYZ_DATA_LEN: usize = 6;
        let mut buf = [0u8; XYZ_DATA_LEN];
        //get the actual mag data from the sensor
        self.read_block(HMC5883L_CONFIG::REG_OUT_X_M, &mut buf)
            .await?;
        let sample_i16 = Vector3::new(
            Self::raw_reading_to_i16(&buf, 0),
            Self::raw_reading_to_i16(&buf, 2),
            Self::raw_reading_to_i16(&buf, 4),
        );

        if !Self::reading_in_range(&sample_i16) {
            debug!("bad reading?");

            return Err(HMC5883Error::OutOfRange);
        }

        //TODO do cross-axis flow calibration?
        Ok(sample_i16)
    }

    /// Get the heading of the compass in degrees (Z-axis is forward)
    pub async fn get_heading_degrees(&mut self) -> Result<f32, HMC5883Error<CommE>> {
        let heading_vec = self.get_mag_vector().await?;

        let x_gauss_data = heading_vec.x as f32 * 0.48828125;
        let y_gauss_data = heading_vec.y as f32 * 0.48828125;

        if x_gauss_data == 0.0 {
            if y_gauss_data < 0.0 {
                return Ok(90.0);
            } else {
                return Ok(0.0);
            }
        }

        let mut d = libm::atanf(y_gauss_data / x_gauss_data) * 180.0 / PI;

        d = if d < 0.0 { d + 360.0 } else { d };
        d = if d > 360.0 { d - 360.0 } else { d };

        Ok(d)
    }

    /// Read temperature from device
    /// Result is degrees Celsius
    pub async fn get_temperature(&mut self) -> Result<i16, HMC5883Error<CommE>> {
        if !self.temp_enabled {
            return Err(HMC5883Error::Configuration);
        }

        const TEMP_DATA_LEN: usize = 2;
        let mut buf = [0; TEMP_DATA_LEN];
        self.read_block(HMC5883L_CONFIG::REG_TEMP_OUTPUT_MSB, &mut buf)
            .await?;

        //TODO datasheet is not clear whether the temp can go negative
        // Temperature=(MSB*2^8+LSB)/(2^4*8)+25in C
        let celsius = (((buf[0] as i16) * 256) + (buf[1] as i16)) / 128 + 25;
        Ok(celsius)
    }

    async fn reset(&mut self) -> Result<(), HMC5883Error<CommE>> {
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
        self.read_block(HMC5883L_CONFIG::REG_IDENT_A, &mut buf)
            .await?;
        if buf[0] != EXPECTED_PROD_ID_A
            || buf[1] != EXPECTED_PROD_ID_B
            || buf[2] != EXPECTED_PROD_ID_C
        {
            return Err(HMC5883Error::UnknownChipId);
        }

        let reg_a = self.generate_a_config(
            false,
            SampleAvgSetting::AvgSamples8,
            OdrSetting::Odr30_0Hz,
            MeasurementModeSetting::NormalMode,
        );

        self.write_reg(HMC5883L_CONFIG::REG_CONFIG_A, reg_a).await?;

        self.set_gain(GainSetting::Gain0820).await?;
        // (Continuous-measurement mode)
        self.write_reg(
            HMC5883L_CONFIG::REG_MODE,
            OperatingModeSetting::SingleMeasurement as u8,
        )
        .await?;
        self.delay.delay_ms(100).await;

        Ok(())
    }

    async fn write_current_config(&mut self) -> Result<(), HMC5883Error<CommE>> {
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
        self.read_block(HMC5883L_CONFIG::REG_IDENT_A, &mut buf)
            .await?;
        if buf[0] != EXPECTED_PROD_ID_A
            || buf[1] != EXPECTED_PROD_ID_B
            || buf[2] != EXPECTED_PROD_ID_C
        {
            return Err(HMC5883Error::UnknownChipId);
        }

        let reg_a = self.generate_a_config(
            self.temp_enabled,
            self.averaging,
            self.odr,
            self.measurement_mode,
        );

        self.write_reg(HMC5883L_CONFIG::REG_CONFIG_A, reg_a).await?;

        self.set_gain(self.gain).await?;
        // (Continuous-measurement mode)
        self.write_reg(HMC5883L_CONFIG::REG_MODE, self.operation_mode as u8)
            .await?;
        self.delay.delay_ms(100).await;

        Ok(())
    }

    /// Set the mag gain, which determines the range
    pub async fn set_gain(&mut self, gain: GainSetting) -> Result<(), HMC5883Error<CommE>> {
        let gain_val = gain as u8;
        self.write_reg(HMC5883L_CONFIG::REG_CONFIG_B, gain_val)
            .await?;

        let confirm_val = self.read_reg(HMC5883L_CONFIG::REG_CONFIG_B).await?;
        if confirm_val != gain_val {
            debug!("gain bad: expected {} got {}", gain_val, confirm_val);
            return Err(HMC5883Error::Configuration);
        }
        self.gain = gain;
        Ok(())
    }

    pub async fn set_sample_rate(
        &mut self,
        rate: SampleAvgSetting,
    ) -> Result<(), HMC5883Error<CommE>> {
        let current_conf =
            self.generate_a_config(self.temp_enabled, rate, self.odr, self.measurement_mode);
        self.write_reg(HMC5883L_CONFIG::REG_CONFIG_A, current_conf)
            .await?;

        let confirm_val = self.read_reg(HMC5883L_CONFIG::REG_CONFIG_A).await?;
        if confirm_val != current_conf {
            debug!(
                "sample rate bad: expected {} got {}",
                current_conf, confirm_val
            );
            return Err(HMC5883Error::Configuration);
        }
        self.averaging = rate;
        Ok(())
    }

    pub async fn set_output_rate(&mut self, odr: OdrSetting) -> Result<(), HMC5883Error<CommE>> {
        let current_conf = self.generate_a_config(
            self.temp_enabled,
            self.averaging,
            odr,
            self.measurement_mode,
        );
        self.write_reg(HMC5883L_CONFIG::REG_CONFIG_A, current_conf)
            .await?;

        let confirm_val = self.read_reg(HMC5883L_CONFIG::REG_CONFIG_A).await?;
        if confirm_val != current_conf {
            debug!(
                "Data Output Rate bad: expected {} got {}",
                current_conf, confirm_val
            );
            return Err(HMC5883Error::Configuration);
        }
        self.odr = odr;
        Ok(())
    }

    /// Set measurement mode, single-measurement or continuous-measurement
    pub async fn set_measurement_mode(
        &mut self,
        measurement_mode: MeasurementModeSetting,
    ) -> Result<(), HMC5883Error<CommE>> {
        let current_conf = self.generate_a_config(
            self.temp_enabled,
            self.averaging,
            self.odr,
            measurement_mode,
        );
        self.write_reg(HMC5883L_CONFIG::REG_MODE, current_conf)
            .await?;

        let confirm_val = self.read_reg(HMC5883L_CONFIG::REG_MODE).await?;
        if confirm_val != current_conf {
            debug!("gain bad: expected {} got {}", current_conf, confirm_val);
            return Err(HMC5883Error::Configuration);
        }
        self.measurement_mode = measurement_mode;
        Ok(())
    }

    fn generate_a_config(
        &self,
        temp_enabled: bool,
        averaging: SampleAvgSetting,
        odr: OdrSetting,
        mode: MeasurementModeSetting,
    ) -> u8 {
        let reg = (if temp_enabled { 1 << 7 } else { 0 })
            & ((averaging as u8) << 6)
            & ((odr as u8) << 4)
            & ((mode as u8) << 2);

        reg
    }

    /// Read a single register
    async fn read_reg(&mut self, reg: u8) -> Result<u8, HMC5883Error<CommE>> {
        let mut buf = [0u8; 1];
        self.read_block(reg, &mut buf).await?;

        Ok(buf[0])
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

    /// Verify that a magnetometer reading is within the expected range.
    fn reading_in_range(sample: &Vector3<i16>) -> bool {
        /// Maximum Dynamic Range for X and Y axes (micro Teslas)
        const MDR_XY_AXES: i16 = 1600;
        /// Maximum Dynamic Range for Z axis (micro Teslas)
        const MDR_Z_AXIS: i16 = 2500;
        /// Resolution (micro Teslas per LSB)
        const RESO_PER_BIT: f32 = 0.3;
        const MAX_VAL_XY: i16 = (((MDR_XY_AXES as f32) / RESO_PER_BIT) as i16) + 1;
        const MAX_VAL_Z: i16 = (((MDR_Z_AXIS as f32) / RESO_PER_BIT) as i16) + 1;

        sample.x.abs() < MAX_VAL_XY && sample.y.abs() < MAX_VAL_XY && sample.z.abs() < MAX_VAL_Z
    }

    /// Combine high and low bytes of i16 mag value
    fn raw_reading_to_i16(buf: &[u8], idx: usize) -> i16 {
        let val: i16 = (buf[idx] as i16) << 8 | buf[idx + 1] as i16;
        val
    }
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
#[derive(Debug, Clone, Copy)]
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
#[derive(Debug, Clone, Copy)]
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
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum SampleAvgSetting {
    AvgSamples1 = 0b00,
    AvgSamples2 = 0b01,
    AvgSamples4 = 0b10,
    /// Average 8 samples
    AvgSamples8 = 0b11,
}

/// Measurement mode settings
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum MeasurementModeSetting {
    NormalMode = 0b00,
    /// Positive bias current
    PositiveBias = 0b01,
    /// Negative bias current
    NegativeBias = 0b10,
}

/// Operating Mode settings
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum OperatingModeSetting {
    /// Continuous-Measurement Mode
    ContinuousMeasurement = 0b00,
    /// Single-Measurement Mode (Default)
    SingleMeasurement = 0b01,
    /// Idle mode
    Idle1 = 0b10,
}
