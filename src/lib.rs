#![no_std]

mod bits;

pub(crate) mod reg_data;

pub mod hmc5883;
pub mod mpu6050;

// pub struct GY86<I, D> {
//     mpu6050: Mpu6050<I, D>,
//     hmc5883: HMC5983<I>,
// }

// impl<I, D> GY86<I, D>
// where
//     D: DelayNs,
//     E: core::fmt::Debug,
// {
//     pub fn new(i2c_manager: BusManagerCortexM<I2c>, delay: D) -> Self {
//         let mpu = Mpu6050::new(i2c_manager.acquire_i2c(), delay);
//         let hmc = HMC5983::new(i2c_manager.acquire_i2c());
//         Self {
//             mpu6050: mpu,
//             hmc5883: hmc,
//         }
//     }
// }

// pub struct OutputData {}
