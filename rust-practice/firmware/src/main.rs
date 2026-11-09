#![no_std]
#![no_main]

mod actuator;
mod adc;
mod board;
mod comms;
mod control_task;
mod shared;

use embassy_executor::Spawner;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_stm32::init(Default::default());
    defmt::info!("robo_arm firmware started");
}
