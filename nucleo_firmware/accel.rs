#![no_std]
#![no_main]

//import libraries
use cortex_m::register::apsr::read;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::{
    bxcan::{filter::Mask32, Fifo, Frame, Interrupt, StandardId},
    Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler,
};
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt;
use embassy_stm32::peripherals::CAN1;
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals};

use embassy_time::{Duration, Timer};
use embedded_hal::spi;
use {defmt_rtt as _, panic_probe as _};

//define address of the accelerometer
const ACCEL_ADDR: u8 = 0x1E;

//bind CAN interrupts
bind_interrupts!(struct Irqs {
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => Rx1InterruptHandler<CAN1>;
    CAN1_SCE => SceInterruptHandler<CAN1>;
    CAN1_TX => TxInterruptHandler<CAN1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    //create objects for the led and CAN bus
    let mut led = Output::new(p.PB7, Level::High, Speed::Low);
    let mut can = Can::new(p.CAN1, p.PD0, p.PD1, Irqs);

    //bind interrupts for I2C communication
    bind_interrupts!(struct Irqs2 {
        I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    });

    // setup can
    can.as_mut()
        .modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    can.as_mut().modify_config().leave_disabled();

    can.set_bitrate(100_000);

    can.enable().await;

    //configure i2c
    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs2,
        NoDma,
        NoDma,
        embassy_stm32::time::Hertz(100000),
        Default::default(),
    );
    // set digital out pins for microcontroller
    let mut reset_pin = Output::new(p.PA4, Level::Low, Speed::Low);

    //initialize data buffers
    let mut data_buffer: [u8; 13] = [0; 13];
    let mut accel_vals: [u8; 2] = [0; 2];

    // define communication commands
    let read_xyz_data: [u8; 1] = [0x33];

    //wait 1 sec
    Timer::after(Duration::from_millis(1000)).await;

    reset_pin.set_high();
    reset_pin.set_low();

    //wait 1 sec
    Timer::after(Duration::from_millis(1000)).await;

    //check that I2C protocol is working correctly
    match i2c.blocking_write_read(ACCEL_ADDR, &[0x0D], &mut data_buffer) {
        Ok(()) => info!("Whoami: {}", data_buffer[0]),
        // Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
    }
    info!("Accel: {:?}", data_buffer);

    //place control register in standby mode
    i2c.blocking_write_read(ACCEL_ADDR, &[0x2A, 0x00], &mut data_buffer);
    //set magnometer to 8x oversampling
    i2c.blocking_write_read(ACCEL_ADDR, &[0x5B, 0x1F], &mut data_buffer);
    //enable magnetic reset after each cycle
    i2c.blocking_write_read(ACCEL_ADDR, &[0x5C, 0x20], &mut data_buffer);
    //set accelerometer to a range of +/-4g
    i2c.blocking_write_read(ACCEL_ADDR, &[0x0E, 0x01], &mut data_buffer);
    //enable sampling
    i2c.blocking_write_read(ACCEL_ADDR, &[0x2A, 0x0D], &mut data_buffer);

    loop {
        //blink led
        led.toggle();

        //read accelerometer and magnetic field values
        i2c.blocking_write_read(ACCEL_ADDR, &[0x00], &mut data_buffer);

        //copy acceleration values to accel_vals buffer
        accel_vals[0] = data_buffer[1];
        accel_vals[1] = data_buffer[2];
        /*         accel_vals[2] = data_buffer[3];
        accel_vals[3] = data_buffer[4];
        accel_vals[4] = data_buffer[5];
        accel_vals[5] = data_buffer[6];*/
        info!("Accel: {:?}", accel_vals);

        // transmit accelerometer data over can
        let tx_frame = Frame::new_data(unwrap!(StandardId::new(300)), accel_vals);
        can.write(&tx_frame).await;
        Timer::after(Duration::from_millis(1000)).await;
    }
}
