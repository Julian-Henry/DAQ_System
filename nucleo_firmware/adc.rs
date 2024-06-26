#![no_std]
#![no_main]

//import libraries
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::{
    bxcan::{filter::Mask32, Fifo, Frame, StandardId},
    Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler,
};
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::CAN1;
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals};

use embassy_time::{Duration, Timer};
use embedded_hal::spi;
use {defmt_rtt as _, panic_probe as _};

//bind interrupts for CAN bus communication
bind_interrupts!(struct Irqs {
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => Rx1InterruptHandler<CAN1>;
    CAN1_SCE => SceInterruptHandler<CAN1>;
    CAN1_TX => TxInterruptHandler<CAN1>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
});

// spi words
const BIT_MASK_SPI_CMD_READ: u8 = 0x80;
const ACC_REG_DATA_START: u8 = 0x02;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    //create objects for the led and CAN bus
    let mut led = Output::new(p.PB7, Level::High, Speed::Low);
    let mut can = Can::new(p.CAN1, p.PD0, p.PD1, Irqs);

    //setup can
    can.as_mut()
        .modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    can.as_mut().modify_config().leave_disabled();

    can.set_bitrate(100_000);

    can.enable().await;

    // configure spi
    let mut spi_config = Config::default();
    spi_config.frequency = Hertz(100_000);
    spi_config.mode = spi::MODE_3;
    let mut spi = Spi::new(p.SPI1, p.PA5, p.PB5, p.PA6, NoDma, NoDma, spi_config);

    // set digital out pins for microcontroller
    let mut accel_active = Output::new(p.PD15, Level::High, Speed::Low);

    //initialize data buffer for SPI values
    let mut data_buffer: [u8; 3] = [0; 3];

    // define communication commands
    let read_request: [u8; 1] = [0x42];
    let read_control: [u8; 1] = [0x08 | 0x40];
    let config_0: [u8; 3] = [0x19, 0x00, 0x78];
    let reset: [u8; 8] = [0xFF; 8];

    //reset the ADC
    accel_active.set_low();
    spi.blocking_write(&reset).unwrap();
    accel_active.set_high();

    //wait 1 sec
    Timer::after(Duration::from_millis(1000)).await;

    //configure ADC to use Analog Vdd as reference
    accel_active.set_low();
    spi.blocking_write(&config_0).unwrap();
    accel_active.set_high();

    //wait 1 sec
    Timer::after(Duration::from_millis(1000)).await;

    //read setting register
    accel_active.set_low();
    spi.blocking_write(&read_control).unwrap();
    spi.blocking_read(&mut data_buffer).unwrap();
    accel_active.set_high();

    // info!("Accel: {:?}", data_buffer);

    loop {
        //blink led
        led.toggle();

        // read ADC data over spi
        accel_active.set_low();
        spi.blocking_write(&read_request).unwrap();
        spi.blocking_read(&mut data_buffer).unwrap();
        accel_active.set_high();

        // write ADC data to console
        info!("Accel: {:?}", data_buffer);

        // transmit ADC data over can
        let tx_frame = Frame::new_data(unwrap!(StandardId::new(123)), data_buffer);
        can.write(&tx_frame).await;
        Timer::after(Duration::from_millis(1000)).await;
    }
}
