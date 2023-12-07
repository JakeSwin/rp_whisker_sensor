//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates a USB serial port that echos.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Pointer;
use defmt::{info, panic};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::{bind_interrupts, i2c};
use embassy_rp::peripherals::{USB, I2C1};
use embassy_rp::usb::{Driver};
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use embedded_hal_async::i2c::I2c;
use heapless::String;
use core::str;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_26;
    let scl = p.PIN_27;

    info!("set up i2c");
    let mut i2c = i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c::Config::default());

    let mut buff = [0u8; 7];
    let mut out = [0u8; 7 * 2];

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        class.wait_connection().await;
        info!("Connected");
        let mut status_buff = [0u8; 1];
        let mut status_out = [0u8; 2];
        // Send EX command
        i2c.write_read(0x18u8, &[0x80], &mut status_buff).await.unwrap();
        hex::encode_to_slice(status_buff, &mut status_out).ok();
        class.write_packet(&status_out).await.unwrap();
        class.write_packet(b"\n").await.unwrap();
        Timer::after(Duration::from_millis(100)).await;
        // Send RT command
        i2c.write_read(0x18u8, &[0xF0], &mut status_buff).await.unwrap();
        hex::encode_to_slice(status_buff, &mut status_out).ok();
        class.write_packet(&status_out).await.unwrap();
        class.write_packet(b"\n").await.unwrap();
        Timer::after(Duration::from_millis(100)).await;
        // Send Start Burst Mode
        i2c.write_read(0x18u8, &[0x1F], &mut status_buff).await.unwrap();
        hex::encode_to_slice(status_buff, &mut status_out).ok();
        class.write_packet(&status_out).await.unwrap();
        class.write_packet(b"\n").await.unwrap();
        Timer::after(Duration::from_millis(500)).await;

        let start_time = Instant::now();
        let mut last_sample = start_time;

        loop {
            i2c.write_read(0x18u8, &[0x4E], &mut buff).await.unwrap();
            let current_time = Instant::now();
            let mut hz_buff = [0u8; 10];
            let hz = 1_000_000 / current_time.duration_since(last_sample).as_micros();
            last_sample = current_time;
            // Timer::after(Duration::from_millis(100)).await;
            hex::encode_to_slice(buff, &mut out).ok();
            // let mut message: String<4> = String::new();
            // message.push(hz);
            class.write_packet(&out).await.unwrap();
            class.write_packet(b" - ").await.unwrap();
            class.write_packet(base_10_bytes(hz, &mut hz_buff)).await.unwrap();
            class.write_packet(b"\n").await.unwrap();
            // let _ = echo(&mut class, &out).await;
            // info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

fn base_10_bytes(mut n: u64, buf: &mut [u8]) -> &[u8] {
    if n == 0 {
        return b"0";
    }
    let mut i = 0;
    while n > 0 {
        buf[i] = (n % 10) as u8 + b'0';
        n /= 10;
        i += 1;
    }
    let slice = &mut buf[..i];
    slice.reverse();
    &*slice
}

// async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>, msg: &[u8]) -> Result<(), Disconnected> {
//     class.write_packet(msg).await?;
//     class.write_packet(b"\n").await?;
// }
