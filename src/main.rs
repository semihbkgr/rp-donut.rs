//! Render a spinning donut on an SSD1306 OLED screen using a Pico board.
#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::{clocks::init_clocks_and_plls, sio::Sio, watchdog::Watchdog};
use defmt_rtt as _;
use hal::fugit::RateExtU32;
use hal::{
    gpio::{FunctionI2C, Pin},
    pac,
};
use libm::{cosf, sinf};
use panic_probe as _;
use rp2040_hal as hal;
use rp_pico as bsp;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const TRIG_LOOKUP_TABLE_SIZE: usize = 628;
const TABLE_STEP: f32 = 0.01;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio18.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio19.reconfigure();

    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let (sin_table, cos_table) = generate_trig_lookup_tables();

    let mut buf = [0x00u8; 1024];
    let mut a = 0.0;
    let mut b = 0.0;

    loop {
        render_donut_frame(&mut buf, a, b, &sin_table, &cos_table);
        display.draw(&buf).unwrap();
        a += 0.04;
        b += 0.02;
    }
}

fn generate_trig_lookup_tables() -> ([f32; TRIG_LOOKUP_TABLE_SIZE], [f32; TRIG_LOOKUP_TABLE_SIZE]) {
    let mut sin_table = [0.0f32; TRIG_LOOKUP_TABLE_SIZE];
    let mut cos_table = [0.0f32; TRIG_LOOKUP_TABLE_SIZE];

    for i in 0..TRIG_LOOKUP_TABLE_SIZE {
        let angle = i as f32 * TABLE_STEP;
        sin_table[i] = sinf(angle);
        cos_table[i] = cosf(angle);
    }

    (sin_table, cos_table)
}

const R1: f32 = 0.7;
const R2: f32 = 1.7;
const K2: f32 = 5.0;
const K1: f32 = 90.0;

fn render_donut_frame(buf: &mut [u8], a: f32, b: f32, sin_table: &[f32], cos_table: &[f32]) {
    buf.fill(0);

    let cos_a = cosf(a);
    let sin_a = sinf(a);
    let cos_b = cosf(b);
    let sin_b = sinf(b);

    for theta in (0..628).step_by(20) {
        for phi in (0..628).step_by(10) {
            let cos_theta = cos_table[theta % TRIG_LOOKUP_TABLE_SIZE];
            let sin_theta = sin_table[theta % TRIG_LOOKUP_TABLE_SIZE];
            let cos_phi = cos_table[phi % TRIG_LOOKUP_TABLE_SIZE];
            let sin_phi = sin_table[phi % TRIG_LOOKUP_TABLE_SIZE];

            let circle_x = R2 + R1 * cos_theta;
            let circle_y = R1 * sin_theta;

            let x =
                circle_x * (cos_b * cos_phi + sin_a * sin_b * sin_phi) - circle_y * cos_a * sin_b;
            let y =
                circle_x * (sin_b * cos_phi - sin_a * cos_b * sin_phi) + circle_y * cos_a * cos_b;
            let z = K2 + cos_a * circle_x * sin_phi + circle_y * sin_a;

            let ooz = 1.0 / z;
            let xp = ((64.0 + K1 * ooz * x) as usize).min(127);
            let yp = ((32.0 - K1 * ooz * y) as usize).min(63);

            set_pixel(buf, xp, yp);
        }
    }
}

fn set_pixel(buf: &mut [u8], x: usize, y: usize) {
    if x < 128 && y < 64 {
        let byte_index = x + (y / 8) * 128;
        let bit_index = y % 8;
        buf[byte_index] |= 1 << bit_index;
    }
}
