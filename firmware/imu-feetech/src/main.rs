//! PDB IMU board (R2f): an LSM6DSV16X whose on-chip sensor fusion (SFLP) is served on the
//! servo bus as if it were one more Feetech servo, ID 200 — so the host reads orientation in
//! the same `SYNC_READ` as the 15 joints. Protocol and register layout:
//! `hardware/rev-a/electronics/imu_to_dxl-spec-feetech.md`; the host side is
//! `duck-control/src/feetech.rs`.
//!
//! Clocks: 64 MHz from HSI16 via the PLL.
//!
//! Board wiring (STM32G031F8P6, TSSOP-20):
//!   USART2  PA2 TX → U4 (74LVC1G126) → DATA_IMU,  PA3 RX ← DATA_IMU,  PA1 = U4 OE.
//!           PA1 is USART2's hardware DE: the peripheral raises it for exactly the length of
//!           a transmission, so the bus is driven only while this board is talking.
//!           PA3 sits on the same net as U4's output, so every byte sent is heard back.
//!   SPI1    PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS (R8 pull-up) → U2 LSM6DSV16X.
//!
//! The block served at 56 (15 bytes):
//!   0..6   gyro x/y/z, i16 LE raw counts at ±500 dps
//!   6..12  SFLP game rotation x/y/z, IEEE binary16 LE, straight from the FIFO; all zero until
//!          the first fusion output (the host's "not ready" convention)
//!   12     sample counter, +1 per fusion output
//!   13     flags: bit0 sensor configured, bit1 orientation valid
//!   14     reserved, 0

#![no_std]
#![no_main]

mod proto;

use core::cell::Cell;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, BufferedUart};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use panic_halt as _;
use static_cell::StaticCell;

use proto::{BLOCK_LEN, DIAG_LEN, MAX_FRAME, Parser, Slave};

bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

/// The bytes the host reads at 56. Written by the IMU loop, read by the bus task.
static BLOCK: Mutex<CriticalSectionRawMutex, Cell<[u8; BLOCK_LEN]>> =
    Mutex::new(Cell::new([0; BLOCK_LEN]));

/// The bus task's counters, readable at `proto::DIAG_ADDR` (all u16 LE, wrapping):
/// 0..2 UART receive errors (overrun/framing/noise), 2..4 SYNC_READ turns abandoned because a
/// device ahead never answered, 4..6 replies sent, 6..8 frames parsed.
static DIAG: Mutex<CriticalSectionRawMutex, Cell<[u8; DIAG_LEN]>> =
    Mutex::new(Cell::new([0; DIAG_LEN]));

fn bump(slot: usize) {
    DIAG.lock(|d| {
        let mut v = d.get();
        let n = u16::from_le_bytes([v[slot * 2], v[slot * 2 + 1]]).wrapping_add(1);
        v[slot * 2..slot * 2 + 2].copy_from_slice(&n.to_le_bytes());
        d.set(v);
    });
}
const RX_ERRORS: usize = 0;
const EXPIRED: usize = 1;
const REPLIES: usize = 2;
const FRAMES: usize = 3;

const FLAG_CONFIGURED: u8 = 1 << 0;
const FLAG_ORIENTATION: u8 = 1 << 1;

// ── LSM6DSV16X ───────────────────────────────────────────────────────────────

mod reg {
    pub const FUNC_CFG_ACCESS: u8 = 0x01;
    pub const FIFO_CTRL4: u8 = 0x0A;
    pub const WHO_AM_I: u8 = 0x0F;
    pub const CTRL1: u8 = 0x10;
    pub const CTRL2: u8 = 0x11;
    pub const CTRL3: u8 = 0x12;
    pub const CTRL6: u8 = 0x15;
    pub const CTRL8: u8 = 0x17;
    pub const FIFO_STATUS1: u8 = 0x1B;
    pub const OUTX_L_G: u8 = 0x22;
    pub const FIFO_DATA_OUT_TAG: u8 = 0x78;
    // Embedded-functions page (FUNC_CFG_ACCESS.emb_func_reg_access = 1).
    pub const EMB_FUNC_EN_A: u8 = 0x04;
    pub const EMB_FUNC_FIFO_EN_A: u8 = 0x44;
    pub const SFLP_ODR: u8 = 0x5E;
    pub const EMB_FUNC_INIT_A: u8 = 0x66;
}

const WHO_AM_I_VALUE: u8 = 0x70;
/// FIFO tag of an SFLP game-rotation-vector word.
const TAG_GAME_ROTATION: u8 = 0x13;

struct Imu {
    spi: Spi<'static, Blocking, spi::mode::Master>,
    cs: Output<'static>,
}

impl Imu {
    fn write(&mut self, reg: u8, val: u8) {
        self.cs.set_low();
        let _ = self.spi.blocking_write(&[reg & 0x7F, val]);
        self.cs.set_high();
    }

    fn read(&mut self, reg: u8, buf: &mut [u8]) {
        self.cs.set_low();
        let _ = self.spi.blocking_write(&[reg | 0x80]);
        let _ = self.spi.blocking_read(buf);
        self.cs.set_high();
    }

    fn read1(&mut self, reg: u8) -> u8 {
        let mut b = [0u8];
        self.read(reg, &mut b);
        b[0]
    }

    fn modify(&mut self, reg: u8, clear: u8, set: u8) {
        let v = self.read1(reg);
        self.write(reg, (v & !clear) | set);
    }

    /// Reset and configure: ±4 g, ±500 dps (the host's GYRO_RAD_PER_LSB assumes it), both at
    /// 120 Hz high-performance, SFLP game rotation at 120 Hz batched to a streaming FIFO.
    /// The same sequence as ST's `lsm6dsv16x_sensor_fusion` example.
    async fn configure(&mut self) -> bool {
        if self.read1(reg::WHO_AM_I) != WHO_AM_I_VALUE {
            return false;
        }
        self.write(reg::CTRL3, 0x01); // SW_RESET
        for _ in 0..50 {
            Timer::after_millis(1).await;
            if self.read1(reg::CTRL3) & 0x01 == 0 {
                break;
            }
        }
        Timer::after_millis(5).await;
        self.write(reg::CTRL3, 0x44); // BDU | IF_INC
        self.write(reg::CTRL8, 0x01); // FS_XL = ±4 g
        self.write(reg::CTRL6, 0x02); // FS_G = ±500 dps

        self.write(reg::FUNC_CFG_ACCESS, 0x80);
        self.modify(reg::EMB_FUNC_FIFO_EN_A, 0, 0x02); // SFLP_GAME_FIFO_EN
        self.modify(reg::SFLP_ODR, 0x38, 3 << 3); // SFLP_GAME_ODR = 120 Hz
        self.modify(reg::EMB_FUNC_EN_A, 0, 0x02); // SFLP_GAME_EN
        self.write(reg::EMB_FUNC_INIT_A, 0x02); // SFLP_GAME_INIT
        self.write(reg::FUNC_CFG_ACCESS, 0x00);

        self.write(reg::FIFO_CTRL4, 0x06); // continuous (stream) mode
        self.write(reg::CTRL1, 0x06); // ODR_XL = 120 Hz, high performance
        self.write(reg::CTRL2, 0x06); // ODR_G = 120 Hz, high performance
        self.read1(reg::WHO_AM_I) == WHO_AM_I_VALUE
    }
}

/// Configure the sensor, then keep the block fresh: one update per fusion output.
async fn imu_loop(mut imu: Imu, mut wdg: IndependentWatchdog<'static, peripherals::IWDG>) -> ! {
    let mut counter: u8 = 0;
    loop {
        wdg.pet();
        BLOCK.lock(|b| b.set([0; BLOCK_LEN]));
        if !imu.configure().await {
            Timer::after_millis(50).await;
            continue;
        }
        BLOCK.lock(|b| {
            let mut v = b.get();
            v[13] = FLAG_CONFIGURED;
            b.set(v);
        });

        let mut quat: Option<[u8; 6]> = None;
        let mut last_sample = Instant::now();
        let mut last_check = Instant::now();
        loop {
            wdg.pet();
            Timer::after_millis(4).await;

            let mut status = [0u8; 2];
            imu.read(reg::FIFO_STATUS1, &mut status);
            let words = status[0] as u16 | ((status[1] as u16 & 0x01) << 8);
            let mut fresh = false;
            for _ in 0..words.min(16) {
                let mut w = [0u8; 7];
                imu.read(reg::FIFO_DATA_OUT_TAG, &mut w);
                if w[0] >> 3 == TAG_GAME_ROTATION {
                    let mut q = [0u8; 6];
                    q.copy_from_slice(&w[1..7]);
                    quat = Some(q);
                    fresh = true;
                }
            }

            if fresh {
                last_sample = Instant::now();
                let mut gyro = [0u8; 6];
                imu.read(reg::OUTX_L_G, &mut gyro);
                counter = counter.wrapping_add(1);
                let mut block = [0u8; BLOCK_LEN];
                block[0..6].copy_from_slice(&gyro);
                if let Some(q) = quat {
                    block[6..12].copy_from_slice(&q);
                }
                block[12] = counter;
                block[13] = FLAG_CONFIGURED | if quat.is_some() { FLAG_ORIENTATION } else { 0 };
                BLOCK.lock(|b| b.set(block));
            }

            // A sensor that has stopped producing, or stopped answering, is reset rather than
            // left serving its last sample: the host reads a frozen block as a frozen robot.
            let silent = last_sample.elapsed() > Duration::from_millis(250);
            let lost = last_check.elapsed() > Duration::from_secs(1) && {
                last_check = Instant::now();
                imu.read1(reg::WHO_AM_I) != WHO_AM_I_VALUE
            };
            if silent || lost {
                break;
            }
        }
    }
}

// ── the bus ──────────────────────────────────────────────────────────────────

/// Drop whatever arrives until the line has been quiet for a little while: the echo of our
/// own reply (PA3 hears U4's output), and anything after it that this board has no use for.
async fn drain(uart: &mut BufferedUart<'static>) {
    let mut sink = [0u8; 32];
    while let Ok(Ok(_)) = with_timeout(Duration::from_micros(150), uart.read(&mut sink)).await {}
}

#[embassy_executor::task]
async fn bus_task(mut uart: BufferedUart<'static>) {
    let mut parser = Parser::new();
    let mut slave = Slave::new();
    let mut rx = [0u8; 64];
    let mut out = [0u8; MAX_FRAME];
    loop {
        let n = if slave.waiting() {
            // Queued behind another device in a SYNC_READ: if it never answers, give up.
            match with_timeout(Duration::from_millis(5), uart.read(&mut rx)).await {
                Ok(Ok(n)) => n,
                Ok(Err(_)) => {
                    bump(RX_ERRORS);
                    parser.reset();
                    continue;
                }
                Err(_) => {
                    bump(EXPIRED);
                    slave.expire();
                    continue;
                }
            }
        } else {
            match uart.read(&mut rx).await {
                Ok(n) => n,
                Err(_) => {
                    // Overrun or framing error: resynchronise on the next header.
                    bump(RX_ERRORS);
                    parser.reset();
                    continue;
                }
            }
        };

        for &b in &rx[..n] {
            let Some(frame) = parser.feed(b) else { continue };
            bump(FRAMES);
            let action = slave.on_frame(&frame);
            let len = proto::reply(action, &BLOCK.lock(|c| c.get()), &DIAG.lock(|c| c.get()), &mut out);
            if len > 0 {
                bump(REPLIES);
                let _ = uart.write_all(&out[..len]).await;
                let _ = uart.flush().await;
                drain(&mut uart).await;
                parser.reset();
                break;
            }
        }
    }
}

// ── start-up ─────────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 64 MHz from HSI16 through the PLL (×8 ÷2). USART2 runs from PCLK, so 500 kbit/s is an
    // exact divide by 128. Not for throughput: at 16 MHz the reply started ~60 µs later than a
    // servo's, enough for the next servo in a SYNC_READ to talk over its end.
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL8,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
    }
    let p = embassy_stm32::init(config);

    let mut wdg = IndependentWatchdog::new(p.IWDG, 1_000_000);
    wdg.unleash();

    static TX: StaticCell<[u8; 64]> = StaticCell::new();
    static RX: StaticCell<[u8; 256]> = StaticCell::new();
    let mut ucfg = usart::Config::default();
    ucfg.baudrate = proto_baud();
    let uart = BufferedUart::new_with_de(
        p.USART2,
        p.PA3,
        p.PA2,
        p.PA1,
        Irqs,
        TX.init([0; 64]),
        RX.init([0; 256]),
        ucfg,
    )
    .unwrap();
    spawner.spawn(bus_task(uart).unwrap());

    let mut scfg = spi::Config::default();
    scfg.mode = spi::MODE_3;
    scfg.frequency = Hertz(4_000_000);
    let spi = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, scfg);
    let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    // The sensor needs ~10 ms after power-up before it answers.
    Timer::after_millis(20).await;
    imu_loop(Imu { spi, cs }, wdg).await
}

const fn proto_baud() -> u32 {
    500_000
}
