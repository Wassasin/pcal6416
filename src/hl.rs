mod eh_async;

use core::{borrow::BorrowMut, marker::PhantomData, mem::ManuallyDrop};

use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex, RawMutex},
    channel::{Channel, Receiver, Sender},
    mutex::Mutex,
};

use crate::*;

type PcalMutex<T, M> = Mutex<M, ll::Device<ll::Interface<T>>>;
type PcalReference<'a, T, M> = &'a PcalMutex<T, M>;

type InterruptChannel<M> = Channel<M, bool, 1>;
type InterruptReceiver<'a, M> = Receiver<'a, M, bool, 1>;
type InterruptSender<'a, M> = Sender<'a, M, bool, 1>;

pub struct Pcal6416<T, M: RawMutex = NoopRawMutex> {
    inner: PcalMutex<T, M>,
    interrupt_channels: [InterruptChannel<M>; 16],
}

pub struct Input<T>(PhantomData<T>);
pub struct Output;

pub struct PullUp;
pub struct PullDown;
pub struct Floating;

pub struct Pin<'a, DIR, T, M: RawMutex> {
    port_driver: PcalReference<'a, T, M>,
    interrupt_receiver: InterruptReceiver<'a, M>,
    idx: u8,
    _dir: PhantomData<DIR>,
}

fn idx_to_mask_be(idx: u16) -> u16 {
    // Note(rotate_left): big endian
    (1u16 << idx).rotate_left(8)
}

impl<'a, DIR, T, M: RawMutex> Pin<'a, DIR, T, M> {
    fn new(
        port_driver: PcalReference<'a, T, M>,
        interrupt_receiver: InterruptReceiver<'a, M>,
        idx: u8,
    ) -> Self {
        Self {
            port_driver,
            interrupt_receiver,
            idx,
            _dir: PhantomData,
        }
    }

    fn mask_be(&self) -> u16 {
        idx_to_mask_be(self.idx as u16)
    }

    fn set_bit(&self, value: u16, bit: bool) -> u16 {
        let value_mask = if bit { self.mask_be() } else { 0u16 };
        value & !self.mask_be() | value_mask
    }

    /// Transform the current pin into one with a different direction.
    fn transform<NEWDIR>(self) -> Pin<'a, NEWDIR, T, M> {
        Pin {
            port_driver: self.port_driver,
            interrupt_receiver: self.interrupt_receiver,
            idx: self.idx,
            _dir: PhantomData,
        }
    }
}

pub struct InterruptPin<'a, PULLUPDOWN, T, M: RawMutex>(
    ManuallyDrop<Pin<'a, Input<PULLUPDOWN>, T, M>>,
);

impl<'a, PULLUPDOWN, T, M: RawMutex> InterruptPin<'a, PULLUPDOWN, T, M> {
    fn new(pin: Pin<'a, Input<PULLUPDOWN>, T, M>) -> Self {
        Self(ManuallyDrop::new(pin))
    }
}

impl<PULLUPDOWN, T, M: RawMutex> Drop for InterruptPin<'_, PULLUPDOWN, T, M> {
    fn drop(&mut self) {
        panic!("Please call deconfigure_interrupt before dropping this pin");
    }
}

#[derive(Debug)]
pub struct DigitalError<E: embedded_hal::i2c::Error>(E);

impl<E: embedded_hal::i2c::Error> embedded_hal::digital::Error for DigitalError<E> {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

impl<E: embedded_hal::i2c::Error> From<E> for DigitalError<E> {
    fn from(value: E) -> Self {
        Self(value)
    }
}

pub trait OutputPin {
    type Error;

    #[allow(async_fn_in_trait)]
    async fn set_value(&mut self, value: bool) -> Result<(), Self::Error>;
}

pub struct InterruptHandler<'a, T, M: RawMutex> {
    port_driver: PcalReference<'a, T, M>,
    interrupt_channels: [InterruptSender<'a, M>; 16],
}

pub struct Pins<'a, T, M: RawMutex> {
    pub p0_0: Pin<'a, Input<Floating>, T, M>,
    pub p0_1: Pin<'a, Input<Floating>, T, M>,
    pub p0_2: Pin<'a, Input<Floating>, T, M>,
    pub p0_3: Pin<'a, Input<Floating>, T, M>,
    pub p0_4: Pin<'a, Input<Floating>, T, M>,
    pub p0_5: Pin<'a, Input<Floating>, T, M>,
    pub p0_6: Pin<'a, Input<Floating>, T, M>,
    pub p0_7: Pin<'a, Input<Floating>, T, M>,
    pub p1_0: Pin<'a, Input<Floating>, T, M>,
    pub p1_1: Pin<'a, Input<Floating>, T, M>,
    pub p1_2: Pin<'a, Input<Floating>, T, M>,
    pub p1_3: Pin<'a, Input<Floating>, T, M>,
    pub p1_4: Pin<'a, Input<Floating>, T, M>,
    pub p1_5: Pin<'a, Input<Floating>, T, M>,
    pub p1_6: Pin<'a, Input<Floating>, T, M>,
    pub p1_7: Pin<'a, Input<Floating>, T, M>,
}

pub struct Parts<'a, T, M: RawMutex> {
    pub pins: Pins<'a, T, M>,
    pub interrupt_handler: InterruptHandler<'a, T, M>,
}

impl<T, M: RawMutex> Pcal6416<T, M> {
    pub fn new(i2c: T, address: Address) -> Self {
        Self {
            inner: Mutex::new(ll::Device::new(ll::Interface::new(i2c, address))),
            interrupt_channels: [const { InterruptChannel::new() }; 16],
        }
    }

    pub fn split(&mut self) -> Parts<'_, T, M> {
        macro_rules! instance_pin {
            ( $idx:expr ) => {
                Pin::new(
                    &self.inner,
                    self.interrupt_channels[$idx as usize].receiver(),
                    $idx,
                )
            };
        }

        Parts {
            pins: Pins {
                p0_0: instance_pin!(0),
                p0_1: instance_pin!(1),
                p0_2: instance_pin!(2),
                p0_3: instance_pin!(3),
                p0_4: instance_pin!(4),
                p0_5: instance_pin!(5),
                p0_6: instance_pin!(6),
                p0_7: instance_pin!(7),
                p1_0: instance_pin!(8),
                p1_1: instance_pin!(9),
                p1_2: instance_pin!(10),
                p1_3: instance_pin!(11),
                p1_4: instance_pin!(12),
                p1_5: instance_pin!(13),
                p1_6: instance_pin!(14),
                p1_7: instance_pin!(15),
            },
            interrupt_handler: InterruptHandler {
                port_driver: &self.inner,
                interrupt_channels: self.interrupt_channels.each_ref().map(|c| c.sender()),
            },
        }
    }
}
