use core::{borrow::BorrowMut, marker::PhantomData, mem::ManuallyDrop};

use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Receiver, Sender},
    mutex::Mutex,
};
use embedded_hal_async::i2c::I2c;

use crate::*;

type PcalMutex<T> = Mutex<NoopRawMutex, ll::Device<ll::Interface<T>>>;
type PcalReference<'a, T> = &'a PcalMutex<T>;

type InterruptChannel = Channel<NoopRawMutex, bool, 1>;
type InterruptReceiver<'a> = Receiver<'a, NoopRawMutex, bool, 1>;
type InterruptSender<'a> = Sender<'a, NoopRawMutex, bool, 1>;

pub struct Pcal6416<T> {
    inner: PcalMutex<T>,
    interrupt_channels: [InterruptChannel; 16],
}

pub struct Input<T>(PhantomData<T>);
pub struct Output;

pub struct PullUp;
pub struct PullDown;
pub struct Floating;

pub trait PinMarker {
    fn pin_idx() -> u8;
}

pub struct Pin<'a, DIR, MARKER: PinMarker, T> {
    port_driver: PcalReference<'a, T>,
    interrupt_receiver: InterruptReceiver<'a>,
    _dir: PhantomData<DIR>,
    _marker: PhantomData<MARKER>,
}

fn idx_to_mask_be(idx: u16) -> u16 {
    // Note(rotate_left): big endian
    (1u16 << idx).rotate_left(8)
}

impl<'a, DIR, MARKER: PinMarker, T, E> Pin<'a, DIR, MARKER, T>
where
    T: I2c<Error = E>,
{
    fn new(port_driver: PcalReference<'a, T>, interrupt_receiver: InterruptReceiver<'a>) -> Self {
        Self {
            port_driver,
            interrupt_receiver,
            _dir: Default::default(),
            _marker: Default::default(),
        }
    }

    fn mask_be(&self) -> u16 {
        idx_to_mask_be(MARKER::pin_idx() as u16)
    }

    fn set_bit(&self, value: u16, bit: bool) -> u16 {
        let value_mask = if bit { self.mask_be() } else { 0u16 };
        value & !self.mask_be() | value_mask
    }

    pub async fn into_input(self) -> Result<Pin<'a, Input<Floating>, MARKER, T>, E> {
        let mut pcal = self.port_driver.lock().await;

        // Set to floating.
        pcal.pull_up_down_enable()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), false)))
            .await?;

        pcal.borrow_mut()
            .configuration()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), true)))
            .await?;

        Ok(Pin {
            port_driver: self.port_driver,
            interrupt_receiver: self.interrupt_receiver,
            _dir: Default::default(),
            _marker: self._marker,
        })
    }

    pub async fn into_output(self, value: bool) -> Result<Pin<'a, Output, MARKER, T>, E> {
        let mut pcal = self.port_driver.lock().await;

        // Set the output value before configuring it as an output

        pcal.borrow_mut()
            .output()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), value)))
            .await?;

        pcal.borrow_mut()
            .configuration()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), false)))
            .await?;

        Ok(Pin {
            port_driver: self.port_driver,
            interrupt_receiver: self.interrupt_receiver,
            _dir: Default::default(),
            _marker: self._marker,
        })
    }
}

impl<'a, PULLUPDOWN, MARKER: PinMarker, T, E> Pin<'a, Input<PULLUPDOWN>, MARKER, T>
where
    T: I2c<Error = E>,
{
    pub async fn set_input_latch(&mut self, value: bool) -> Result<(), E> {
        let mut pcal = self.port_driver.lock().await;
        pcal.input_latch()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), value)))
            .await
    }

    pub async fn is_high(&mut self) -> Result<bool, E> {
        let mut pcal = self.port_driver.lock().await;
        let value = pcal.input().read_async().await?;
        Ok(value.value() & self.mask_be() != 0)
    }

    pub async fn is_low(&mut self) -> Result<bool, E> {
        Ok(!self.is_high().await?)
    }

    pub async fn into_pull_up(self) -> Result<Pin<'a, Input<PullUp>, MARKER, T>, E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.pull_up_down_selection()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), true)))
            .await?;

        pcal.pull_up_down_enable()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), true)))
            .await?;

        Ok(Pin {
            port_driver: self.port_driver,
            interrupt_receiver: self.interrupt_receiver,
            _dir: Default::default(),
            _marker: self._marker,
        })
    }

    pub async fn into_pull_down(self) -> Result<Pin<'a, Input<PullDown>, MARKER, T>, E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.pull_up_down_selection()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), false)))
            .await?;

        pcal.pull_up_down_enable()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), true)))
            .await?;

        Ok(Pin {
            port_driver: self.port_driver,
            interrupt_receiver: self.interrupt_receiver,
            _dir: Default::default(),
            _marker: self._marker,
        })
    }

    pub async fn into_pull_floating(self) -> Result<Pin<'a, Input<Floating>, MARKER, T>, E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.pull_up_down_enable()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), false)))
            .await?;

        Ok(Pin {
            port_driver: self.port_driver,
            interrupt_receiver: self.interrupt_receiver,
            _dir: Default::default(),
            _marker: self._marker,
        })
    }

    async fn configure_interrupt(&mut self, enabled: bool) -> Result<(), E> {
        let mut pcal = self.port_driver.lock().await;
        pcal.input_latch()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), enabled)))
            .await?;
        // A mask is negative, hence invert enabled.
        pcal.interrupt_mask()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), !enabled)))
            .await?;

        Ok(())
    }

    pub async fn into_interrupt(mut self) -> Result<InterruptPin<'a, PULLUPDOWN, MARKER, T>, E> {
        self.configure_interrupt(true).await?;
        Ok(InterruptPin::new(self))
    }

    async fn wait_for_value(&mut self, value: bool) -> Result<(), E> {
        let need_await = value != self.is_high().await?;
        if need_await {
            loop {
                if value == self.interrupt_receiver.receive().await {
                    break;
                }
            }
        }
        Ok(())
    }

    async fn wait_for_edge(&mut self, rising_nfalling: bool) -> Result<(), E> {
        let target_value = rising_nfalling;
        loop {
            // That we got this interrupt, and the new value *is* target_value, means we have an corresponding edge.
            if target_value == self.interrupt_receiver.receive().await {
                break;
            }
        }
        Ok(())
    }
}

pub struct InterruptPin<'a, PULLUPDOWN, MARKER: PinMarker, T>(
    ManuallyDrop<Pin<'a, Input<PULLUPDOWN>, MARKER, T>>,
);

impl<'a, PULLUPDOWN, MARKER: PinMarker, T, E> InterruptPin<'a, PULLUPDOWN, MARKER, T>
where
    T: I2c<Error = E>,
{
    fn new(pin: Pin<'a, Input<PULLUPDOWN>, MARKER, T>) -> Self {
        Self(ManuallyDrop::new(pin))
    }

    pub async fn deconfigure_interrupt(
        mut self,
    ) -> Result<Pin<'a, Input<PULLUPDOWN>, MARKER, T>, E> {
        self.0.configure_interrupt(false).await?;
        // Deconstruct `self` into the inner pin without calling drop on `self`.
        let pin = unsafe { ManuallyDrop::take(&mut self.0) };
        core::mem::forget(self); // Prevent drop from being called on `self`.
        Ok(pin)
    }
}

impl<PULLUPDOWN, MARKER: PinMarker, T> Drop for InterruptPin<'_, PULLUPDOWN, MARKER, T> {
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

impl<PULLUPDOWN, MARKER: PinMarker, T, E> embedded_hal::digital::ErrorType
    for InterruptPin<'_, PULLUPDOWN, MARKER, T>
where
    T: I2c<Error = E>,
    E: embedded_hal::i2c::Error,
{
    type Error = DigitalError<E>;
}

impl<E: embedded_hal::i2c::Error> From<E> for DigitalError<E> {
    fn from(value: E) -> Self {
        Self(value)
    }
}

impl<PULLUPDOWN, MARKER: PinMarker, T, E> embedded_hal_async::digital::Wait
    for InterruptPin<'_, PULLUPDOWN, MARKER, T>
where
    T: I2c<Error = E>,
    E: embedded_hal::i2c::Error,
{
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.0.wait_for_value(true).await?)
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.0.wait_for_value(false).await?)
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Ok(self.0.wait_for_edge(true).await?)
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Ok(self.0.wait_for_edge(false).await?)
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        let _ = self.0.interrupt_receiver.try_receive(); // Throw away any staged interrupts
        let _ = self.0.interrupt_receiver.receive().await;
        Ok(())
    }
}

pub trait OutputPin {
    type Error;

    #[allow(async_fn_in_trait)]
    async fn set_value(&mut self, value: bool) -> Result<(), Self::Error>;
}

impl<MARKER: PinMarker, T, E> OutputPin for Pin<'_, Output, MARKER, T>
where
    T: I2c<Error = E>,
{
    type Error = E;

    async fn set_value(&mut self, value: bool) -> Result<(), E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.output()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), value)))
            .await
    }
}

pub struct InterruptHandler<'a, T> {
    port_driver: PcalReference<'a, T>,
    interrupt_channels: [InterruptSender<'a>; 16],
}

impl<T, E> InterruptHandler<'_, T>
where
    T: I2c<Error = E>,
{
    pub async fn handle(&mut self) -> Result<(), E> {
        let mut pcal = self.port_driver.lock().await;

        let status = pcal.interrupt_status().read_async().await?.value();
        // Clears interrupts
        let input = pcal.input().read_async().await?.value();

        for idx in 0..16u16 {
            let mask_be = idx_to_mask_be(idx);
            if status & mask_be != 0 {
                let input = input & mask_be != 0;
                if self.interrupt_channels[idx as usize]
                    .try_send(input)
                    .is_err()
                {
                    #[cfg(feature = "defmt-1")]
                    defmt::debug!(
                        "Interrupt for {} could not be sent as they previous is still pending",
                        idx
                    );
                }
            }
        }

        Ok(())
    }
}

pub struct Pins<'a, T> {
    pub p0_0: Pin<'a, Input<Floating>, P0_0, T>,
    pub p0_1: Pin<'a, Input<Floating>, P0_1, T>,
    pub p0_2: Pin<'a, Input<Floating>, P0_2, T>,
    pub p0_3: Pin<'a, Input<Floating>, P0_3, T>,
    pub p0_4: Pin<'a, Input<Floating>, P0_4, T>,
    pub p0_5: Pin<'a, Input<Floating>, P0_5, T>,
    pub p0_6: Pin<'a, Input<Floating>, P0_6, T>,
    pub p0_7: Pin<'a, Input<Floating>, P0_7, T>,
    pub p1_0: Pin<'a, Input<Floating>, P1_0, T>,
    pub p1_1: Pin<'a, Input<Floating>, P1_1, T>,
    pub p1_2: Pin<'a, Input<Floating>, P1_2, T>,
    pub p1_3: Pin<'a, Input<Floating>, P1_3, T>,
    pub p1_4: Pin<'a, Input<Floating>, P1_4, T>,
    pub p1_5: Pin<'a, Input<Floating>, P1_5, T>,
    pub p1_6: Pin<'a, Input<Floating>, P1_6, T>,
    pub p1_7: Pin<'a, Input<Floating>, P1_7, T>,
}

pub struct Parts<'a, T> {
    pub pins: Pins<'a, T>,
    pub interrupt_handler: InterruptHandler<'a, T>,
}

impl<T, E> Pcal6416<T>
where
    T: I2c<Error = E>,
{
    pub fn new(i2c: T, address: Address) -> Self {
        Self {
            inner: Mutex::new(ll::Device::new(ll::Interface::new(i2c, address))),
            interrupt_channels: [const { InterruptChannel::new() }; 16],
        }
    }

    pub fn split(&mut self) -> Parts<'_, T> {
        macro_rules! instance_pin {
            ( $name:ident ) => {
                Pin::new(
                    &self.inner,
                    self.interrupt_channels[$name::pin_idx() as usize].receiver(),
                )
            };
        }

        Parts {
            pins: Pins {
                p0_0: instance_pin!(P0_0),
                p0_1: instance_pin!(P0_1),
                p0_2: instance_pin!(P0_2),
                p0_3: instance_pin!(P0_3),
                p0_4: instance_pin!(P0_4),
                p0_5: instance_pin!(P0_5),
                p0_6: instance_pin!(P0_6),
                p0_7: instance_pin!(P0_7),
                p1_0: instance_pin!(P1_0),
                p1_1: instance_pin!(P1_1),
                p1_2: instance_pin!(P1_2),
                p1_3: instance_pin!(P1_3),
                p1_4: instance_pin!(P1_4),
                p1_5: instance_pin!(P1_5),
                p1_6: instance_pin!(P1_6),
                p1_7: instance_pin!(P1_7),
            },
            interrupt_handler: InterruptHandler {
                port_driver: &self.inner,
                interrupt_channels: self.interrupt_channels.each_ref().map(|c| c.sender()),
            },
        }
    }
}

macro_rules! define_pin {
    ( $name:ident, $idx:literal ) => {
        pub struct $name;

        impl PinMarker for $name {
            fn pin_idx() -> u8 {
                $idx
            }
        }
    };
}

define_pin!(P0_0, 0);
define_pin!(P0_1, 1);
define_pin!(P0_2, 2);
define_pin!(P0_3, 3);
define_pin!(P0_4, 4);
define_pin!(P0_5, 5);
define_pin!(P0_6, 6);
define_pin!(P0_7, 7);
define_pin!(P1_0, 8);
define_pin!(P1_1, 9);
define_pin!(P1_2, 10);
define_pin!(P1_3, 11);
define_pin!(P1_4, 12);
define_pin!(P1_5, 13);
define_pin!(P1_6, 14);
define_pin!(P1_7, 15);
