use core::{borrow::BorrowMut, marker::PhantomData, mem::ManuallyDrop};

use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex, RawMutex},
    channel::{Channel, Receiver, Sender},
    mutex::Mutex,
};
use embedded_hal_async::i2c::I2c;

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

impl<'a, DIR, T, M: RawMutex, E> Pin<'a, DIR, T, M>
where
    T: I2c<Error = E>,
{
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

    pub async fn into_input(self) -> Result<Pin<'a, Input<Floating>, T, M>, E> {
        let mut pcal = self.port_driver.lock().await;

        // Set to floating.
        pcal.pull_up_down_enable()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), false)))
            .await?;

        pcal.borrow_mut()
            .configuration()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), true)))
            .await?;

        Ok(self.transform())
    }

    pub async fn into_output(self, value: bool) -> Result<Pin<'a, Output, T, M>, E> {
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

        Ok(self.transform())
    }
}

impl<'a, PULLUPDOWN, T, M: RawMutex, E> Pin<'a, Input<PULLUPDOWN>, T, M>
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

    pub async fn into_pull_up(self) -> Result<Pin<'a, Input<PullUp>, T, M>, E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.pull_up_down_selection()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), true)))
            .await?;

        pcal.pull_up_down_enable()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), true)))
            .await?;

        Ok(self.transform())
    }

    pub async fn into_pull_down(self) -> Result<Pin<'a, Input<PullDown>, T, M>, E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.pull_up_down_selection()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), false)))
            .await?;

        pcal.pull_up_down_enable()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), true)))
            .await?;

        Ok(self.transform())
    }

    pub async fn into_pull_floating(self) -> Result<Pin<'a, Input<Floating>, T, M>, E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.pull_up_down_enable()
            .modify_async(|r| r.set_value(self.set_bit(r.value(), false)))
            .await?;

        Ok(self.transform())
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

    pub async fn into_interrupt(mut self) -> Result<InterruptPin<'a, PULLUPDOWN, T, M>, E> {
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

pub struct InterruptPin<'a, PULLUPDOWN, T, M: RawMutex>(
    ManuallyDrop<Pin<'a, Input<PULLUPDOWN>, T, M>>,
);

impl<'a, PULLUPDOWN, T, M: RawMutex, E> InterruptPin<'a, PULLUPDOWN, T, M>
where
    T: I2c<Error = E>,
{
    fn new(pin: Pin<'a, Input<PULLUPDOWN>, T, M>) -> Self {
        Self(ManuallyDrop::new(pin))
    }

    pub async fn deconfigure_interrupt(mut self) -> Result<Pin<'a, Input<PULLUPDOWN>, T, M>, E> {
        self.0.configure_interrupt(false).await?;
        // Deconstruct `self` into the inner pin without calling drop on `self`.
        let pin = unsafe { ManuallyDrop::take(&mut self.0) };
        core::mem::forget(self); // Prevent drop from being called on `self`.
        Ok(pin)
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

impl<PULLUPDOWN, T, M: RawMutex, E> embedded_hal::digital::ErrorType
    for InterruptPin<'_, PULLUPDOWN, T, M>
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

impl<PULLUPDOWN, T, M: RawMutex, E> embedded_hal_async::digital::Wait
    for InterruptPin<'_, PULLUPDOWN, T, M>
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

impl<T, M: RawMutex, E> OutputPin for Pin<'_, Output, T, M>
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

pub struct InterruptHandler<'a, T, M: RawMutex> {
    port_driver: PcalReference<'a, T, M>,
    interrupt_channels: [InterruptSender<'a, M>; 16],
}

impl<T, M: RawMutex, E> InterruptHandler<'_, T, M>
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

impl<T, M: RawMutex, E> Pcal6416<T, M>
where
    T: I2c<Error = E>,
{
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
