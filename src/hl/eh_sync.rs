use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_hal::i2c::I2c;

use super::*;

impl<'a, DIR, T, M: RawMutex, E> Pin<'a, DIR, T, M, Blocking>
where
    T: I2c<Error = E>,
{
    pub async fn into_input(self) -> Result<Pin<'a, Input<Floating>, T, M, Blocking>, E> {
        let mut pcal = self.port_driver.lock().await;

        // Set to floating.
        pcal.pull_up_down_enable()
            .modify(|r| r.set_value(self.set_bit(r.value(), false)))?;

        pcal.borrow_mut()
            .configuration()
            .modify(|r| r.set_value(self.set_bit(r.value(), true)))?;

        Ok(self.transform())
    }

    pub async fn into_output(self, value: bool) -> Result<Pin<'a, Output, T, M, Blocking>, E> {
        let mut pcal = self.port_driver.lock().await;

        // Set the output value before configuring it as an output

        pcal.borrow_mut()
            .output()
            .modify(|r| r.set_value(self.set_bit(r.value(), value)))?;

        pcal.borrow_mut()
            .configuration()
            .modify(|r| r.set_value(self.set_bit(r.value(), false)))?;

        Ok(self.transform())
    }
}

impl<'a, PULLUPDOWN, T, M: RawMutex, E> Pin<'a, Input<PULLUPDOWN>, T, M, Blocking>
where
    T: I2c<Error = E>,
{
    pub async fn set_input_latch(&mut self, value: bool) -> Result<(), E> {
        let mut pcal = self.port_driver.lock().await;
        pcal.input_latch()
            .modify(|r| r.set_value(self.set_bit(r.value(), value)))
    }

    pub async fn is_high(&mut self) -> Result<bool, E> {
        let mut pcal = self.port_driver.lock().await;
        let value = pcal.input().read()?;
        Ok(value.value() & self.mask_be() != 0)
    }

    pub async fn is_low(&mut self) -> Result<bool, E> {
        Ok(!self.is_high().await?)
    }

    pub async fn into_pull_up(self) -> Result<Pin<'a, Input<PullUp>, T, M, Blocking>, E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.pull_up_down_selection()
            .modify(|r| r.set_value(self.set_bit(r.value(), true)))?;

        pcal.pull_up_down_enable()
            .modify(|r| r.set_value(self.set_bit(r.value(), true)))?;

        Ok(self.transform())
    }

    pub async fn into_pull_down(self) -> Result<Pin<'a, Input<PullDown>, T, M, Blocking>, E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.pull_up_down_selection()
            .modify(|r| r.set_value(self.set_bit(r.value(), false)))?;

        pcal.pull_up_down_enable()
            .modify(|r| r.set_value(self.set_bit(r.value(), true)))?;

        Ok(self.transform())
    }

    pub async fn into_pull_floating(self) -> Result<Pin<'a, Input<Floating>, T, M, Blocking>, E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.pull_up_down_enable()
            .modify(|r| r.set_value(self.set_bit(r.value(), false)))?;

        Ok(self.transform())
    }

    async fn configure_interrupt(&mut self, enabled: bool) -> Result<(), E> {
        let mut pcal = self.port_driver.lock().await;
        pcal.input_latch()
            .modify(|r| r.set_value(self.set_bit(r.value(), enabled)))?;
        // A mask is negative, hence invert enabled.
        pcal.interrupt_mask()
            .modify(|r| r.set_value(self.set_bit(r.value(), !enabled)))?;

        Ok(())
    }

    pub async fn into_interrupt(
        mut self,
    ) -> Result<InterruptPin<'a, PULLUPDOWN, T, M, Blocking>, E> {
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

impl<'a, PULLUPDOWN, T, M: RawMutex, E> InterruptPin<'a, PULLUPDOWN, T, M, Blocking>
where
    T: I2c<Error = E>,
{
    pub async fn deconfigure_interrupt(
        mut self,
    ) -> Result<Pin<'a, Input<PULLUPDOWN>, T, M, Blocking>, E> {
        self.0.configure_interrupt(false).await?;
        // Deconstruct `self` into the inner pin without calling drop on `self`.
        let pin = unsafe { ManuallyDrop::take(&mut self.0) };
        core::mem::forget(self); // Prevent drop from being called on `self`.
        Ok(pin)
    }
}

impl<PULLUPDOWN, T, M: RawMutex, E> embedded_hal::digital::ErrorType
    for InterruptPin<'_, PULLUPDOWN, T, M, Blocking>
where
    T: I2c<Error = E>,
    E: embedded_hal::i2c::Error,
{
    type Error = DigitalError<E>;
}

impl<PULLUPDOWN, T, M: RawMutex, E> embedded_hal_async::digital::Wait
    for InterruptPin<'_, PULLUPDOWN, T, M, Blocking>
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

impl<T, M: RawMutex, E> OutputPin for Pin<'_, Output, T, M, Blocking>
where
    T: I2c<Error = E>,
{
    type Error = E;

    async fn set_value(&mut self, value: bool) -> Result<(), E> {
        let mut pcal = self.port_driver.lock().await;

        pcal.output()
            .modify(|r| r.set_value(self.set_bit(r.value(), value)))
    }
}

impl<T, M: RawMutex, E> InterruptHandler<'_, T, M, Blocking>
where
    T: I2c<Error = E>,
{
    pub async fn handle(&mut self) -> Result<(), E> {
        let mut pcal = self.port_driver.lock().await;

        let status = pcal.interrupt_status().read()?.value();
        // Clears interrupts
        let input = pcal.input().read()?.value();

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
