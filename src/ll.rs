use device_driver::{AsyncRegisterInterface, RegisterInterface};

use crate::Address;

const MAX_TRANSACTION_SIZE: usize = 5;

pub struct Interface<I2C> {
    i2c: I2C,
    address: Address,
}

impl<I2C, E> AsyncRegisterInterface for Interface<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
{
    type Error = E;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; MAX_TRANSACTION_SIZE];
        buf[0] = address;
        buf[1..data.len() + 1].copy_from_slice(data);
        let buf = &buf[0..data.len() + 1];

        self.i2c.write(self.address.addr(), buf).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c
            .write_read(self.address.addr(), &[address], data)
            .await
    }
}

impl<I2C, E> RegisterInterface for Interface<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    type Error = E;
    type AddressType = u8;

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; MAX_TRANSACTION_SIZE];
        buf[0] = address;
        buf[1..data.len() + 1].copy_from_slice(data);
        let buf = &buf[0..data.len() + 1];

        self.i2c.write(self.address.addr(), buf)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address.addr(), &[address], data)
    }
}

impl<I2C> Interface<I2C> {
    pub fn new(i2c: I2C, address: Address) -> Self {
        Self { i2c, address }
    }

    pub fn take(self) -> I2C {
        self.i2c
    }
}

device_driver::create_device!(
    device_name: Device,
    manifest: "device.yaml"
);
