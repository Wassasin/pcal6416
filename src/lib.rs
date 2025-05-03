#![no_std]

use num_enum::{IntoPrimitive, TryFromPrimitive};

pub mod hl;
pub mod ll;

#[derive(Clone, Copy)]
pub enum Address {
    Default0,
    Alternative1,
}

impl Address {
    fn addr(self) -> u8 {
        use Address::*;
        match self {
            Default0 => 0b0100000,
            Alternative1 => 0b0100001,
        }
    }
}

#[derive(Debug, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum ConfigurationMode {
    Output = 0,
    Input = 1,
}

impl From<bool> for ConfigurationMode {
    fn from(value: bool) -> Self {
        (value as u8).try_into().unwrap()
    }
}

impl Into<bool> for ConfigurationMode {
    fn into(self) -> bool {
        self as u8 != 0
    }
}

#[derive(Debug, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum LogicValue {
    Low = 0,
    High = 1,
}

impl From<bool> for LogicValue {
    fn from(value: bool) -> Self {
        (value as u8).try_into().unwrap()
    }
}

impl Into<bool> for LogicValue {
    fn into(self) -> bool {
        self as u8 != 0
    }
}

#[derive(Debug, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum DriveStrength {
    Drive1_4 = 0b00,
    Drive2_4 = 0b01,
    Drive3_4 = 0b10,
    Drive4_4 = 0b11,
}

#[derive(Debug, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum PullUpDown {
    PullDown = 0,
    PullUp = 1,
}

impl From<bool> for PullUpDown {
    fn from(value: bool) -> Self {
        (value as u8).try_into().unwrap()
    }
}

impl Into<bool> for PullUpDown {
    fn into(self) -> bool {
        self as u8 != 0
    }
}

#[derive(Debug, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum OutputPortMode {
    PushPull = 0,
    OpenDrain = 1,
}

impl From<bool> for OutputPortMode {
    fn from(value: bool) -> Self {
        (value as u8).try_into().unwrap()
    }
}

impl Into<bool> for OutputPortMode {
    fn into(self) -> bool {
        self as u8 != 0
    }
}
