#![no_std]

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
