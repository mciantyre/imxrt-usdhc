//! Implementation of a blocking transport.

use sdio_host::{
    common_cmd::{Resp, ResponseLen},
    sd::BusWidth,
    BlockingSdioTransport, Cmd, TransportData, TransportMode,
};
pub use sdio_host::{HostError, TransportError};

use crate::{ral, DataTransferWidth, PresentState, Status, Usdhc, Watermark};

/// A blocking SDIO host using uSDHC.
pub type BlockingSdioHost = sdio_host::BlockingSdioHost<Usdhc>;

fn transport_error(status: Status) -> TransportError {
    if status.intersects(Status::CTOE) {
        TransportError::CommandTimeout
    } else if status.intersects(Status::CIE) {
        TransportError::CommandIndex
    } else if status.intersects(Status::DTOE) {
        TransportError::DataTimeout
    } else if status.intersects(Status::CCE | Status::DCE) {
        TransportError::Crc
    } else if status.intersects(Status::CEBE | Status::DEBE) {
        TransportError::Bit
    } else {
        TransportError::uncategorized()
    }
}

impl Usdhc {
    fn wait_for(&mut self, flags: Status) -> Result<(), TransportError> {
        while {
            let status = self.status();
            if status.is_error() {
                return Err(transport_error(status));
            }
            !status.intersects(flags)
        } {}
        self.clear_status(flags);
        Ok(())
    }

    /// Correct execution depends on watermark levels. See the `transfer`
    /// implementation for more details.
    fn read_into(&mut self, buffer: &mut [u8]) -> Result<(), TransportError> {
        for bytes in buffer.chunks_exact_mut(4) {
            self.wait_for(Status::BRR)?;

            let word = self.read_data_buffer();
            bytes.copy_from_slice(&word.to_le_bytes());
        }

        Ok(())
    }

    /// Correct execution depends on watermark levels. See the `transfer`
    /// implementation for more details.
    fn write_from(&mut self, buffer: &[u8]) -> Result<(), TransportError> {
        for bytes in buffer.chunks_exact(4) {
            self.wait_for(Status::BWR)?;

            let word = u32::from_le_bytes(bytes.try_into().unwrap());
            self.write_data_buffer(word);
        }

        Ok(())
    }
}

/// Implements the blocking SDIO transport.
///
/// # Assumptions
///
/// Power cycle assumes that your reset line controls the hardware. If this isn't
/// the case, the implementation may not actually power cycle your device. You're
/// responsible for muxing the reset pin with your IOMUXC peripheral.
impl BlockingSdioTransport for Usdhc {
    fn transfer<R>(
        &mut self,
        command: &Cmd<R>,
        response: &mut [u32; 4],
        data: TransportData<'_>,
    ) -> Result<(), TransportError>
    where
        R: Resp,
    {
        if self.status().is_error() {
            self.clear_status(Status::ERRORS);
        }

        while self
            .present_state()
            .intersects(PresentState::CIHB | PresentState::CDIHB)
        {
            // TODO timeout in case these never clear...
        }

        self.clear_status(Status::all());

        // For now, always signal whenever one data word is available for
        // reading. I'm not sure what happens if the data is not a multiple
        // of four bytes...
        assert!(
            data.len() % 4 == 0,
            "Data lenght must be a multiple of four"
        );
        self.set_watermark(Watermark {
            write_level: 128,
            read_level: 1,
        });

        let rsptyp = match command.response_len() {
            ResponseLen::Zero => 0,
            ResponseLen::R136 => 1,
            ResponseLen::R48 => 2,
            // TODO not signaling "check busy" for R1b / R5b.
            // As of this writing, this information is not
            // available in sdio-host.
        };

        ral::modify_reg!(ral, self.inst, MIX_CTRL, DTDSEL: data.is_read() as u32);
        ral::write_reg!(ral, self.inst, BLK_ATT, BLKSIZE: data.len() as u32);
        ral::write_reg!(ral, self.inst, CMD_ARG, command.arg);
        ral::write_reg!(ral, self.inst, CMD_XFR_TYP,
            CMDINX: command.cmd as u32,
            CMDTYP: 0, // TODO may need to derive this from command index...
            DPSEL: !data.is_none() as u32,
            CICEN: R::COMMAND_INDEX as u32,
            CCCEN: R::CRC as u32,
            RSPTYP: rsptyp
        );

        self.wait_for(Status::CC)?;

        match command.response_len() {
            ResponseLen::Zero => {}
            ResponseLen::R48 => {
                response[0] = ral::read_reg!(ral, self.inst, CMD_RSP0);
            }
            ResponseLen::R136 => {
                response[0] = ral::read_reg!(ral, self.inst, CMD_RSP0);
                response[1] = ral::read_reg!(ral, self.inst, CMD_RSP1);
                response[2] = ral::read_reg!(ral, self.inst, CMD_RSP2);
                response[3] = ral::read_reg!(ral, self.inst, CMD_RSP3);

                // Hardware does not expose the internal CRC and end bit.
                // We're allowed to spoof these values to meet the interface
                // requirements. In this implementation, the CRC and end bit
                // are zero.
                response[3] = response[3] << 8 | response[2] >> 24;
                response[2] = response[2] << 8 | response[1] >> 24;
                response[1] = response[1] << 8 | response[0] >> 24;
                response[0] <<= 8;
            }
        };

        match data {
            TransportData::Read { buffer } => self.read_into(buffer)?,
            TransportData::Write { buffer } => self.write_from(buffer)?,
            TransportData::None => (),
        }

        Ok(())
    }

    fn power_cycle(&mut self, delay: &mut impl FnMut(u32)) -> Result<(), TransportError> {
        // Reset the device by driving the reset line.
        self.set_hardware_reset(false);
        delay(100);
        self.set_hardware_reset(true);

        // Software reset the entire peripheral.
        self.software_reset();

        // Make sure we can see all the status flags.
        self.set_status_enable(Status::all());
        // (Should have been cleared from reset, but
        // you never know...)
        self.clear_status(Status::all());
        // Don't enable any interrupts.
        self.set_status_interrupt(Status::empty());
        // No DMA in this blocking implementation...
        self.set_dma_enable(None);

        // Send the ~80 clock cycles to the card.
        delay(5);
        self.initialize_card();
        // Let that settle...
        delay(5);

        Ok(())
    }

    fn set_bus_width(&mut self, bus_width: sdio_host::sd::BusWidth) -> Result<(), TransportError> {
        self.set_data_transfer_width(match bus_width {
            BusWidth::One => DataTransferWidth::Bit1,
            BusWidth::Four => DataTransferWidth::Bit4,
            BusWidth::Eight => DataTransferWidth::Bit8,
            _ => return Err(TransportError::NotSupported),
        });

        Ok(())
    }

    fn set_mode(&mut self, _: TransportMode) -> Result<(), TransportError> {
        // TODO actually do a clock / voltage change...
        Ok(())
    }
}
