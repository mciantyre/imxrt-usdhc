//! A driver for the i.MX RT uSDHC peripheral.
//!
//! Although it's designed for the imxrt-rs project, this driver may be
//! portable to other systems.

#![no_std]
#![deny(missing_docs, unsafe_op_in_unsafe_fn)]

mod blocking;
mod ral;

pub use blocking::{BlockingSdioHost, HostError, TransportError};

/// The size, in bits, for a data transfer.
///
/// This informs how many data pins are used in transfers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum DataTransferWidth {
    /// 1-bit mode.
    Bit1 = 0,
    /// 4-bit mode.
    Bit4 = 1,
    /// 8-bit mode.
    Bit8 = 2,
}

/// The DMA selection, if any.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
#[non_exhaustive]
pub enum DmaSelect {
    /// Simple DMA support.
    Simple = 0,
}

bitflags::bitflags! {
    /// Status and interrupt flags.
    ///
    /// Use this to understand all status bits, to enable status assertion, and to
    /// enable interrupts on status assertion.
    pub struct Status: u32 {
        /// An internal DMA transfer has failed.
        ///
        /// Could be caused by simple or advanced DMA failure, depending on which DMA
        /// interface you're using. Check the DMA system address to understand what
        /// fetch may have generated the error.
        const DMAE = 1 << 28;
        /// A tuning error occurred.
        ///
        /// Only applicable for or SD3.0 SDR104 mode and EMMC HS200 mode. It indicates
        /// an error in a tuning circuit.
        const TNE = 1 << 26;
        /// Auto CMD12 error.
        ///
        /// One of the fields in the auto CMD12 error status register has changed from 0 to 1.
        /// This flag also asserts when the auto CMD12 is not executed due a prior command error.
        const AC12E = 1 << 24;

        //
        // TOOD more docs for the rest of these flags.
        //

        /// Data end bit error.
        const DEBE = 1 << 22;
        /// Data CRC error.
        const DCE = 1 << 21;
        /// Data timeout error.
        const DTOE = 1 << 20;
        /// Command index error.
        const CIE = 1 << 19;
        /// Command end bit error.
        const CEBE = 1 << 18;
        /// Command CRC error
        const CCE = 1 << 17;
        /// Command timeout error.
        const CTOE = 1 << 16;
        /// Tuning pass.
        ///
        /// Only applies to SD3.0 SDR104 mode and EMMC HS200 modes. CMD18 transfer was successful, and the sampling
        /// point is correct.
        const TP = 1 << 14;
        /// Re-tuning event.
        ///
        /// Only applies to SD3.0 SDR104 mode and EMMC HS200 modes. A clear bit indicates that re-tuning is not
        /// required. On the other hand, a set bit indicates that you should re-tune.
        const RTE = 1 << 12;
        /// Card interrupt.
        const CINT = 1 << 8;
        /// Card removal.
        const CRM = 1 << 7;
        /// Card insertion.
        const CINS = 1 << 6;
        /// Buffer read ready.
        ///
        /// This flag sets when `BREN` asserts in the [`PresentState`].
        const BRR = 1 << 5;
        /// Buffer write ready.
        ///
        /// This flag sets when `BWEN` asserts in the [`PresentState`].
        const BWR = 1 << 4;
        /// DMA interrupt.
        ///
        /// The internal DMA finished a transfer. Occurs for both simple and advanced DMA transfers.
        const DINT = 1 << 3;
        /// Block gap event.
        const BGE = 1 << 2;
        /// Transfer complete.
        const TC = 1 << 1;
        /// Command complete.
        ///
        /// Set whenever the peripheral receives the end field of the command response.
        /// This *does not* apply to CMD12.
        const CC = 1 << 0;
    }
}

impl Status {
    /// All flags that indicate an error.
    pub const ERRORS: Status = Status::empty()
        .union(Self::DMAE)
        .union(Self::TNE)
        .union(Self::AC12E)
        .union(Self::DEBE)
        .union(Self::DCE)
        .union(Self::DTOE)
        .union(Self::CIE)
        .union(Self::CEBE)
        .union(Self::CCE)
        .union(Self::CTOE);

    /// Indicates if any error bit is set.
    #[inline]
    pub const fn is_error(self) -> bool {
        self.intersects(Self::ERRORS)
    }
}

/// Endianness for the data transfer.
///
/// Describes the types of byte swaps that occur when interfacing the internal
/// data buffer. A "word" is 32 bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u32)]
pub enum EndianMode {
    /// Big endian.
    ///
    /// Swap all bytes in the data word when reading from / writing to the internal
    /// data buffer.
    BigEndian = 0,
    /// Swap half of a big endian word.
    ///
    /// Assume 16-bit little endian packing, then swap the order of those half
    /// words.
    HalwordBigEndian = 1,
    /// Little endian, the default behavior.
    ///
    /// No swapping. Read from / write to the internal data buffer while maintaining
    /// system endianness.
    #[default]
    LittleEndian = 2,
}

/// Read and write watermark levels.
///
/// "Levels" represent the number of words for DMA operations.
/// The implementation clamps the range between 1 and 128. For maximum
/// throughput to memory, select a value that's a multiple of 16 for each
/// level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Watermark {
    /// Number of watermark words for a DMA write.
    ///
    /// This can be set to any value between 1 and 128, no matter
    /// the DMA mode.
    pub write_level: u8,
    /// Number of watermark words for a DMA read.
    ///
    /// When using the internal DMA mode, this cannot be greater than 16.
    /// Applies to both simple and advanced DMA modes. Use 16 for best
    /// performance.
    // (Developers note: iMXRT1170 reference manual field documentation
    // says "must be set to the value less than 16." Following this guidance
    // strictly would indicate that we cannot set "16." The immediate next
    // sentence (as well as "data buffer" documentation) say we can, and should
    // use the value "16." So we're ignoring that first sentence in the field doc,
    // I guess.
    pub read_level: u8,
}

/// Timing parameters.
///
/// Controls the frequency of the CLK line as a function
/// of the input clock frequency. Also controls the data
/// rate and data timeout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    /// Internal divisor.
    ///
    /// The implementation clamps this between 1 and 16.
    pub divisor: u8,
    /// Data rate selection.
    ///
    /// This also determines the prescaler for the clock.
    pub data_rate: DataRate,
    // TODO data timeout.
}

/// Data rate and prescaler selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataRate {
    /// Single data rate mode.
    SingleDataRate(SDRPrescaler),
    /// Dual data rate mode.
    DualDataRate(DDRPrescaler),
}

/// Prescaler selections for single data rate mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
#[repr(u32)]
pub enum SDRPrescaler {
    /// Divide by 1.
    ///
    /// This effectively bypasses the prescaler.
    Divide1 = 0,
}

/// Prescaler selections for dual data rate mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
#[repr(u32)]
pub enum DDRPrescaler {
    /// Divide by 2.
    Divide2 = 0,
}

bitflags::bitflags! {
    /// Status flags depending on card presence.
    ///
    /// Some flags are only used for debugging. However,
    /// other fields influence when / if commands and data
    /// are sent by the peripheral.
    pub struct PresentState: u32 {
        // TODO data signal lines.
        //
        // The prose for this field (at least in the 1170 reference
        // manual) makes it seem that this *isn't* a bit field. But
        // it really seems that it is a bit field...

        //
        // TODO more docs.
        //

        /// CMD line signal level.
        ///
        /// Reflects the state of the CMD line.
        const CLSL = 1 << 23;
        /// Write protect switch pin level.
        ///
        /// If this flag is high, then write protect is enabled.
        const WPSPL = 1 << 19;
        /// Card detect pin level.
        ///
        /// If this flag is set, then there is a card present.
        const CDPL = 1 << 18;
        /// Card inserted.
        ///
        /// If this flag is set, then there is a card inserted.
        const CINST = 1 << 16;
        /// Tap select change done.
        const TSCD = 1 << 15;
        /// Re-tuning request.
        ///
        /// Only applies for SD3.0 SDR 104 and EMMC HS200 modes.
        const RTR = 1 << 12;
        /// Buffer read enable.
        ///
        /// Used for non-DMA transfers. If this flag is set, then
        /// there is _data_ for reading in the internal buffer.
        /// The amount of _data_ is greater than the read watermark level.
        const BREN = 1 << 11;
        /// Buffer write enable.
        ///
        /// Used for non-DMA transfers. If this flag is set, then
        /// there is _space_ in the internal buffer for writing.
        /// The amount of _space_ is greater than the write watermark level.
        const BWEN = 1 << 10;

        // TODO RTA, WTA.

        /// SD clock gated off internally.
        ///
        /// This is only used for debugging transactions. Reasons the flag
        /// is set are indicated below.
        ///
        /// - `FRC_SDCLK_ON` is cleared, so the peripheral gates the
        ///    clock in the idle status. `FRC_SDCLK_ON` is nominally cleared.
        /// - Buffer over / under run.
        /// - Read pause without wait assertion.
        const SDOFF = 1 << 7;

        // TODO PERCLK, IPG, HCLK status flags.

        /// SD clock stable.
        ///
        /// If this is set, the clock is stable.
        const SDSTB = 1 << 3;

        /// Data line active.
        ///
        /// If this flag is set, one of the data lines is in use. "In use"
        /// depends on the transfer direction.
        const DLA = 1 << 2;

        /// Command inhibit data.
        ///
        /// If this flag is set, you cannot issue a command that uses a data
        /// line. When this flag clears, it generates a tranfer complete status.
        /// While this flag is clear, you can issue commands that require data
        /// lines.
        const CDIHB = 1 << 1;

        /// Command inhibit.
        ///
        /// If this flag is set, you cannot issue any command. When this flag
        /// clears, it generates a command complete status. While this flag is clear,
        /// you can issue commands using only the CMD signal.
        ///
        /// Note that auto CMD12 behaviors are not reflected in this field.
        const CIHB = 1 << 0;
    }
}

/// Indicates the direction of the next data transfer.
///
/// This controls the data line direction. There are special conditions
/// under which you must use a write, particularly around re-initialization.
/// Consult the reference manual for more information.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum DataTransferDirection {
    /// Write data (host to card).
    Write,
    /// Read data (card to host).
    Read,
}

/// A uSDHC driver.
///
/// This is a lower-level driver on which you can build more advanced, safer functions.
pub struct Usdhc {
    inst: ral::Instance,
}

impl Usdhc {
    /// Create a new uSDHC driver.
    ///
    /// This call simply allocates the driver. It does not perform any kind of reset
    /// or state changes on the peripheral.
    ///
    /// # Safety
    ///
    /// Caller must supply a pointer to the start of a uSDHC peripheral
    /// block. This varies by chip. Consult your reference manual for more
    /// information.
    ///
    /// The implementation assumes that it is the sole owner of this peripheral.
    /// Caller must also ensure that this object represents all of the software
    /// manipulating this peripheral.
    #[inline]
    pub unsafe fn new(ptr: *const ()) -> Self {
        let inst = unsafe { ral::Instance::new(ptr) };
        Self { inst }
    }

    /// Issue a full software reset.
    ///
    /// This resets all system control, command, and data states. Blocks
    /// until the reset completes.
    #[inline]
    pub fn software_reset(&mut self) {
        const RESET_DONE: (u32, u32, u32) = (0, 0, 0);
        // Might be overkill to also tickle RSTC and RSTD; RSTA is supposed
        // to affect those functions. But let's be thorough...
        ral::modify_reg!(ral, self.inst, SYS_CTRL, RSTA: 1, RSTC: 1, RSTD: 1);
        while ral::read_reg!(ral, self.inst, SYS_CTRL, RSTA, RSTC, RSTD) != RESET_DONE {}
    }

    /// Reset the command path and line.
    ///
    /// This performs a subset of the [`software_reset()`] behavior, just for
    /// the command circuit. Blocks until the reset completes.
    #[inline]
    pub fn command_reset(&mut self) {
        ral::modify_reg!(ral, self.inst, SYS_CTRL, RSTC: 1);
        while ral::read_reg!(ral, self.inst, SYS_CTRL, RSTC == 1) {}
    }

    /// Control the hardware reset line.
    ///
    /// When `true`, the reset line signals "on" to the device. When
    /// `false`, the reset line signals "off." To reset your device,
    /// call this with `false`, then delay, then call this with `true`.
    ///
    /// The implementation assumes that you have muxed the reset signal
    /// to the uSDHC controller. If the reset signal is not muxed, the
    /// behavior is unspecified.
    #[inline]
    pub fn set_hardware_reset(&mut self, enable: bool) {
        ral::modify_reg!(ral, self.inst, SYS_CTRL, IPP_RST_N: !enable as u32);
    }

    /// Set timing parameters.
    ///
    /// See [`Timing`] documentation for more information. This call blocks while
    /// until the internal SD clock stabilizes.
    #[inline]
    pub fn set_timing(&mut self, timing: Timing) {
        while !self.present_state().intersects(PresentState::SDSTB) {}

        let (prescaler, ddr_en) = match timing.data_rate {
            DataRate::DualDataRate(ddr) => (ddr as u32, 1),
            DataRate::SingleDataRate(sdr) => (sdr as u32, 0),
        };
        ral::modify_reg!(ral, self.inst, MIX_CTRL, DDR_EN: ddr_en);

        let divisor = timing.divisor.clamp(1, 16) - 1;
        ral::modify_reg!(
            ral,
            self.inst,
            SYS_CTRL,
            DVS: divisor as u32,
            SDCLKFS: prescaler,
            DTOCV: 0 // TODO allow user config through Timing.
        );
    }

    /// Read the status flags.
    ///
    /// The set of flags that _could_ be set are based on the status enable
    /// setting.
    #[inline]
    pub fn status(&self) -> Status {
        Status::from_bits_truncate(ral::read_reg!(ral, self.inst, INT_STATUS))
    }

    /// Clear the status bits that are set high in `status`.
    ///
    /// Software can use [`status`](Self::status) to perform a RMW operation
    /// on the status bits. This clears any interrupt status conditions.
    #[inline]
    pub fn clear_status(&self, status: Status) {
        ral::write_reg!(ral, self.inst, INT_STATUS, status.bits());
    }

    /// Set the conditions that are signaled through status flags.
    ///
    /// Set bits indicate that the status could be signaled. Clear bits are
    /// never signaled.
    #[inline]
    pub fn set_status_enable(&self, status: Status) {
        ral::write_reg!(ral, self.inst, INT_STATUS_EN, status.bits());
    }

    /// Set the status conditions that trigger an interrupt.
    ///
    /// Note that the interrupt doesn't activate if the corresponding bit
    /// is clear in [`set_status_enable`](Self::set_status_enable).
    #[inline]
    pub fn set_status_interrupt(&self, status: Status) {
        ral::write_reg!(ral, self.inst, INT_SIGNAL_EN, status.bits());
    }

    /// Enable or disable DMA support.
    ///
    /// `None` disables DMA. A `Some(...)` enables DMA using the provided
    /// selection.
    #[inline]
    pub fn set_dma_enable(&mut self, dma_enable: Option<DmaSelect>) {
        match dma_enable {
            None => {
                ral::modify_reg!(ral, self.inst, MIX_CTRL, DMAEN: 0);
                ral::modify_reg!(ral, self.inst, PROT_CTRL, DMASEL: 0);
            }
            Some(selection) => {
                ral::modify_reg!(ral, self.inst, PROT_CTRL, DMASEL: selection as u32);
                ral::modify_reg!(ral, self.inst, MIX_CTRL, DMAEN: 1);
            }
        }
    }

    /// Returns the endian mode.
    pub fn endian_mode(&self) -> EndianMode {
        let emode = ral::read_reg!(ral, self.inst, PROT_CTRL, EMODE);
        assert!(emode < 3, "Invalid EMODE");
        // Safety: module inspection shows that EndianMode can take values
        // 0, 1, and 2. Assert ensures there's no invalid values.
        unsafe { core::mem::transmute(emode) }
    }

    /// Set the endian mode.
    ///
    /// See the [`EndianMode`] documentation for more information.
    #[inline]
    pub fn set_endian_mode(&mut self, mode: EndianMode) {
        ral::modify_reg!(ral, self.inst, PROT_CTRL, EMODE: mode as u32);
    }

    /// Returns the watermark levels for the FIFO thresholds.
    #[inline]
    pub fn watermark(&self) -> Watermark {
        let (write_level, read_level) = ral::read_reg!(ral, self.inst, WTMK_LVL, WR_WML, RD_WML);
        Watermark {
            write_level: write_level as u8,
            read_level: read_level as u8,
        }
    }

    /// Set the watermark levels.
    ///
    /// See the [`Watermark`] documentation for more information about valid
    /// values.
    #[inline]
    pub fn set_watermark(&self, watermark: Watermark) {
        ral::modify_reg!(ral, self.inst, WTMK_LVL,
            WR_WML: watermark.write_level.clamp(0, 128) as u32,
            RD_WML: watermark.read_level.clamp(0, 128) as u32
        )
    }

    /// Returns the data transfer width.
    pub fn data_transfer_width(&self) -> DataTransferWidth {
        let dtw = ral::read_reg!(ral, self.inst, PROT_CTRL, DTW);
        assert!(dtw < 3, "Invalid DTW");
        // Safety: module inspection shows that DataTransferWidth
        // can take values 0, 1, and 2. Assert ensures there's no invalid
        // values.
        unsafe { core::mem::transmute(dtw) }
    }

    /// Set the data transfer width.
    #[inline]
    pub fn set_data_transfer_width(&mut self, dtw: DataTransferWidth) {
        ral::modify_reg!(ral, self.inst, PROT_CTRL, DTW: dtw as u32);
    }

    /// Instruct the peripheral to initialize the card.
    ///
    /// This should be performed after software reset, and prior
    /// to sending a reset command. When this call returns, the
    /// driver has instructed the peripheral to send 80 clock cycles
    /// to the card.
    ///
    /// This call may return before all clock cycles are sent to the
    /// card. This is OK; any commands enqueued during the initialization
    /// will be sent after the dummy cycles are complete.
    ///
    /// This call does not check the command or data inhibit states.
    /// Caller is responsible for checking these states before initializing
    /// the card. In summary, this call does nothing if the command or data
    /// lines are busy.
    #[inline]
    pub fn initialize_card(&mut self) {
        ral::modify_reg!(ral, self.inst, SYS_CTRL, INITA: 1);
    }

    /// Read the status flags for card presence.
    #[inline]
    pub fn present_state(&self) -> PresentState {
        PresentState::from_bits_truncate(ral::read_reg!(ral, self.inst, PRES_STATE))
    }

    /// Write to the data buffer.
    ///
    /// This performs no checks for available space in the data
    /// buffer. To understand the available space, use the watermark
    /// and write buffer status flags.
    ///
    /// If DMA is enabled, this write is ignored.
    #[inline]
    pub fn write_data_buffer(&self, word: u32) {
        ral::write_reg!(ral, self.inst, DATA_BUFF_ACC_PORT, word);
    }

    /// Read from the data buffer.
    ///
    /// This performs no checks for valid data in the data buffer.
    /// To understand the available space, use the watermark and
    /// read buffer status flags.
    ///
    /// If DMA is enabled, this always returns zero.
    #[inline]
    pub fn read_data_buffer(&self) -> u32 {
        ral::read_reg!(ral, self.inst, DATA_BUFF_ACC_PORT)
    }

    /// Set the direction for the next data transfer.
    #[inline]
    pub fn set_data_transfer_direction(&mut self, dir: DataTransferDirection) {
        ral::modify_reg!(ral, self.inst, MIX_CTRL, DTDSEL: dir as u32);
    }
}
