//! GPIO-based SWD probe for Raspberry Pi
//!
//! Implements bit-banging SWD protocol using GPIO pins via rppal library.
//! Supports 8 independent GPIO SWD probe slots for parallel debugging of multiple targets.

use crate::{
    probe::{DebugProbe, DebugProbeError, DebugProbeInfo, DebugProbeSelector, ProbeCreationError, ProbeFactory, WireProtocol},
    architecture::arm::{
        ArmError, RawDapAccess, DapProbe,
        communication_interface::{ArmCommunicationInterface, ArmDebugInterface},
        sequences::ArmDebugSequence,
        RegisterAddress,
    },
    CoreStatus,
};
use rppal::gpio::{Gpio, Level};
use std::sync::Arc;
use std::time::Duration;
use std::thread;
use std::fmt;

// GPIO pin configurations for 8 slots
const GPIO_SLOTS: [(u8, u8); 8] = [
    (3, 2),    // Slot 1: CLK=3, DIO=2
    (27, 17),  // Slot 2: CLK=27, DIO=17
    (9, 10),   // Slot 3: CLK=9, DIO=10
    (6, 5),    // Slot 4: CLK=6, DIO=5
    (26, 19),  // Slot 5: CLK=26, DIO=19
    (24, 23),  // Slot 6: CLK=24, DIO=23
    (8, 25),   // Slot 7: CLK=8, DIO=25
    (21, 20),  // Slot 8: CLK=21, DIO=20
];

/// Low-level GPIO SWD bit-banging implementation
struct GpioSwdInterface {
    clk_pin: u8,
    dio_pin: u8,
}

impl GpioSwdInterface {
    fn new(clk_pin: u8, dio_pin: u8) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let gpio = Gpio::new()?;
        // Verify GPIO access
        let _test_clk = gpio.get(clk_pin)?;
        let _test_dio = gpio.get(dio_pin)?;
        Ok(GpioSwdInterface { clk_pin, dio_pin })
    }

    fn clock_pulse(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let gpio = Gpio::new()?;
        let mut clk = gpio.get(self.clk_pin)?.into_output();
        clk.set_low();
        thread::sleep(Duration::from_micros(1));
        clk.set_high();
        thread::sleep(Duration::from_micros(1));
        Ok(())
    }

    fn read_bit(&self) -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
        let gpio = Gpio::new()?;
        let dio = gpio.get(self.dio_pin)?.into_input();
        thread::sleep(Duration::from_micros(1));
        Ok(dio.read() == Level::High)
    }

    fn write_bit(&self, bit: bool) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let gpio = Gpio::new()?;
        let mut dio = gpio.get(self.dio_pin)?.into_output();
        if bit {
            dio.set_high();
        } else {
            dio.set_low();
        }
        thread::sleep(Duration::from_micros(1));
        Ok(())
    }

    fn line_reset(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let gpio = Gpio::new()?;
        let mut dio = gpio.get(self.dio_pin)?.into_output();
        dio.set_high();
        for _ in 0..64 {
            self.clock_pulse()?;
        }
        Ok(())
    }

    fn jtag_to_swd(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sequence = 0xE79Eu16;
        for i in (0..16).rev() {
            let bit = (sequence >> i) & 1 == 1;
            self.write_bit(bit)?;
            self.clock_pulse()?;
        }
        Ok(())
    }

    fn write_command(&self, cmd: u8) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        for i in 0..8 {
            let bit = (cmd >> i) & 1 == 1;
            self.write_bit(bit)?;
            self.clock_pulse()?;
        }
        let parity = (cmd.count_ones() & 1) as u8;
        self.write_bit(parity != 0)?;
        self.clock_pulse()?;
        self.write_bit(false)?;
        self.clock_pulse()?;
        self.write_bit(true)?;
        self.clock_pulse()?;
        Ok(())
    }
}

/// GPIO-based SWD probe for Raspberry Pi
pub struct GpioSwdProbe {
    inner: GpioSwdInterface,
    protocol: Option<WireProtocol>,
    speed_khz: u32,
    slot: u8,
}

impl GpioSwdProbe {
    /// Create a new GPIO SWD probe for the specified slot (1-8)
    pub fn new(slot: u8) -> Result<Self, DebugProbeError> {
        if !(1..=8).contains(&slot) {
            return Err(DebugProbeError::ProbeCouldNotBeCreated(
                ProbeCreationError::Other("Invalid slot, must be 1-8")
            ));
        }

        let (clk, dio) = GPIO_SLOTS[slot as usize - 1];
        let inner = GpioSwdInterface::new(clk, dio)
            .map_err(|_| DebugProbeError::ProbeCouldNotBeCreated(
                ProbeCreationError::Other("Failed to initialize GPIO")
            ))?;

        Ok(GpioSwdProbe {
            inner,
            protocol: None,
            speed_khz: 100,
            slot,
        })
    }

    /// List all 8 GPIO SWD probe slots
    pub fn list_all() -> Vec<DebugProbeInfo> {
        (1..=8)
            .map(|slot| DebugProbeInfo::new(
                format!("GPIO SWD Slot {}", slot),
                0x0000, // VID (not applicable for GPIO)
                0x0000, // PID (not applicable for GPIO)
                Some(format!("gpio-slot-{}", slot)),
                &GpioSwdProbeFactory,
                None, // hid_interface (not applicable for GPIO)
            ))
            .collect()
    }
}

impl fmt::Debug for GpioSwdProbe {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GpioSwdProbe")
            .field("slot", &self.slot)
            .field("clk_pin", &self.inner.clk_pin)
            .field("dio_pin", &self.inner.dio_pin)
            .field("protocol", &self.protocol)
            .field("speed_khz", &self.speed_khz)
            .finish()
    }
}

impl DebugProbe for GpioSwdProbe {
    fn get_name(&self) -> &str {
        "GPIO SWD Probe (Raspberry Pi)"
    }

    fn speed_khz(&self) -> u32 {
        self.speed_khz
    }

    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        let actual_speed = speed_khz.clamp(10, 1000);
        self.speed_khz = actual_speed;
        Ok(actual_speed)
    }

    fn attach(&mut self) -> Result<(), DebugProbeError> {
        self.inner.line_reset()
            .map_err(|e| DebugProbeError::Other(format!("Line reset failed: {}", e)))?;
        self.inner.jtag_to_swd()
            .map_err(|e| DebugProbeError::Other(format!("JTAG-to-SWD failed: {}", e)))?;
        Ok(())
    }

    fn detach(&mut self) -> Result<(), crate::Error> {
        self.protocol = None;
        Ok(())
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), DebugProbeError> {
        match protocol {
            WireProtocol::Swd => {
                self.protocol = Some(WireProtocol::Swd);
                Ok(())
            }
            WireProtocol::Jtag => {
                Err(DebugProbeError::UnsupportedProtocol(WireProtocol::Jtag))
            }
        }
    }

    fn active_protocol(&self) -> Option<WireProtocol> {
        self.protocol
    }

    fn target_reset(&mut self) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset",
        })
    }

    fn target_reset_assert(&mut self) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset_assert",
        })
    }

    fn target_reset_deassert(&mut self) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset_deassert",
        })
    }

    fn has_arm_interface(&self) -> bool {
        true
    }

    fn has_riscv_interface(&self) -> bool {
        false
    }

    fn has_xtensa_interface(&self) -> bool {
        false
    }

    fn try_get_arm_debug_interface<'probe>(
        self: Box<Self>,
        sequence: Arc<dyn ArmDebugSequence>,
    ) -> Result<Box<dyn ArmDebugInterface + 'probe>, (Box<dyn DebugProbe>, ArmError)> {
        Ok(ArmCommunicationInterface::create(self, sequence, false))
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }
}

impl RawDapAccess for GpioSwdProbe {
    fn raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
        let ap_dp = if address.is_ap() { 1u8 } else { 0u8 };
        let reg_addr = address.a2_and_3();
        let cmd = (1u8 << 0) | (ap_dp << 1) | (reg_addr << 2);

        self.inner.write_command(cmd)
            .map_err(|e| ArmError::Other(format!("SWD command error: {}", e)))?;

        let mut ack = 0u8;
        for i in 0..3 {
            let bit = self.inner.read_bit()
                .map_err(|e| ArmError::Other(format!("ACK read error: {}", e)))? as u8;
            ack |= bit << i;
            self.inner.clock_pulse()
                .map_err(|e| ArmError::Other(format!("Clock error: {}", e)))?;
        }

        if ack != 0b001 {
            return Err(ArmError::Other(format!("Bad ACK: {:03b}", ack)));
        }

        let mut data = 0u32;
        for i in 0..32 {
            let bit = self.inner.read_bit()
                .map_err(|e| ArmError::Other(format!("Data read error: {}", e)))? as u32;
            data |= bit << i;
            self.inner.clock_pulse()
                .map_err(|e| ArmError::Other(format!("Clock error: {}", e)))?;
        }

        let _parity = self.inner.read_bit()
            .map_err(|e| ArmError::Other(format!("Parity read error: {}", e)))?;
        self.inner.clock_pulse()
            .map_err(|e| ArmError::Other(format!("Clock error: {}", e)))?;

        Ok(data)
    }

    fn raw_write_register(&mut self, address: RegisterAddress, value: u32) -> Result<(), ArmError> {
        let ap_dp = if address.is_ap() { 1u8 } else { 0u8 };
        let reg_addr = address.a2_and_3();
        let cmd = (ap_dp << 1) | (reg_addr << 2);

        self.inner.write_command(cmd)
            .map_err(|e| ArmError::Other(format!("SWD command error: {}", e)))?;

        let mut ack = 0u8;
        for i in 0..3 {
            let bit = self.inner.read_bit()
                .map_err(|e| ArmError::Other(format!("ACK read error: {}", e)))? as u8;
            ack |= bit << i;
            self.inner.clock_pulse()
                .map_err(|e| ArmError::Other(format!("Clock error: {}", e)))?;
        }

        if ack != 0b001 {
            return Err(ArmError::Other(format!("Bad ACK: {:03b}", ack)));
        }

        self.inner.clock_pulse()
            .map_err(|e| ArmError::Other(format!("Clock error: {}", e)))?;

        for i in 0..32 {
            let bit = (value >> i) & 1 == 1;
            self.inner.write_bit(bit)
                .map_err(|e| ArmError::Other(format!("Data write error: {}", e)))?;
            self.inner.clock_pulse()
                .map_err(|e| ArmError::Other(format!("Clock error: {}", e)))?;
        }

        let parity = value.count_ones() & 1 == 1;
        self.inner.write_bit(parity)
            .map_err(|e| ArmError::Other(format!("Parity write error: {}", e)))?;
        self.inner.clock_pulse()
            .map_err(|e| ArmError::Other(format!("Clock error: {}", e)))?;

        Ok(())
    }

    fn raw_read_block(&mut self, address: RegisterAddress, values: &mut [u32]) -> Result<(), ArmError> {
        for value in values.iter_mut() {
            *value = self.raw_read_register(address)?;
        }
        Ok(())
    }

    fn raw_write_block(&mut self, address: RegisterAddress, values: &[u32]) -> Result<(), ArmError> {
        for &value in values {
            self.raw_write_register(address, value)?;
        }
        Ok(())
    }

    fn raw_flush(&mut self) -> Result<(), ArmError> {
        Ok(())
    }

    fn jtag_sequence(&mut self, _cycles: u8, _tms: bool, _tdi: u64) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::UnsupportedProtocol(WireProtocol::Jtag))
    }

    fn swj_sequence(&mut self, bit_len: u8, bits: u64) -> Result<(), DebugProbeError> {
        for i in 0..bit_len {
            let bit = (bits >> i) & 1 == 1;
            self.inner.write_bit(bit)
                .map_err(|e| DebugProbeError::Other(format!("SWJ write failed: {}", e)))?;
            self.inner.clock_pulse()
                .map_err(|e| DebugProbeError::Other(format!("SWJ clock failed: {}", e)))?;
        }
        Ok(())
    }

    fn swj_pins(&mut self, _pin_out: u32, _pin_select: u32, _pin_wait: u32) -> Result<u32, DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "swj_pins",
        })
    }

    fn configure_jtag(&mut self, _skip_scan: bool) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::UnsupportedProtocol(WireProtocol::Jtag))
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }

    fn core_status_notification(&mut self, _state: CoreStatus) -> Result<(), DebugProbeError> {
        Ok(())
    }
}

impl DapProbe for GpioSwdProbe {}

/// Factory for creating GPIO SWD probes
pub struct GpioSwdProbeFactory;

impl fmt::Display for GpioSwdProbeFactory {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "GPIO SWD")
    }
}

impl fmt::Debug for GpioSwdProbeFactory {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "GpioSwdProbeFactory")
    }
}

impl ProbeFactory for GpioSwdProbeFactory {
    fn open(&self, selector: &DebugProbeSelector) -> Result<Box<dyn DebugProbe>, DebugProbeError> {
        // Parse slot from serial number (format: "gpio-slot-N")
        let slot = if let Some(serial) = &selector.serial_number {
            if let Some(slot_str) = serial.strip_prefix("gpio-slot-") {
                slot_str.parse::<u8>()
                    .map_err(|_| DebugProbeError::ProbeCouldNotBeCreated(
                        ProbeCreationError::Other("Invalid slot in serial")
                    ))?
            } else {
                return Err(DebugProbeError::ProbeCouldNotBeCreated(
                    ProbeCreationError::Other("Invalid GPIO probe serial")
                ));
            }
        } else {
            1 // Default to slot 1
        };

        let probe = GpioSwdProbe::new(slot)?;
        Ok(Box::new(probe))
    }

    fn list_probes(&self) -> Vec<DebugProbeInfo> {
        GpioSwdProbe::list_all()
    }
}
