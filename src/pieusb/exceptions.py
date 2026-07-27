'''
Exception hierarchy for pieusb.

    PieusbError
    |-- TransportError      USB/SCSI transaction failed below the command layer
    |-- Timeout             a command or a wait did not complete in time
    |-- CheckCondition      SCSI CHECK CONDITION, carrying decoded sense data
    |-- DeviceNotReady      device is reachable but cannot start a scan
    |   `-- WarmingUp       ... specifically because the lamp is warming up
    `-- ScanInProgress      host-side: this Scanner already has a scan running

Layering note: `CheckCondition` is what the transport layer raises -- it reports
what the device said, nothing more. Turning a particular sense triple into
`WarmingUp` or `DeviceNotReady` is the Scanner's job, because that is an
interpretation of the sense data rather than a property of the transaction. The
C backend splits it the same way: `sanei_pieusb_command()` returns a status and
`sane_start()` decides what it means.
'''

# SCSI sense keys we distinguish (SPC-3 table 27)
SENSE_KEY_NOT_READY = 0x02
SENSE_KEY_UNIT_ATTENTION = 0x06

class PieusbError(Exception):
    '''Base class for every error raised by this package.'''

class TransportError(PieusbError):
    '''A USB or SCSI transaction failed below the command layer.'''

class Timeout(PieusbError):
    '''A command, or a wait for the device, did not complete in time.'''

class ScanInProgress(PieusbError):
    '''This Scanner already has a scan running.

    Host-side state, not something the device reported -- deliberately not a
    DeviceNotReady, because "wait and retry" and "you have a bug" want different
    handling.
    '''

class DeviceNotReady(PieusbError):
    '''The device is reachable but cannot start a scan.'''

class WarmingUp(DeviceNotReady):
    '''The lamp is still warming up. Retryable.

    A subclass of DeviceNotReady because that is what the wire says: both are
    sense key NOT_READY, and warming up is the single case singled out by
    code 0x04 / qualifier 0x01 (sanei_pieusb_decode_sense, pieusb_usb.c:387-397).
    '''

class CheckCondition(PieusbError):
    '''The device returned CHECK CONDITION; sense data has been read back.'''

    def __init__(self, sense_key: int, sense_code: int, sense_qualifier: int) -> None:
        self.sense_key = sense_key
        self.sense_code = sense_code
        self.sense_qualifier = sense_qualifier
        super().__init__(
            f"CHECK CONDITION key=0x{sense_key:02x} code=0x{sense_code:02x} "
            f"qualifier=0x{sense_qualifier:02x}"
        )

    @property
    def not_ready(self) -> bool:
        '''Sense key NOT READY -- the device cannot act on commands yet.

        Broader than `warming_up`: it also covers the not-ready reasons the C
        backend lumps into a generic failure (pieusb_usb.c:393-396).
        '''
        return self.sense_key == SENSE_KEY_NOT_READY

    @property
    def warming_up(self) -> bool:
        return self.not_ready and self.sense_code == 0x04 and self.sense_qualifier == 0x01

    @property
    def must_calibrate(self) -> bool:
        # "Calibration disable not granted" -- returned by START SCAN when
        # skipShadingAnalysis was requested but the scanner insists on
        # calibrating. In the SANE backend (sanei_pieusb_decode_sense) this maps
        # to PIEUSB_STATUS_MUST_CALIBRATE, which is NOT an error: it is the
        # signal to enter the calibration phase and read shading reference data
        # (pieusb.c:1091-1121). We treat it the same way.
        return (self.sense_key == SENSE_KEY_UNIT_ATTENTION
                and self.sense_code == 0x82
                and self.sense_qualifier == 0x00)
