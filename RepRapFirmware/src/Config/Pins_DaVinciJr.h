#ifndef PINS_DAVINCIJR_H__
#define PINS_DAVINCIJR_H__

#include <PinDescription.h>
#include <LpcProtocol.h>

// SAM4E mappings are cross-checked against the current da-vinci-jr-1.0-hacking
// pinout, component, connector and board documentation. Older conflicting
// connector/package-pin notes are not treated as verified hardware evidence.

#define BOARD_NAME              "Da Vinci Jr 1.0"
#define BOARD_SHORT_NAME        "DaVinciJr"
#define FIRMWARE_NAME           "RepRapFirmware for Da Vinci Jr 1.0"
#define DEFAULT_BOARD_TYPE      BoardType::DaVinciJr_10
#define IAP_FIRMWARE_FILE       "DaVinciJrFirmware.bin"
#define IAP_UPDATE_FILE         "DaVinciJrIap.bin"

constexpr uint32_t IAP_IMAGE_START = 0x20018000;

// Only the SAM4E-connected hardware mapped for this board is enabled here.
#define HAS_LWIP_NETWORKING     0
#define HAS_WIFI_NETWORKING     0
#define HAS_W5500_NETWORKING    0
#define HAS_SBC_INTERFACE       0
#define HAS_MASS_STORAGE        1
#define HAS_HIGH_SPEED_SD       1
#define HAS_CPU_TEMP_SENSOR     1
#define HAS_VOLTAGE_MONITOR     0
#define HAS_VREF_MONITOR        0
#define SUPPORT_TMC2660         0
#define SUPPORT_TMC22xx         0
#define SUPPORT_TMC51xx         0
#define SUPPORT_SPI_SENSORS     0
#define USART_SPI               0
#define SUPPORT_DHT_SENSOR      0
#define SUPPORT_12864_LCD       0
#define SUPPORT_ACCELEROMETERS  0
#define SUPPORT_LED_STRIPS      0
#define SUPPORT_LASER           0
#define SUPPORT_IOBITS          0
#define SUPPORT_OBJECT_MODEL    1
#define SUPPORT_ASYNC_MOVES     0
#define USE_CACHE               1
#define USE_MPU                 0

constexpr size_t NumDirectDrivers = 4;
constexpr size_t MaxSmartDrivers = 0;
constexpr size_t MaxSensors = 8;
constexpr size_t MaxHeaters = 1;          // hotend heater is controlled by the LPC1115 thermal controller
constexpr size_t MaxPortsPerHeater = 1;
constexpr size_t MaxMonitorsPerHeater = 3;
constexpr size_t MaxBedHeaters = 1;
constexpr size_t MaxChamberHeaters = 1;
constexpr int8_t DefaultE0Heater = 0;
constexpr size_t NumThermistorInputs = 0;
constexpr size_t MaxZProbes = 1;
constexpr size_t MaxGpInPorts = 9;
constexpr size_t MaxGpOutPorts = 1;
constexpr size_t MinAxes = 3;
constexpr size_t MaxAxes = 3;
constexpr size_t MaxDriversPerAxis = 1;
constexpr size_t MaxExtruders = 1;
constexpr size_t MaxAxesPlusExtruders = NumDirectDrivers;
constexpr size_t MaxHeatersPerTool = 1;
constexpr size_t MaxExtrudersPerTool = 1;
constexpr size_t MaxFans = 2;
constexpr unsigned int MaxTriggers = 16;
constexpr size_t MaxSpindles = 1;
constexpr size_t MaxLedStrips = 0;

constexpr size_t NumSerialChannels = 1;
constexpr size_t FirstAuxChannel = 1;
constexpr size_t NumAuxChannels = 0;
#define SERIAL_MAIN_DEVICE serialUSB
constexpr Pin UsbVBusPin = NoPin;

constexpr uint32_t LpcUartBaudRate = 115200;
constexpr Pin LpcUartRxPin = PortAPin(5);
constexpr Pin LpcUartTxPin = PortAPin(6);
constexpr GpioPinFunction LpcUartPinFunction = GpioPinFunction::C;

// LPC1115 ROM ISP control: PC13 drives PIO0_1/ISP and PC15 drives
// PIO0_0/RESET. Both signals are active low during boot entry.
constexpr Pin LpcIspPin = PortCPin(13);
constexpr Pin LpcResetPin = PortCPin(15);

// X, Y, Z, E1 motor wiring. The TB62269 ENABLE inputs are active high.
constexpr Pin DriverEnablePins[NumDirectDrivers] = {
	PortDPin(3), PortDPin(5), PortDPin(6), PortDPin(16)
};
constexpr Pin STEP_PINS[NumDirectDrivers] = {
	PortCPin(23), PortCPin(22), PortCPin(20), PortCPin(28)
};
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = {
	PortDPin(4), PortEPin(2), PortDPin(7), PortDPin(17)
};
constexpr bool DriverEnableActiveHigh = true;
constexpr uint32_t DefaultStandstillCurrentPercent = 100;

// No SAM4E thermistor inputs are mapped yet.
constexpr Pin TEMP_SENSE_PINS[1] = { NoPin };
constexpr float DefaultThermistorSeriesR = 4700.0;

// No SAM4E diagnostic/status LED or Z probe is assigned yet.
constexpr Pin DiagPin = NoPin;
constexpr bool DiagOnPolarity = true;

// SD card: HSMCI four-bit bus on PA26..PA31, card detect on PA25.
constexpr size_t NumSdCards = 1;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortAPin(25) };
// The socket grounds CD when no card is inserted; with the pull-up enabled,
// a high level therefore means that a card is present.
constexpr bool SdCardDetectHighMeansNoCard = false;
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin };
constexpr Pin SdSpiCSPins[1] = { NoPin };
constexpr IRQn SdhcIRQn = HSMCI_IRQn;
constexpr uint32_t ExpectedSdCardSpeed = 20000000;
constexpr Pin HsmciClockPin = PortAPin(29);
constexpr Pin HsmciOtherPins[] = {
	PortAPin(28), PortAPin(30), PortAPin(31), PortAPin(26), PortAPin(27)
};
constexpr GpioPinFunction HsmciPinsFunction = GpioPinFunction::C;
// Step pulse timer. All four step pins are on PIOC.
#define STEP_TC          (TC0)
#define STEP_TC_CHAN     (2)
#define STEP_TC_IRQN     TC2_IRQn
#define STEP_TC_HANDLER  TC2_Handler
#define STEP_TC_ID       ID_TC2

#define PIN_NONE        { TcOutput::none, PwmOutput::none, AdcInput::none, PinCapability::none, nullptr }
#define PIN_READ(name)  { TcOutput::none, PwmOutput::none, AdcInput::none, PinCapability::read, name }
#define PIN_WRITE(name) { TcOutput::none, PwmOutput::none, AdcInput::none, PinCapability::write, name }
#define PIN_RW(name)    { TcOutput::none, PwmOutput::none, AdcInput::none, PinCapability::rw, name }
#define PIN_PWM(name)   { TcOutput::none, PwmOutput::none, AdcInput::none, PinCapability::wpwm, name }
#define PIN_AIN(name)   { TcOutput::none, PwmOutput::none, AdcInput::none, PinCapability::ainr, name }

constexpr PinDescription PinTable[] =
{
	// Port A
	PIN_NONE,		// PA00
	PIN_NONE,		// PA01
	{ TcOutput::none, PwmOutput::pwm0h2_a, AdcInput::none, PinCapability::wpwm, "buzzer" },	// PA02 BZ1 PWMH2
	PIN_NONE,		// PA03
	PIN_NONE,		// PA04
	PIN_NONE,		// PA05
	PIN_NONE,		// PA06
	PIN_NONE,		// PA07
	PIN_NONE,		// PA08
	PIN_NONE,		// PA09
	PIN_NONE,		// PA10
	PIN_NONE,		// PA11
	PIN_NONE,		// PA12
	PIN_NONE,		// PA13
	PIN_NONE,		// PA14
	PIN_NONE,		// PA15
	PIN_NONE,		// PA16
	PIN_READ("!button.enter"),	// PA17 SW5 Enter button, active low
	PIN_NONE,		// PA18
	PIN_NONE,		// PA19
	PIN_NONE,		// PA20
	PIN_READ("!button.down"),	// PA21 SW2 Down button, active low
	PIN_NONE,		// PA22
	PIN_NONE,		// PA23
	PIN_NONE,		// PA24
	PIN_NONE,		// PA25 SD card detect
	PIN_NONE,		// PA26 SD DAT2
	PIN_NONE,		// PA27 SD DAT3
	PIN_NONE,		// PA28 SD CMD
	PIN_NONE,		// PA29 SD CLK
	PIN_NONE,		// PA30 SD DAT0
	PIN_NONE,		// PA31 SD DAT1

	// Port B
	PIN_NONE,		// PB00
	PIN_NONE,		// PB01
	PIN_NONE,		// PB02
	PIN_READ("!button.left"),	// PB03 SW4 Left button, active low
	PIN_NONE,		// PB04
	PIN_NONE,		// PB05
	PIN_NONE,		// PB06
	PIN_NONE,		// PB07
	PIN_NONE,		// PB08
	PIN_NONE,		// PB09
	PIN_NONE,		// PB10
	PIN_NONE,		// PB11
	PIN_NONE,		// PB12
	PIN_NONE,		// PB13
	PIN_NONE,		// PB14
	PIN_NONE,		// PB15
	PIN_NONE,		// PB16
	PIN_NONE,		// PB17
	PIN_NONE,		// PB18
	PIN_NONE,		// PB19
	PIN_NONE,		// PB20
	PIN_NONE,		// PB21
	PIN_NONE,		// PB22
	PIN_NONE,		// PB23
	PIN_NONE,		// PB24
	PIN_NONE,		// PB25
	PIN_NONE,		// PB26
	PIN_NONE,		// PB27
	PIN_NONE,		// PB28
	PIN_NONE,		// PB29
	PIN_NONE,		// PB30
	PIN_NONE,		// PB31

	// Port C
	PIN_RW("lcd.db0"),		// PC00 LCD DB0
	PIN_RW("lcd.db1"),		// PC01 LCD DB1
	PIN_RW("lcd.db2"),		// PC02 LCD DB2
	PIN_RW("lcd.db3"),		// PC03 LCD DB3
	PIN_RW("lcd.db4"),		// PC04 LCD DB4
	PIN_RW("lcd.db5"),		// PC05 LCD DB5
	PIN_RW("lcd.db6"),		// PC06 LCD DB6
	PIN_RW("lcd.db7"),		// PC07 LCD DB7
	PIN_WRITE("lcd.rw"),	// PC08 LCD R/W
	PIN_NONE,		// PC09
	PIN_WRITE("!lcd.backlight"),	// PC10 LCD cathode, active low
	PIN_NONE,		// PC11
	PIN_NONE,		// PC12
	PIN_WRITE("lcd.enable"),	// PC13 LCD Enable
	PIN_NONE,		// PC14
	PIN_NONE,		// PC15
	PIN_NONE,		// PC16
	PIN_NONE,		// PC17
	PIN_WRITE("lcd.rs"),	// PC18 LCD RS
	PIN_READ("ystop"),	// PC19 Y endstop
	PIN_NONE,		// PC20 Z step
	PIN_NONE,		// PC21
	PIN_NONE,		// PC22 Y step
	PIN_NONE,		// PC23 X step
	PIN_NONE,		// PC24
	PIN_NONE,		// PC25
	PIN_NONE,		// PC26
	PIN_NONE,		// PC27
	PIN_NONE,		// PC28 E1 step
	PIN_NONE,		// PC29
	PIN_NONE,		// PC30
	PIN_NONE,		// PC31

	// Port D
	PIN_NONE,		// PD00
	PIN_NONE,		// PD01
	PIN_NONE,		// PD02
	PIN_NONE,		// PD03 X enable
	PIN_NONE,		// PD04 X direction
	PIN_NONE,		// PD05 Y enable
	PIN_NONE,		// PD06 Z enable
	PIN_NONE,		// PD07 Z direction
	PIN_READ("xstop"),	// PD08 X endstop
	PIN_READ("zstop"),	// PD09 Z endstop
	PIN_NONE,		// PD10
	PIN_NONE,		// PD11
	PIN_NONE,		// PD12
	PIN_NONE,		// PD13
	PIN_NONE,		// PD14
	PIN_NONE,		// PD15
	PIN_NONE,		// PD16 E1 enable
	PIN_NONE,		// PD17 E1 direction
	PIN_NONE,		// PD18
	PIN_NONE,		// PD19
	PIN_NONE,		// PD20
	PIN_NONE,		// PD21
	PIN_NONE,		// PD22
	PIN_WRITE("toplamp"),	// PD23 Top lamp
	PIN_NONE,		// PD24
	PIN_NONE,		// PD25
	PIN_NONE,		// PD26
	PIN_NONE,		// PD27
	PIN_NONE,		// PD28
	PIN_NONE,		// PD29
	PIN_READ("!button.home"),	// PD30 SW6 Home button, active low
	PIN_NONE,		// PD31

	// Port E
	PIN_NONE,		// PE00
	PIN_READ("!button.up"),	// PE01 SW1 Up button, active low
	PIN_NONE,		// PE02 Y direction
	PIN_NONE,		// PE03
	PIN_READ("!button.right"),	// PE04 SW3 Right button, active low
	PIN_NONE,		// PE05

	// Verified non-NFC LPC1115-owned I/O, routed over the on-board UART.
	PIN_READ("lpc.filament_runout"),	// PIO2_7
	PIN_READ("lpc.rotation"),		// PIO2_1
	PIN_READ("!lpc.filament"),		// PIO0_6, active low
	PIN_WRITE("lpc.statusled"),		// PIO2_10
	PIN_PWM("lpc.fan"),			// PIO2_5 hotend fan
	PIN_PWM("lpc.reflowfan"),		// PIO1_10 reflow fan
	PIN_AIN("lpc.ntc")			// PIO1_0 hotend NTC
};

#undef PIN_NONE
#undef PIN_READ
#undef PIN_WRITE
#undef PIN_RW
#undef PIN_PWM
#undef PIN_AIN

constexpr size_t NumNamedPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32 + 32 + 32 + 32 + 6;
constexpr uint8_t LpcPinIds[] = {
	LpcProtocol::Pins::FilamentRunout, LpcProtocol::Pins::Rotation, LpcProtocol::Pins::HotendFilament,
	LpcProtocol::Pins::StatusLed, LpcProtocol::Pins::HotendFan, LpcProtocol::Pins::ReflowFan, LpcProtocol::Pins::HotendNtc
};
constexpr size_t NumLpcPins = ARRAY_SIZE(LpcPinIds);
constexpr Pin FirstLpcPin = NumRealPins;
static_assert(NumNamedPins == NumRealPins + NumLpcPins);

constexpr bool IsLpcPin(Pin pin) noexcept
{
	return pin >= FirstLpcPin && pin < NumNamedPins;
}

constexpr uint8_t GetLpcPinId(Pin pin) noexcept
{
	return LpcPinIds[pin - FirstLpcPin];
}

namespace StepPins
{
	static inline uint32_t CalcDriverBitmap(size_t driver) noexcept
	{
		return (driver < NumDirectDrivers) ? 1u << (STEP_PINS[driver] & 0x1Fu) : 0;
	}

	static inline __attribute__((always_inline)) void StepDriversHigh(uint32_t driverMap) noexcept
	{
		PIOC->PIO_SODR = driverMap;
	}

	static inline __attribute__((always_inline)) void StepDriversLow(uint32_t driverMap) noexcept
	{
		PIOC->PIO_CODR = driverMap;
	}
}

#endif
