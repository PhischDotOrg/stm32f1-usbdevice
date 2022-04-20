/*-
 * $Copyright$
-*/
#include <common/Infrastructure.hpp>
#include <phisch/log.h>

/* for vTaskStartScheduler */
#include <FreeRTOS.h>
#include <FreeRTOS/include/task.h>

#include <stm32/Cpu.hpp>

#include <stm32/Pll.hpp>
#include <stm32/Pwr.hpp>
#include <stm32/Flash.hpp>
#include <stm32/Gpio.hpp>
#include <stm32/Rcc.hpp>
#include <stm32/Scb.hpp>
#include <stm32/Nvic.hpp>

#include <gpio/GpioAccess.hpp>
#include <gpio/GpioEngine.hpp>
#include <gpio/GpioPin.hpp>

#include <stm32/Uart.hpp>
#include <uart/UartAccess.hpp>
#include <uart/UartDevice.hpp>

#include <stm32/Itm.hpp>
#include <stm32/Tpi.hpp>
#include <stm32/CoreDbg.hpp>
#include <stm32/DbgMcu.hpp>

#include <tasks/Heartbeat.hpp>

#include <usb/UsbTypes.hpp>

#include <usb/UsbDevice.hpp>
#include <usb/UsbInEndpoint.hpp>
#include <usb/UsbOutEndpoint.hpp>
#include <usb/UsbControlPipe.hpp>
#include <usb/UsbConfiguration.hpp>
#include <usb/UsbVendorInterface.hpp>

#include <usb/UsbApplication.hpp>

#include <usb/UsbDescriptors.hpp>

#include <array>

/*******************************************************************************
 * System Devices
 ******************************************************************************/
static const constexpr stm32::PllCfg pllCfg = {
    .m_pllSource        = stm32::PllCfg::PllSource_t::e_PllSourceHSE,
    .m_hseSpeedInHz     = 8 * 1000 * 1000,
    .m_hsePrescaler     = stm32::PllCfg::HSEPrescaler_t::e_HSEPrescaler_None,
    .m_pllM             = stm32::PllCfg::PllMul_t::e_PllM_9,
    .m_sysclkSource     = stm32::PllCfg::SysclkSource_t::e_SysclkPLL,
    .m_ahbPrescaler     = stm32::PllCfg::AHBPrescaler_t::e_AHBPrescaler_None,
    .m_apb1Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_Div2,
    .m_apb2Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_None,
    .m_usbPrescaler     = stm32::PllCfg::USBPrescaler_t::e_USBPrescaler_Div15,
};

static stm32::Scb                       scb(SCB);
static stm32::Nvic                      nvic(NVIC, scb);

static stm32::Flash                     flash(FLASH);
static stm32::Rcc                       rcc(RCC, pllCfg, flash);
static stm32::Pwr                       pwr(rcc, scb);

/*******************************************************************************
 * GPIO Engine Handlers
 ******************************************************************************/
static stm32::Gpio::A                   gpio_A(rcc);
static gpio::GpioEngine                 gpio_engine_A(&gpio_A);

static stm32::Gpio::B                   gpio_B(rcc);
static gpio::GpioEngine                 gpio_engine_B(&gpio_B);

/*******************************************************************************
 * LEDs
 ******************************************************************************/
static gpio::AlternateFnPin             g_mco1(gpio_engine_A, 8);
static gpio::DigitalOutPin              g_led_green(gpio_engine_B, 2);

/*******************************************************************************
 * Debug Pins
 ******************************************************************************/
static gpio::DigitalOutPinT< ::gpio::PinPolicy::Termination_e::e_None >   debugPin0(gpio_engine_A, 0);
static gpio::DigitalOutPinT< ::gpio::PinPolicy::Termination_e::e_None >   debugPin1(gpio_engine_A, 1);
static gpio::DigitalOutPinT< ::gpio::PinPolicy::Termination_e::e_None >   debugPin2(gpio_engine_A, 2);
static gpio::DigitalOutPinT< ::gpio::PinPolicy::Termination_e::e_None >   debugPin3(gpio_engine_A, 3);
static gpio::DigitalOutPinT< ::gpio::PinPolicy::Termination_e::e_None >   debugPin4(gpio_engine_A, 4);
static gpio::DigitalOutPinT< ::gpio::PinPolicy::Termination_e::e_None >   debugPin5(gpio_engine_A, 5);

static constexpr
std::array
debugPins {
    &debugPin0, //  USB_LP_CAN1_RX0_IRQHandler
    &debugPin1, //  USB Ctrl Pipe -- State Bit #0
    &debugPin2, //  USB Ctrl Pipe -- State Bit #1
    &debugPin3, //  USB Ctrl Pipe -- State Bit #2
    &debugPin4, //  Ctrl OUT Endp -- Data Toggle
    &debugPin5, //  Ctrl IN Endp -- Data Toggle
};

/*******************************************************************************
 * SWO Trace via the Cortex M4 Debug Infrastructure
 ******************************************************************************/
static gpio::AlternateFnPin swo(gpio_engine_B, 3);

static stm32::DbgMcuT<DBGMCU_BASE, decltype(swo)>                   dbgMcu(swo);
static stm32::CoreDbgT<CoreDebug_BASE>                              coreDbg;
static stm32::TpiT<TPI_BASE, decltype(coreDbg), decltype(dbgMcu)>   tpi(coreDbg, dbgMcu);
static stm32::ItmT<ITM_BASE, decltype(tpi)>                         itm(tpi, stm32::Itm::getDivisor(SystemCoreClock, 4'000'000 /*, 2'250'000 */));
static stm32::ItmPort                                               itmPrintf(itm, 8);

/*******************************************************************************
 * UART
 ******************************************************************************/
static gpio::AlternateFnPin             uart_tx(gpio_engine_A, 9);
static gpio::AlternateFnPin             uart_rx(gpio_engine_A, 10);
static stm32::Uart::Usart1<gpio::AlternateFnPin>    uart_access(rcc, uart_rx, uart_tx);
uart::UartDevice                        g_uart(&uart_access);

/*******************************************************************************
 * USB Device
 ******************************************************************************/
static gpio::AlternateFnPin             usb_pin_dm(gpio_engine_A, 11);
static gpio::AlternateFnPin             usb_pin_dp(gpio_engine_A, 12);

alignas(4) stm32::f1::usb::Device::FifoSpace<64>    EP0_BUF USB_MEM;
static_assert(EP0_BUF.size() == 64);                    // Can store 64 Bytes of Data
static_assert(sizeof(EP0_BUF) == 2 * EP0_BUF.size());   // Occupies twice the amount of Address Space 

alignas(4) stm32::f1::usb::Device::FifoSpace<64>    IN_EP1_BUF      USB_MEM;
alignas(4) stm32::f1::usb::Device::FifoSpace<64>    OUT_EP1_BUF     USB_MEM;

static stm32::f1::usb::UsbFullSpeedCoreT<
  decltype(nvic),
  decltype(rcc),
  decltype(usb_pin_dm)
>                                               usbHwDevice(nvic, rcc, usb_pin_dm, usb_pin_dp, /* p_rxFifoSzInWords = */ 256);
static stm32::Usb::CtrlInEndpoint               defaultHwCtrlInEndpoint(usbHwDevice, EP0_BUF.data(), EP0_BUF.size());
static ::usb::UsbCtrlInEndpoint                 defaultCtrlInEndpoint(defaultHwCtrlInEndpoint);

static stm32::Usb::BulkInEndpoint               bulkInHwEndp(usbHwDevice, IN_EP1_BUF.data(), IN_EP1_BUF.size(), 1);
static ::usb::UsbBulkInEndpoint                 bulkInEndp(bulkInHwEndp);

static usb::UsbBulkOutLoopbackApplicationT<512> bulkOutApplication(bulkInEndp);

static usb::UsbBulkOutEndpoint                  bulkOutEndp(bulkOutApplication);
static stm32::Usb::BulkOutEndpoint              bulkOutHwEndp(usbHwDevice, bulkOutEndp, OUT_EP1_BUF.data(), OUT_EP1_BUF.size(), 1);

static usb::UsbVendorInterface                  usbInterface(bulkOutEndp, bulkInEndp);

static usb::UsbConfiguration                    usbConfiguration(usbInterface, *::usb::descriptors::vendor::usbConfigurationDescriptor);
static usb::UsbDevice                           genericUsbDevice(usbHwDevice, ::usb::descriptors::vendor::usbDeviceDescriptor, ::usb::descriptors::vendor::usbStringDescriptors, { &usbConfiguration });

static usb::UsbControlPipe                      defaultCtrlPipe(genericUsbDevice, defaultCtrlInEndpoint);

static usb::UsbCtrlOutEndpoint                  defaultCtrlOutEndp(defaultCtrlPipe);
static stm32::Usb::CtrlOutEndpoint              defaultHwCtrlOutEndp(usbHwDevice, defaultCtrlOutEndp, EP0_BUF.data(), EP0_BUF.size());

/*******************************************************************************
 * Tasks
 ******************************************************************************/
static tasks::HeartbeatT<decltype(g_led_green)> heartbeat_gn("hrtbt_g", g_led_green, 3, 500);

/*******************************************************************************
 *
 ******************************************************************************/
const uint32_t SystemCoreClock = pllCfg.getSysclkSpeedInHz();

static_assert(pllCfg.isValid() == true,                             "PLL Configuration is not valid!");
static_assert(SystemCoreClock               ==  72 * 1000 * 1000,   "Expected System Clock to be at 72 MHz!");
static_assert(pllCfg.getAhbSpeedInHz()      ==  72 * 1000 * 1000,   "Expected AHB to be running at  72 MHz!");
static_assert(pllCfg.getApb1SpeedInHz()     ==  36 * 1000 * 1000,   "Expected APB1 to be running at 36 MHz!");
static_assert(pllCfg.getApb2SpeedInHz()     ==  72 * 1000 * 1000,   "Expected APB2 to be running at 72 MHz!");
static_assert(pllCfg.getUsbSpeedInHz()      ==  48 * 1000 * 1000,   "Expected USB to be running at 48 MHz!");

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#if defined(HOSTBUILD)
int
#else
[[noreturn]]
void
#endif
main(void) {
    for (auto pin : debugPins) {
        pin->set(false);
    }

    /*
     *  If the Board is powered via USB, then the external Pull-up on D+ is "always on". This
     * means the host does not detect the USB device as being re-connected if the µC is restarted
     * e.g. from the Debugger.
     *  We can temporarily configure the D+ µC Pin as an OpenDrain output and pull it towards GND.
     * This will trigger the Host to re-enumerate the USB.
     * 
     *  Once USB Device is enabled, the D+ µC GPIO Pin is directly connected to the Peripheral.
     * See STM32F1 CPU Reference Manual RM0008 Rev 20 on pg. 168 (Sect. 9.1.11, Table 29).
     * Therefore, the USB Hardware must not be started before the USB D+ Pin has been
     * toggled via the Code below.
     *
     *  The Code below should be on a temporary stack frame as that will cause the DigitalOutPinT<>
     * object's destructor to run. The destructor calls the disable() method which will re-set the
     * Pin to its default state (Floating Input).
     */
    {
        const ::gpio::DigitalOutPinT<::gpio::PinPolicy::Termination_e::e_PullUp> usb_rst(gpio_engine_A, 12);
        usb_rst.set(false);
        for (unsigned cnt = 100; cnt != 0; --cnt) __NOP();
    }

    rcc.setMCO(g_mco1, decltype(rcc)::MCOOutput_e::e_HSE, decltype(rcc)::MCOPrescaler_t::e_MCOPre_None);

    uart_access.setBaudRate(decltype(uart_access)::BaudRate_e::e_230400);

    const unsigned sysclk = pllCfg.getSysclkSpeedInHz() / 1000;
    const unsigned ahb    = pllCfg.getAhbSpeedInHz() / 1000;
    const unsigned apb1   = pllCfg.getApb1SpeedInHz() / 1000;
    const unsigned apb2   = pllCfg.getApb2SpeedInHz() / 1000;

    PrintStartupMessage(sysclk, ahb, apb1, apb2);

    /* Inform FreeRTOS about clock speed */
    if (SysTick_Config(SystemCoreClock / configTICK_RATE_HZ)) {
        PHISCH_LOG("FATAL: Capture Error!\r\n");
        goto bad;
    }

    defaultCtrlPipe.start();

    PHISCH_LOG("Starting FreeRTOS Scheduler...\r\n");
    vTaskStartScheduler();

    defaultCtrlPipe.stop();

bad:
    PHISCH_LOG("FATAL ERROR!\r\n");
    while (1) ;

#if defined(HOSTBUILD)
    return (0);
#endif
}

void
debug_printf(const char * const p_fmt, ...) {
    va_list va;
    va_start(va, p_fmt);

    ::tfp_format(&itmPrintf, decltype(itmPrintf)::putf, p_fmt, va);

    va_end(va);
}

void
debug_setpin(const unsigned p_pin, const bool p_value) {
    assert(p_pin < debugPins.size());

    debugPins[p_pin]->set(p_value);
}

/*******************************************************************************
 * Interrupt Handlers
 ******************************************************************************/
void
USB_HP_CAN1_TX_IRQHandler(void) {
    while (1);
}

void
USB_LP_CAN1_RX0_IRQHandler(void) {
    PHISCH_SETPIN(0, true);
    usbHwDevice.handleIrq();
    PHISCH_SETPIN(0, false);
}

void
USBWakeUp_IRQHandler(void) {
    while (1);
}

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined (__cplusplus) */
