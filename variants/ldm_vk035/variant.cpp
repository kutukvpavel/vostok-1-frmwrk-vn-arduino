#include "variant.h"
#include "api/Common.h"

/*
 * Pins descriptions
 */
const PinDescription pins_description_map[] =
{
        {GPIOA, GPIO_Pin_0, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},       //   0 / PA0 / I2C_SCL
        {GPIOA, GPIO_Pin_1, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},       //   1 / PA1 / I2C_SDA
        {GPIOA, GPIO_Pin_2, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},       //   2 / PA2 / JTAG_TCK
        {GPIOA, GPIO_Pin_3, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},       //   3 / PA3 / JTAG_TMS
        {GPIOA, GPIO_Pin_4, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},       //   4 / PA4 / ECAP0
        {GPIOA, GPIO_Pin_5, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},       //   5 / PA5 / JTAG_TDO
        {GPIOA, GPIO_Pin_6, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},       //   6 / PA6
        {GPIOA, GPIO_Pin_7, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},       //   7 / PA7 / BOOT_EN / uKEY
        {GPIOA, GPIO_Pin_8, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_CH_NUM(0)},  //   8 / PA8 / PWM_0_A / uLED
        {GPIOA, GPIO_Pin_9, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_CH_NUM(1)},  //   9 / PA9 / PWM_0_B
        {GPIOA, GPIO_Pin_10, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_CH_NUM(2)}, //   10 / PA10 / PWM_1_A
        {GPIOA, GPIO_Pin_11, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_CH_NUM(3)}, //   11 / PA11 / PWM_1_B
        {GPIOA, GPIO_Pin_12, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_CH_NUM(4)}, //   12 / PA12 / PWM_2_A
        {GPIOA, GPIO_Pin_13, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_CH_NUM(5)}, //   13 / PA13 / PWM_1_B
        {GPIOA, GPIO_Pin_14, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},      //   14 / PA14
        {GPIOA, GPIO_Pin_15, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},      //   15 / PA15

        {GPIOB, GPIO_Pin_0, (PIN_ATTR_NONE), PIN_ADC_CH_NUM(0), PIN_PWM_NONE}, //   16 / PB0 / ADC_0 / A0
        {GPIOB, GPIO_Pin_1, (PIN_ATTR_NONE), PIN_ADC_CH_NUM(1), PIN_PWM_NONE}, //   17 / PB1 / ADC_1 / A1
        {GPIOB, GPIO_Pin_2, (PIN_ATTR_NONE), PIN_ADC_CH_NUM(2), PIN_PWM_NONE}, //   18 / PB2 / ADC_2 / A2
        {GPIOB, GPIO_Pin_3, (PIN_ATTR_NONE), PIN_ADC_CH_NUM(3), PIN_PWM_NONE}, //   19 / PB3 / ADC_3 / A3
        {GPIOB, GPIO_Pin_4, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},      //   20 / PB4 / SPI_CS
        {GPIOB, GPIO_Pin_5, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},      //   21 / PB5 / SPI_SCK
        {GPIOB, GPIO_Pin_6, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},      //   22 / PB6 / SPI_MISO
        {GPIOB, GPIO_Pin_7, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},      //   23 / PB7 / SPI_MOSI
        {GPIOB, GPIO_Pin_8, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},      //   24 / PB8 / UART1 TX
        {GPIOB, GPIO_Pin_9, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},      //   25 / PB9 / UART1 RX
        {GPIOB, GPIO_Pin_10, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},     //   26 / PB10 / UART0 TX
        {GPIOB, GPIO_Pin_11, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},     //   27 / PB11 / UART0 RX
        {GPIOB, GPIO_Pin_12, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},     //   28 / PB12 / CAN1RX
        {GPIOB, GPIO_Pin_13, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},     //   29 / PB13 / CAN1TX
        {GPIOB, GPIO_Pin_14, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE},     //   30 / PB14
        {GPIOB, GPIO_Pin_15, (PIN_ATTR_NONE), PIN_ADC_NONE, PIN_PWM_NONE}      //   31 / PB15
};

arduino::UartSerial Serial(UART1, PIN_UART_TX1, PIN_UART_RX1);
arduino::UartSerial Serial1(UART0, PIN_UART_TX0, PIN_UART_RX0);
extern "C"
{
  void UART0_RX_IRQHandler()
  {
    Serial1.IrqHandlerRx();
  }
  void UART0_TX_IRQHandler()
  {
    Serial1.IrqHandlerTx();
  }
  void UART0_E_RT_IRQHandler()
  {
    Serial1.IrqHandlerRxTimeout();
  }

  void UART1_RX_IRQHandler()
  {
    Serial.IrqHandlerRx();
  }
  void UART1_TX_IRQHandler()
  {
    Serial.IrqHandlerTx();
  }
  void UART1_E_RT_IRQHandler()
  {
    Serial.IrqHandlerRxTimeout();
  }

  pin_size_t pin_get_description_with_pwm(pin_size_t pin_num)
  {
    pin_size_t pwm_pin = pin_num;
    if (pins_description_map[pin_num].pwm_ch != PIN_PWM_NONE)
    {
      return pin_num;
    }
    return pwm_pin;
  }

  void initVariant()
  {
    GPIO_LockKeyCmd(GPIOA, ENABLE);
    WRITE_REG(GPIOA->LOCKCLR, GPIO_Pin_4 | GPIO_Pin_6); // DISABLE LOCK at JTAG_TRST and JTAG_TDI, need wait 2 CLK periods for unlock
    WRITE_REG(GPIOA->LOCKCLR, GPIO_Pin_4 | GPIO_Pin_6); // DISABLE LOCK at JTAG_TRST and JTAG_TDI
    WRITE_REG(GPIOA->LOCKCLR, GPIO_Pin_4 | GPIO_Pin_6); // DISABLE LOCK at JTAG_TRST and JTAG_TDI
    GPIO_LockKeyCmd(GPIOA, DISABLE);
  }
}

void digital_pin_use_hook(pin_size_t pin_num)
{
  
}

void analog_pin_use_hook(pin_size_t pin_num)
{
  
}
