#include "Arduino.h"
#include "k1921vk_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

void pinMode( pin_size_t ulPin, PinMode ulMode ){
  const PinDescription *pin_description = PIN_GET_DESCRIPTION(ulPin);
  if(pin_description == NULL){
    return;
  }

  GPIO_Init_TypeDef gpio_init_struct;
  GPIO_StructInit(&gpio_init_struct);

  #ifdef MCU_K1921VK035
    gpio_init_struct.Pin = pin_description->pin_msk;
    gpio_init_struct.Out = ulMode  == OUTPUT || ulMode  == OUTPUT_OPENDRAIN ? ENABLE : DISABLE;
    gpio_init_struct.AltFunc = DISABLE;
    gpio_init_struct.OutMode = ulMode  == OUTPUT_OPENDRAIN ? GPIO_OutMode_OD : GPIO_OutMode_PP;
    gpio_init_struct.InMode = GPIO_InMode_Schmitt;
    gpio_init_struct.PullMode = ulMode == INPUT_PULLUP ?   GPIO_PullMode_PU:
                                ulMode == INPUT_PULLDOWN ? GPIO_PullMode_PD:
                                                           GPIO_PullMode_Disable;
    gpio_init_struct.DriveMode = GPIO_DriveMode_HighFast;
    gpio_init_struct.Digital = ENABLE;
  #elif MCU_K1921VK01T
    gpio_init_struct.GPIO_AltFunc = GPIO_AltFunc_1;
    gpio_init_struct.GPIO_Dir = ulMode  == OUTPUT || ulMode  == OUTPUT_OPENDRAIN ? GPIO_Dir_Out : GPIO_Dir_In;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IO;
    gpio_init_struct.GPIO_Out = ulMode  == OUTPUT || ulMode  == OUTPUT_OPENDRAIN ? GPIO_Out_En : GPIO_Out_Dis;
    gpio_init_struct.GPIO_OutMode =  ulMode  == OUTPUT_OPENDRAIN ? GPIO_OutMode_OD : GPIO_OutMode_PP;
    gpio_init_struct.GPIO_PullUp = ulMode == INPUT_PULLUP ?   GPIO_PullUp_En:
                                   ulMode == INPUT_PULLDOWN ? GPIO_PullUp_Dis:
                                                           GPIO_PullUp_Dis;;
    gpio_init_struct.GPIO_Pin = pin_description->pin_msk;
  #endif
  GPIO_Init(pin_description->port,&gpio_init_struct);
  return;
}

void digitalWrite( pin_size_t ulPin, PinStatus ulVal )
{

  const PinDescription *pin_description = PIN_GET_DESCRIPTION(ulPin);
  if(pin_description == NULL){
    return;
  }
  if((pin_description->pin_attribute & PIN_ATTR_NEED_LS_CTRL) == PIN_ATTR_NEED_LS_CTRL){
    pinMode(adc_ls_ctrl_map[pin_description->adc_ch], OUTPUT);
    digitalWrite(adc_ls_ctrl_map[pin_description->adc_ch],HIGH);
  }
  GPIO_WriteBit(pin_description->port, pin_description->pin_msk,ulVal);
  return;
}

PinStatus digitalRead( pin_size_t ulPin )
{
  const PinDescription *pin_description = PIN_GET_DESCRIPTION(ulPin);
  if(pin_description == NULL){
    return LOW;
  }
  if((pin_description->pin_attribute & PIN_ATTR_NEED_LS_CTRL) == PIN_ATTR_NEED_LS_CTRL){
    pinMode(adc_ls_ctrl_map[pin_description->adc_ch], OUTPUT);
    digitalWrite(adc_ls_ctrl_map[pin_description->adc_ch],HIGH);
  }
  return GPIO_ReadBit(pin_description->port, pin_description->pin_msk) ? HIGH: LOW;
}

#ifdef __cplusplus
}
#endif
