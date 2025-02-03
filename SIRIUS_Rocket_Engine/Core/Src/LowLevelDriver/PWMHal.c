#include "../../Inc/LowLevelDriver/PWMHAL.h"

void PWMHAL_init(PWM* instance) {
  instance->status.value = 0;
  instance->errorStatus.value = 0;
  TIM_HandleTypeDef halHandle = (TIM_HandleTypeDef)instance->externalInstance;

  HAL_TIM_Base_Stop_IT(&halHandle);
  __HAL_TIM_SET_PRESCALER(&halHandle, prescale);
  if(HAL_TIM_Base_Start_IT(&halHandle) != HAL_OK) {
      return;
  }

  HAL_TIM_PWM_Start(&halHandle, instance->channel);

  instance->setDutyCycle(&instance, instance->minDutyCycle);
}

void PWMHAL_setDutyCycle(PWM* instance, uint8_t dutyCycle_pct) {
  
}