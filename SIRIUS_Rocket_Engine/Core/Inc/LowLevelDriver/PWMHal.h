#pragma once

#include "../../sirius-embedded-common/Inc/LowLevelDriver/PWM/PWM.h"

extern void PWMHAL_init(PWM* instance);

extern void PWMHAL_setDutyCycle(PWM* instance);