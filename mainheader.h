#include <LPC17xx.h>

#define Fpclk 25e6	// Fcpu/4 (defecto despu?s del reset)
#define Tpwm 20e-3	// Perido de la se?al PWM (15ms)

#define MIN 0
#define MAX 180

void config_pwm2(void);
void config_EINT1(void);
void set_servo(float);