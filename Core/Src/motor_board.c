#include "motor_board.h"
#include "motor_board_config.h"

static const GPIO_pin_t direction_pins[] = {
    {MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN}, {MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN},
    {MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN}, {MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN},
    {MOTOR_STBY_PORT, MOTOR_STBY_PIN},
};
static bool hardware_ready;
static void driver_enable(bool enable)
{
    if (hardware_ready) GPIO_Write(direction_pins[4],enable);
}
static const MotorAppConfig board_config = {
    .wheels = {
        {.direction1={MOTOR_LEFT_IN1_PORT,MOTOR_LEFT_IN1_PIN},
         .direction2={MOTOR_LEFT_IN2_PORT,MOTOR_LEFT_IN2_PIN},
         .pwm={TIM3,MOTOR_LEFT_PWM_CHANNEL}, .encoder_timer=MOTOR_LEFT_ENCODER_TIMER,
         .counts_per_rev=MOTOR_COUNTS_PER_REV,
         .output_reversed=MOTOR_LEFT_OUTPUT_REVERSED, .encoder_reversed=MOTOR_LEFT_ENCODER_REVERSED,
         .kp=MOTOR_LEFT_KP, .ki=MOTOR_LEFT_KI, .kd=MOTOR_LEFT_KD, .incremental_pid=MOTOR_INCREMENTAL_PID},
        {.direction1={MOTOR_RIGHT_IN1_PORT,MOTOR_RIGHT_IN1_PIN},
         .direction2={MOTOR_RIGHT_IN2_PORT,MOTOR_RIGHT_IN2_PIN},
         .pwm={TIM3,MOTOR_RIGHT_PWM_CHANNEL}, .encoder_timer=MOTOR_RIGHT_ENCODER_TIMER,
         .counts_per_rev=MOTOR_COUNTS_PER_REV,
         .output_reversed=MOTOR_RIGHT_OUTPUT_REVERSED, .encoder_reversed=MOTOR_RIGHT_ENCODER_REVERSED,
         .kp=MOTOR_RIGHT_KP, .ki=MOTOR_RIGHT_KI, .kd=MOTOR_RIGHT_KD, .incremental_pid=MOTOR_INCREMENTAL_PID},
    },
    .duty_limit=MOTOR_DUTY_LIMIT, .rpm_limit=MOTOR_RPM_LIMIT,
    .forward_in1=true, .forward_in2=false, .reverse_in1=false, .reverse_in2=true,
    .idle_in1=true, .idle_in2=true, /* PWM 零期间为短路制动；总停机时 STBY 拉低。 */
    .speed_control_allowed=MOTOR_SPEED_CONTROL_ALLOWED,
    .set_driver_enabled=driver_enable,
};
static bool mapping_valid(void)
{
    for (unsigned i=0; i<5; ++i) {
        GPIO_pin_t p=direction_pins[i];
        if (!GPIO_PinValid(p)) return false;
        if (!((p.port==GPIOA && (p.pin==GPIO_PIN_11 || p.pin==GPIO_PIN_12 || p.pin==GPIO_PIN_15)) ||
              (p.port==GPIOB && (p.pin==GPIO_PIN_3 || p.pin==GPIO_PIN_4)))) return false;
        for (unsigned j=0; j<i; ++j)
            if (p.port==direction_pins[j].port && p.pin==direction_pins[j].pin) return false;
    }
    unsigned l=board_config.wheels[0].pwm.channel, r=board_config.wheels[1].pwm.channel;
    if (!((l==3 && r==4) || (l==4 && r==3))) return false;
    TIM_TypeDef *a=board_config.wheels[0].encoder_timer, *b=board_config.wheels[1].encoder_timer;
    return (a==TIM1 && b==TIM4) || (a==TIM4 && b==TIM1);
}
static void encoder_setup(TIM_TypeDef *timer)
{
    timer->CR1=0; timer->PSC=0; timer->ARR=65535;
    // TI1/TI2 直接输入，数字滤波值 6，编码器模式 3：双通道双边沿四倍频。
    timer->CCMR1=TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | (6u<<4) | (6u<<12);
    timer->CCER=TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->SMCR=TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    timer->EGR=TIM_EGR_UG; timer->CNT=0; timer->SR=0;
    timer->CR1=TIM_CR1_CEN;
}
const MotorAppConfig *MotorBoard_Init(void)
{
    if (!MOTOR_BOARD_READY || !mapping_valid()) return NULL;
    uint32_t timer_clock=HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1)!=0) timer_clock*=2;
    if (!MOTOR_PWM_HZ || timer_clock % MOTOR_PWM_HZ || timer_clock/MOTOR_PWM_HZ<2 ||
        timer_clock/MOTOR_PWM_HZ>65535) return NULL;
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); // 释放 PA15/PB3/PB4，保留 SWD。
    GPIO_InitTypeDef gpio={0}; gpio.Mode=GPIO_MODE_OUTPUT_PP; gpio.Speed=GPIO_SPEED_FREQ_HIGH;
    // 先配置待机脚并保持低电平，再触碰方向和 PWM。
    GPIO_Write(direction_pins[4],false);
    gpio.Pin=direction_pins[4].pin; HAL_GPIO_Init(direction_pins[4].port,&gpio);
    hardware_ready=true; driver_enable(false);
    for (unsigned i=0; i<4; ++i) {
        GPIO_Write(direction_pins[i],true);
        gpio.Pin=direction_pins[i].pin; HAL_GPIO_Init(direction_pins[i].port,&gpio);
    }
    __HAL_RCC_TIM1_CLK_ENABLE(); __HAL_RCC_TIM4_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM1_FORCE_RESET(); __HAL_RCC_TIM1_RELEASE_RESET();
    __HAL_RCC_TIM4_FORCE_RESET(); __HAL_RCC_TIM4_RELEASE_RESET();
    __HAL_RCC_TIM3_FORCE_RESET(); __HAL_RCC_TIM3_RELEASE_RESET();
    gpio.Mode=GPIO_MODE_INPUT; gpio.Pull=GPIO_PULLUP;
    gpio.Pin=GPIO_PIN_8|GPIO_PIN_9; HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_6|GPIO_PIN_7; HAL_GPIO_Init(GPIOB,&gpio);
    encoder_setup(TIM1); encoder_setup(TIM4);
    TIM3->PSC=0; TIM3->ARR=timer_clock/MOTOR_PWM_HZ-1;
    TIM3->CCR3=0; TIM3->CCR4=0;
    TIM3->CCMR2=TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
    // 不启用 CCR 预装载，确保换向前写零立即作用于比较值。
    TIM3->CCER=TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->EGR=TIM_EGR_UG; TIM3->SR=0;
    gpio.Mode=GPIO_MODE_AF_PP; gpio.Pull=GPIO_NOPULL; gpio.Pin=GPIO_PIN_0|GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB,&gpio);
    TIM3->CR1=TIM_CR1_CEN;
    return &board_config;
}
