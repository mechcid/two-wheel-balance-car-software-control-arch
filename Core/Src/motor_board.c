#include "motor_board.h"
#include "motor_board_config.h"
#include "tim.h"

static const GPIO_pin_t direction_pins[] = {
    {MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN}, {MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN},
    {MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN}, {MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN},
    {MOTOR_STBY_PORT, MOTOR_STBY_PIN},
};
/* 板级持有编码器，Motor 只引用；静态存储覆盖整个应用生命周期。 */
static Encoder left_encoder = {.timer=MOTOR_LEFT_ENCODER_TIMER};
static Encoder right_encoder = {.timer=MOTOR_RIGHT_ENCODER_TIMER};
static bool hardware_ready;
static void driver_enable(bool enable)
{
    if (hardware_ready) GPIO_Write(direction_pins[4],enable);
}
static const MotorAppConfig board_config = {
    .left = {.direction1={MOTOR_LEFT_IN1_PORT,MOTOR_LEFT_IN1_PIN},
         .direction2={MOTOR_LEFT_IN2_PORT,MOTOR_LEFT_IN2_PIN},
         .pwm={TIM3,MOTOR_LEFT_PWM_CHANNEL}, .encoder=&left_encoder,
         .counts_per_output_rev=MOTOR_COUNTS_PER_REV, .radius_m=MOTOR_WHEEL_RADIUS_M,
         .output_reversed=MOTOR_LEFT_OUTPUT_REVERSED, .encoder_reversed=MOTOR_LEFT_ENCODER_REVERSED,
         .duty_limit=MOTOR_DUTY_LIMIT,
         .forward_in1=true, .forward_in2=false, .reverse_in1=false, .reverse_in2=true,
         .idle_in1=true, .idle_in2=true},
    .right = {.direction1={MOTOR_RIGHT_IN1_PORT,MOTOR_RIGHT_IN1_PIN},
         .direction2={MOTOR_RIGHT_IN2_PORT,MOTOR_RIGHT_IN2_PIN},
         .pwm={TIM3,MOTOR_RIGHT_PWM_CHANNEL}, .encoder=&right_encoder,
         .counts_per_output_rev=MOTOR_COUNTS_PER_REV, .radius_m=MOTOR_WHEEL_RADIUS_M,
         .output_reversed=MOTOR_RIGHT_OUTPUT_REVERSED, .encoder_reversed=MOTOR_RIGHT_ENCODER_REVERSED,
         .duty_limit=MOTOR_DUTY_LIMIT,
         .forward_in1=true, .forward_in2=false, .reverse_in1=false, .reverse_in2=true,
         .idle_in1=true, .idle_in2=true},
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
    unsigned l=board_config.left.pwm.channel, r=board_config.right.pwm.channel;
    if (!((l==3 && r==4) || (l==4 && r==3))) return false;
    TIM_TypeDef *a=board_config.left.encoder->timer, *b=board_config.right.encoder->timer;
    return (a==TIM1 && b==TIM4) || (a==TIM4 && b==TIM1);
}
/* 配置只由 MX_TIMx_Init 完成；这里检查关键运行约定，不覆盖 PSC/ARR。 */
static bool hardware_valid(void)
{
    return htim1.Instance==TIM1 && htim4.Instance==TIM4 && htim3.Instance==TIM3 &&
        TIM1->ARR==65535 && TIM4->ARR==65535 && TIM1->PSC==0 && TIM4->PSC==0 &&
        (TIM1->SMCR & TIM_SMCR_SMS)==TIM_ENCODERMODE_TI12 &&
        (TIM4->SMCR & TIM_SMCR_SMS)==TIM_ENCODERMODE_TI12 &&
        PWM_PinValid(board_config.left.pwm) &&
        PWM_PinValid(board_config.right.pwm) &&
        !(TIM3->CR1 & (TIM_CR1_DIR | TIM_CR1_CMS)) &&
        (TIM3->CCMR2 & (TIM_CCMR2_OC3M | TIM_CCMR2_OC4M))==
            (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2) &&
        !(TIM3->CCMR2 & (TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE)) &&
        !(TIM3->CCER & (TIM_CCER_CC3P | TIM_CCER_CC4P));
}
const MotorAppConfig *MotorBoard_Init(void)
{
    static bool attempted;
    if (attempted || !MOTOR_BOARD_READY || !mapping_valid()) return NULL;
    attempted=true;
    /* MX_GPIO_Init 已配置方向/STBY，HAL_MspInit 已释放 JTAG 引脚。 */
    hardware_ready=true;
    driver_enable(false);
    if (!hardware_valid()) return NULL;
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
    if (HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL)!=HAL_OK ||
        HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL)!=HAL_OK ||
        HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3)!=HAL_OK ||
        HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4)!=HAL_OK) {
        HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
        HAL_TIM_Encoder_Stop(&htim1,TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Stop(&htim4,TIM_CHANNEL_ALL);
        return NULL;
    }
    return &board_config;
}
