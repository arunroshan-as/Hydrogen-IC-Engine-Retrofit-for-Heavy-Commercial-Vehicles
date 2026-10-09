/*
 * stm32_port.c - reference STM32 HAL glue for crank_decoder (NUCLEO-F303RE
 * style target: 72 MHz, TIM2 = 32-bit timer).
 *
 * STATUS: reference port. The decoder core it calls is verified on the host
 * (firmware/tests). This glue file has not been run on hardware in this
 * repository.
 *
 * Timer setup (CubeMX):
 *   TIM2  prescaler 71 -> 1 MHz tick, 32-bit free-running
 *   CH1   input capture, rising edge, crank sensor (60-2 wheel)
 *   CH2   output compare (no pin), drives the event queue below
 *   GPIO  CAM_Pin input (cam sensor), INJ_Pin / IGN_Pin push-pull outputs
 *   SysTick 1 kHz -> HAL_SYSTICK_Callback() for stall polling
 */
#include "stm32f3xx_hal.h"
#include "crank_decoder.h"
#include "timing_map.h"

extern TIM_HandleTypeDef htim2;

#define N_CYL        4
#define QUEUE_LEN    32
#define INJ_WIDTH_US 2000u          /* placeholder pulse width           */
#define IGN_DWELL_US 3000u          /* placeholder coil dwell            */

typedef struct { uint32_t tick; GPIO_TypeDef *port; uint16_t pin; GPIO_PinState level; } q_item_t;

static cd_t        g_dec;
static cd_event_t  g_ev[N_CYL * 2];
static q_item_t    g_q[QUEUE_LEN];
static volatile uint8_t g_qn;
static volatile float   g_load_pct = 60.0f;   /* updated by the load task  */

extern GPIO_TypeDef *INJ_Port[N_CYL], *IGN_Port[N_CYL];
extern const uint16_t INJ_Pin[N_CYL], IGN_Pin[N_CYL];
extern GPIO_TypeDef *CAM_Port; extern const uint16_t CAM_Pin;

static void rearm_compare(void)
{
    if (g_qn == 0) { __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2); return; }
    uint8_t k = 0;                                   /* earliest item        */
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
    for (uint8_t i = 1; i < g_qn; i++) {
        if ((int32_t)(g_q[i].tick - now) < (int32_t)(g_q[k].tick - now)) { k = i; }
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_q[k].tick);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
}

static void q_push(uint32_t tick, GPIO_TypeDef *p, uint16_t pin, GPIO_PinState lvl)
{
    if (g_qn < QUEUE_LEN) { g_q[g_qn++] = (q_item_t){ tick, p, pin, lvl }; }
}

void port_init(void)
{
    cd_config_t cfg;
    cd_default_config(&cfg, 1000000u);
    cd_init(&g_dec, &cfg);
    for (int i = 0; i < N_CYL * 2; i++) { cd_event_init(&g_ev[i]); }
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
}

/* Crank tooth edge. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) { return; }
    uint32_t t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    bool cam = (HAL_GPIO_ReadPin(CAM_Port, CAM_Pin) == GPIO_PIN_SET);
    cd_on_edge(&g_dec, t, cam);

    if (!g_dec.outputs_enabled) { return; }
    float inj = tm_inj_start_btdc(&TM_DEMO_MAP, g_dec.rpm, g_load_pct);
    float spk = tm_spark_btdc(&TM_DEMO_MAP, g_dec.rpm, g_load_pct);
    for (int c = 0; c < N_CYL; c++) {
        float tdc = (float)c * (720.0f / N_CYL);
        uint32_t ft;
        if (cd_event_update(&g_ev[c * 2], &g_dec, cd_btdc_to_cycle(inj) + tdc, &ft)) {
            q_push(ft, INJ_Port[c], INJ_Pin[c], GPIO_PIN_SET);
            q_push(ft + INJ_WIDTH_US, INJ_Port[c], INJ_Pin[c], GPIO_PIN_RESET);
        }
        if (cd_event_update(&g_ev[c * 2 + 1], &g_dec, cd_btdc_to_cycle(spk) + tdc, &ft)) {
            q_push(ft - IGN_DWELL_US, IGN_Port[c], IGN_Pin[c], GPIO_PIN_SET);   /* dwell on  */
            q_push(ft, IGN_Port[c], IGN_Pin[c], GPIO_PIN_RESET);                /* spark     */
        }
    }
    rearm_compare();
}

/* Output-compare: execute every queue item that is due. */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) { return; }
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
    for (uint8_t i = 0; i < g_qn; ) {
        if ((int32_t)(now - g_q[i].tick) >= 0) {
            HAL_GPIO_WritePin(g_q[i].port, g_q[i].pin, g_q[i].level);
            g_q[i] = g_q[--g_qn];
        } else { i++; }
    }
    rearm_compare();
}

/* 1 kHz: stall detection. On stall or sync loss all outputs are forced off. */
void HAL_SYSTICK_Callback(void)
{
    cd_poll(&g_dec, __HAL_TIM_GET_COUNTER(&htim2));
    if (!g_dec.outputs_enabled) {
        g_qn = 0;
        for (int c = 0; c < N_CYL; c++) {
            HAL_GPIO_WritePin(INJ_Port[c], INJ_Pin[c], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IGN_Port[c], IGN_Pin[c], GPIO_PIN_RESET);
        }
    }
}
