/*
 * crank_decoder.h - 60-2 crank trigger-wheel decoder with cam phase,
 * RPM estimation, angle-based event scheduling and a fail-safe output gate.
 *
 * Hardware independent: the decoder consumes (timestamp, cam level) pairs, so
 * the same code runs in an STM32 timer input-capture ISR and in the host
 * test bench under firmware/tests.
 *
 * Conventions
 *   - Timestamps are free-running uint32_t timer ticks (wrap-safe).
 *   - Tooth 0 is the first tooth AFTER the gap. Slot pitch is 6 deg.
 *   - Cycle angle is 0..720 deg; 0 = TDC compression of cylinder 1.
 */
#ifndef CRANK_DECODER_H
#define CRANK_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#define CD_SLOTS_TOTAL    60u   /* 60-2 wheel: 60 slots                    */
#define CD_SLOTS_MISSING   2u
#define CD_TEETH_PRESENT  (CD_SLOTS_TOTAL - CD_SLOTS_MISSING)   /* 58      */
#define CD_LAST_TOOTH     (CD_TEETH_PRESENT - 1u)               /* 57      */
#define CD_DEG_PER_SLOT   6.0f
#define CD_GAP_SLOTS      (CD_SLOTS_MISSING + 1u)  /* gap interval = 3 slots */
#define CD_WINDOW         8u    /* RPM averaging window (edges)            */

typedef enum { CD_SEARCHING = 0, CD_SYNCED = 1 } cd_state_t;

typedef struct {
    uint32_t tick_hz;          /* timer tick frequency                      */
    float    gap_ratio;        /* edge interval / slot estimate above this  */
                               /* is the gap (nominal 3.0, default 2.5)     */
    float    irregular_ratio;  /* interval above this away from the gap     */
                               /* means a dropped tooth (default 1.5)       */
    float    glitch_ratio;     /* interval below this => spurious edge,     */
                               /* rejected (default 0.4)                    */
    uint32_t stall_timeout;    /* ticks without an edge => stall            */
    float    rpm_limit;        /* over-speed cut                            */
    float    rpm_limit_hyst;   /* re-enable below rpm_limit - hyst          */
    float    tdc_offset_deg;   /* angle of tooth 0 relative to TDC (cyl 1)  */
} cd_config_t;

typedef struct {
    cd_config_t cfg;

    cd_state_t state;
    bool     have_prev;
    uint32_t t_last;            /* timestamp of last edge                   */
    uint32_t dt_last;           /* previous edge interval                   */
    uint8_t  tooth_idx;         /* index of last edge, 0..57                */
    uint8_t  phase;             /* 0 or 1: which crank rev of the cycle     */
    bool     gap_seen;          /* a candidate gap has been located         */

    uint32_t win_dt[CD_WINDOW]; /* ring buffer of edge intervals            */
    uint8_t  win_slots[CD_WINDOW];
    uint8_t  win_n, win_head;

    float    rpm;
    float    slot_ticks;        /* estimated ticks per 6 deg slot           */

    bool     overspeed;
    bool     outputs_enabled;   /* fail-safe gate: false => no inj/spark    */

    uint32_t sync_loss_count;
    uint32_t cam_fault_count;
    uint32_t stall_count;
    uint32_t glitch_count;      /* spurious edges rejected                  */
    uint32_t dropout_count;     /* dropped teeth detected                   */
} cd_t;

void  cd_default_config(cd_config_t *cfg, uint32_t tick_hz);
void  cd_init(cd_t *d, const cd_config_t *cfg);

/* Call from the crank input-capture ISR on every tooth edge.
 * cam_level: cam-sensor level sampled at the edge.                         */
void  cd_on_edge(cd_t *d, uint32_t t, bool cam_level);

/* Call periodically (e.g. 1 kHz) for stall detection.                      */
void  cd_poll(cd_t *d, uint32_t now);

/* Cycle angle (0..720 deg) of the most recent tooth.                       */
float cd_tooth_cycle_angle(const cd_t *d);

/* Interpolated cycle angle at time t (t at or after the last edge).        */
float cd_angle_at(const cd_t *d, uint32_t t);

/*
 * Event scheduling. Call after cd_on_edge(). If the requested cycle angle
 * falls inside the interval that follows the last tooth, returns true and
 * writes the timer tick at which the output must fire.
 * Always false while not synced or while outputs are gated off.
 */
bool  cd_schedule(const cd_t *d, float event_cycle_deg, uint32_t *fire_tick);

/*
 * One-shot event latch. A scheduled event must fire once per 720 deg cycle
 * even though its target angle (from the timing map) moves slightly between
 * teeth. cd_event_update() schedules the event at most once per cycle and
 * re-arms it when the crank is at least 180 deg past the event angle.
 */
typedef struct { bool armed; } cd_event_t;

void  cd_event_init(cd_event_t *e);
bool  cd_event_update(cd_event_t *e, const cd_t *d, float event_cycle_deg,
                      uint32_t *fire_tick);

/* Convert "deg before TDC compression" to cycle angle (0..720).            */
float cd_btdc_to_cycle(float deg_btdc);

#endif /* CRANK_DECODER_H */
