/*
 * crank_decoder.c - see crank_decoder.h
 *
 * Synchronisation strategy
 *   1. Locate the gap: an edge interval longer than gap_ratio x the running
 *      slot estimate (nominal ratio 3.0 for a 60-2 wheel).
 *   2. Declare SYNCED only after TWO consecutive gaps that are exactly 58
 *      teeth apart and whose cam levels alternate (720 deg phase check).
 *   3. While SYNCED, every interval is validated:
 *        - shorter than glitch_ratio x slot  -> spurious edge, ignored
 *        - longer than irregular_ratio x slot away from the gap -> dropped
 *          tooth, sync is dropped immediately
 *        - gap missing where tooth 57 -> 0 is due, or present early -> sync lost
 *      Any loss of sync gates the outputs off (fail-safe) until re-acquired.
 */
#include "crank_decoder.h"

#include <math.h>
#include <string.h>

void cd_default_config(cd_config_t *cfg, uint32_t tick_hz)
{
    cfg->tick_hz         = tick_hz;
    cfg->gap_ratio       = 2.5f;
    cfg->irregular_ratio = 1.5f;
    cfg->glitch_ratio    = 0.6f;
    cfg->stall_timeout   = tick_hz / 5u;      /* 200 ms */
    cfg->rpm_limit       = 3200.0f;
    cfg->rpm_limit_hyst  = 200.0f;
    cfg->tdc_offset_deg  = 0.0f;
}

static void reset_window(cd_t *d)
{
    d->win_n = 0;
    d->win_head = 0;
    d->rpm = 0.0f;
    d->slot_ticks = 0.0f;
}

static void update_gate(cd_t *d)
{
    if (d->rpm > d->cfg.rpm_limit) {
        d->overspeed = true;
    } else if (d->rpm < d->cfg.rpm_limit - d->cfg.rpm_limit_hyst) {
        d->overspeed = false;
    }
    d->outputs_enabled = (d->state == CD_SYNCED) && !d->overspeed &&
                         (d->rpm > 0.0f);
}

void cd_init(cd_t *d, const cd_config_t *cfg)
{
    memset(d, 0, sizeof(*d));
    d->cfg = *cfg;
    d->state = CD_SEARCHING;
}

static void drop_sync(cd_t *d)
{
    if (d->state == CD_SYNCED) {
        d->sync_loss_count++;
    }
    d->state = CD_SEARCHING;
    d->gap_seen = false;
    d->dt_last = 0;               /* previous interval is no longer trusted */
    reset_window(d);
    update_gate(d);
}

static void window_push(cd_t *d, uint32_t dt, uint8_t slots)
{
    d->win_dt[d->win_head]    = dt;
    d->win_slots[d->win_head] = slots;
    d->win_head = (uint8_t)((d->win_head + 1u) % CD_WINDOW);
    if (d->win_n < CD_WINDOW) {
        d->win_n++;
    }
    if (d->win_n >= 4u) {
        uint32_t sum_dt = 0;
        uint32_t sum_slots = 0;
        for (uint8_t i = 0; i < d->win_n; i++) {
            sum_dt    += d->win_dt[i];
            sum_slots += d->win_slots[i];
        }
        if (sum_dt > 0u) {
            d->slot_ticks = (float)sum_dt / (float)sum_slots;
            d->rpm = (float)d->cfg.tick_hz * (float)sum_slots / (float)sum_dt;
        }
    }
}

void cd_on_edge(cd_t *d, uint32_t t, bool cam_level)
{
    if (!d->have_prev) {
        d->have_prev = true;
        d->t_last = t;
        d->dt_last = 0;
        return;
    }

    uint32_t dt = t - d->t_last;               /* wrap-safe */
    bool synced = (d->state == CD_SYNCED);

    /* Reference interval. While searching, the previous interval is used
     * (a gap is >2.5x it, a normal tooth never is). Once synced, the
     * averaged slot period is available and all intervals are validated. */
    float ref = (synced && d->slot_ticks > 0.0f) ? d->slot_ticks
                                                 : (float)d->dt_last;

    /* ---- glitch filter (synced only) ----------------------------------- */
    if (synced && ref > 0.0f && (float)dt < d->cfg.glitch_ratio * ref) {
        d->glitch_count++;
        return;                                 /* edge ignored entirely */
    }

    bool is_gap = (ref > 0.0f) && ((float)dt > d->cfg.gap_ratio * ref);
    bool long_non_gap = synced && (ref > 0.0f) && !is_gap &&
                        ((float)dt > d->cfg.irregular_ratio * ref);
    bool observed_phase = cam_level;

    d->t_last = t;
    d->dt_last = dt;

    if (is_gap) {
        window_push(d, dt, (uint8_t)CD_GAP_SLOTS);

        if (d->gap_seen && d->tooth_idx == CD_LAST_TOOTH) {
            /* Valid gap: exactly 58 teeth since the previous one. */
            uint8_t expected = (uint8_t)(d->phase ^ 1u);
            if (observed_phase == (bool)expected) {
                d->phase = expected;
                d->state = CD_SYNCED;
            } else {
                d->cam_fault_count++;
                if (d->state == CD_SYNCED) {
                    d->sync_loss_count++;
                }
                d->state = CD_SEARCHING;
                d->phase = observed_phase ? 1u : 0u;
            }
        } else {
            /* Early/late gap or first gap seen: restart the candidate. */
            if (d->state == CD_SYNCED) {
                d->sync_loss_count++;
            }
            d->state = CD_SEARCHING;
            d->phase = observed_phase ? 1u : 0u;
        }
        d->gap_seen = true;
        d->tooth_idx = 0;
        update_gate(d);
        return;
    }

    /* ---- ordinary tooth ------------------------------------------------ */
    if (long_non_gap) {
        d->dropout_count++;
        drop_sync(d);                           /* dropped tooth */
        return;
    }

    window_push(d, dt, 1u);

    if (d->gap_seen) {
        d->tooth_idx++;
        if (d->tooth_idx > CD_LAST_TOOTH) {
            /* A gap was due here and did not arrive. */
            drop_sync(d);
            return;
        }
    }
    update_gate(d);
}

void cd_poll(cd_t *d, uint32_t now)
{
    /* Signed age: an ISR may stamp an edge slightly later than the 'now'
     * sampled by a poll that ran just before it, so a small negative age is
     * legitimate and must not be mistaken for a huge unsigned one. */
    int32_t age = (int32_t)(now - d->t_last);
    if (d->have_prev && age > 0 && (uint32_t)age > d->cfg.stall_timeout) {
        d->stall_count++;
        d->have_prev = false;
        d->gap_seen = false;
        d->state = CD_SEARCHING;
        d->overspeed = false;
        reset_window(d);
        update_gate(d);
    }
}

static float wrap720(float a)
{
    a = fmodf(a, 720.0f);
    if (a < 0.0f) {
        a += 720.0f;
    }
    return a;
}

float cd_tooth_cycle_angle(const cd_t *d)
{
    return wrap720((float)d->phase * 360.0f +
                   (float)d->tooth_idx * CD_DEG_PER_SLOT +
                   d->cfg.tdc_offset_deg);
}

float cd_angle_at(const cd_t *d, uint32_t t)
{
    if (d->slot_ticks <= 0.0f) {
        return cd_tooth_cycle_angle(d);
    }
    float elapsed = (float)(uint32_t)(t - d->t_last);
    return wrap720(cd_tooth_cycle_angle(d) +
                   CD_DEG_PER_SLOT * elapsed / d->slot_ticks);
}

bool cd_schedule(const cd_t *d, float event_cycle_deg, uint32_t *fire_tick)
{
    if (!d->outputs_enabled || d->slot_ticks <= 0.0f) {
        return false;
    }
    float a0   = cd_tooth_cycle_angle(d);
    float span = (d->tooth_idx == CD_LAST_TOOTH)
                     ? (float)CD_GAP_SLOTS * CD_DEG_PER_SLOT
                     : CD_DEG_PER_SLOT;
    float rel = wrap720(event_cycle_deg - a0);

    if (rel > 0.0f && rel <= span) {
        *fire_tick = d->t_last +
                     (uint32_t)(rel / CD_DEG_PER_SLOT * d->slot_ticks + 0.5f);
        return true;
    }
    return false;
}

void cd_event_init(cd_event_t *e)
{
    e->armed = true;
}

bool cd_event_update(cd_event_t *e, const cd_t *d, float event_cycle_deg,
                     uint32_t *fire_tick)
{
    if (d->state != CD_SYNCED) {
        return false;
    }
    if (!e->armed) {
        float behind = wrap720(cd_tooth_cycle_angle(d) - event_cycle_deg);
        if (behind >= 180.0f && behind < 540.0f) {
            e->armed = true;
        }
        return false;
    }
    if (cd_schedule(d, event_cycle_deg, fire_tick)) {
        e->armed = false;
        return true;
    }
    return false;
}

float cd_btdc_to_cycle(float deg_btdc)
{
    return wrap720(720.0f - deg_btdc);
}
