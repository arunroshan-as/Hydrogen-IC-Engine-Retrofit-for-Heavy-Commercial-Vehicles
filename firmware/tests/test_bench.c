/*
 * test_bench.c - host-side verification of the crank decoder.
 *
 * A virtual 4-cylinder engine with a 60-2 crank wheel and a cam sensor is
 * integrated in 1 us steps. Every tooth edge is time-stamped with Gaussian
 * jitter, quantised to the 1 MHz timer, and fed to the decoder exactly as an
 * input-capture ISR would. The bench then compares what the decoder believes
 * (RPM, crank angle, scheduled injection/spark instants) against the exact
 * simulated truth.
 *
 * Scenarios
 *   A  start-up, idle, ramps and a hard acceleration; timer wraps mid-run
 *   B  fault injection: dropped tooth, short glitch, mid-length glitch,
 *      inverted cam signal, sensor-signal loss (stall)
 *   C  over-speed gating with hysteresis
 *
 * Exit status is non-zero if any check fails.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "crank_decoder.h"
#include "timing_map.h"

/* --------------------------------------------------------------------- */
/* deterministic RNG                                                      */
/* --------------------------------------------------------------------- */
static uint64_t rng_s = 88172645463325252ULL;

static double rnd_u(void)
{
    rng_s ^= rng_s << 13;
    rng_s ^= rng_s >> 7;
    rng_s ^= rng_s << 17;
    return (double)(rng_s >> 11) * (1.0 / 9007199254740992.0);
}

static double rnd_n(void)
{
    double u1 = rnd_u();
    double u2 = rnd_u();
    if (u1 < 1e-12) {
        u1 = 1e-12;
    }
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

/* --------------------------------------------------------------------- */
/* scenario description                                                   */
/* --------------------------------------------------------------------- */
typedef struct { double t_s; double rpm; } pt_t;

typedef struct {
    const char *name;
    double      duration_s;
    const pt_t *prof;
    int         nprof;
    double      load_pct;
    double      jitter_frac;     /* sigma as fraction of the slot period   */
    uint32_t    t0_ticks;        /* timer value at t = 0 (wrap testing)    */

    double drop_at_s[4];   int n_drop;
    double spur_at_s[4];   double spur_frac[4]; int n_spur;
    double cam_inv_from_s, cam_inv_to_s;       /* cam stuck high; < 0: none */
    double stall_from_s,   stall_to_s;         /* < 0: none                */
} scenario_t;

static double rpm_at(const scenario_t *sc, double t)
{
    if (t <= sc->prof[0].t_s) {
        return sc->prof[0].rpm;
    }
    for (int i = 1; i < sc->nprof; i++) {
        if (t <= sc->prof[i].t_s) {
            double f = (t - sc->prof[i - 1].t_s) /
                       (sc->prof[i].t_s - sc->prof[i - 1].t_s);
            return sc->prof[i - 1].rpm + f * (sc->prof[i].rpm - sc->prof[i - 1].rpm);
        }
    }
    return sc->prof[sc->nprof - 1].rpm;
}

static double accel_at(const scenario_t *sc, double t)    /* rpm per second */
{
    return (rpm_at(sc, t + 0.001) - rpm_at(sc, t)) / 0.001;
}

/* --------------------------------------------------------------------- */
/* statistics helpers                                                     */
/* --------------------------------------------------------------------- */
typedef struct {
    double *v;
    size_t  n, cap;
} vec_t;

static void vec_push(vec_t *a, double x)
{
    if (a->n == a->cap) {
        a->cap = a->cap ? a->cap * 2 : 1024;
        a->v = (double *)realloc(a->v, a->cap * sizeof(double));
    }
    a->v[a->n++] = x;
}

static int cmp_abs(const void *a, const void *b)
{
    double x = fabs(*(const double *)a), y = fabs(*(const double *)b);
    return (x > y) - (x < y);
}

typedef struct { size_t n; double mean, sd, p99, maxabs; } stats_t;

static stats_t vec_stats(vec_t *a)
{
    stats_t s = {0, 0, 0, 0, 0};
    if (a->n == 0) {
        return s;
    }
    double sum = 0, sq = 0;
    for (size_t i = 0; i < a->n; i++) {
        sum += a->v[i];
        sq  += a->v[i] * a->v[i];
    }
    s.n = a->n;
    s.mean = sum / (double)a->n;
    s.sd = sqrt(fmax(0.0, sq / (double)a->n - s.mean * s.mean));
    qsort(a->v, a->n, sizeof(double), cmp_abs);
    s.maxabs = fabs(a->v[a->n - 1]);
    s.p99 = fabs(a->v[(size_t)(0.99 * (double)(a->n - 1))]);
    return s;
}

/* --------------------------------------------------------------------- */
/* check bookkeeping                                                      */
/* --------------------------------------------------------------------- */
static int g_checks = 0, g_fails = 0;

#define CHECK(cond, ...)                                  \
    do {                                                  \
        g_checks++;                                       \
        if (cond) {                                       \
            printf("  PASS  ");                           \
        } else {                                          \
            printf("  FAIL  ");                           \
            g_fails++;                                    \
        }                                                 \
        printf(__VA_ARGS__);                              \
        printf("\n");                                     \
    } while (0)

/* --------------------------------------------------------------------- */
/* simulation                                                             */
/* --------------------------------------------------------------------- */
#define N_CYL    4
#define N_EVENTS (N_CYL * 2)      /* injection + spark per cylinder */
#define MAX_PEND 64

typedef struct {
    double   target_theta;
    uint32_t fire_tick;
    int      id;
    bool     transient;
} pending_t;

typedef struct {
    double   t_lost, t_back;       /* seconds */
    double   revs;
} resync_t;

typedef struct {
    /* outputs */
    stats_t rpm_err_steady, rpm_err_trans, rpm_err_runup;
    stats_t ev_err_steady, ev_err_trans, ev_err_all;
    size_t  events_total, events_gt1deg, events_gt2deg;
    double  first_sync_revs;
    resync_t resync[16];
    int      n_resync;
    double   gate_off_after_sync_s;
    size_t   gated_while_over_limit_violations;
    cd_t     dec;
} result_t;

static double wrap_pm(double a, double period)
{
    a = fmod(a, period);
    if (a >= period / 2) {
        a -= period;
    }
    if (a < -period / 2) {
        a += period;
    }
    return a;
}

static bool cam_level_at(double cyc_deg)
{
    return cyc_deg >= 90.0 && cyc_deg < 450.0;
}

static void run(const scenario_t *sc, result_t *res, const char *csv_prefix)
{
    memset(res, 0, sizeof(*res));

    cd_config_t cfg;
    cd_default_config(&cfg, 1000000u);
    cd_t dec;
    cd_init(&dec, &cfg);
    cd_event_t ev[N_EVENTS];
    for (int i = 0; i < N_EVENTS; i++) {
        cd_event_init(&ev[i]);
    }

    char path[256];
    snprintf(path, sizeof path, "%s_edges.csv", csv_prefix);
    FILE *fe = fopen(path, "w");
    snprintf(path, sizeof path, "%s_events.csv", csv_prefix);
    FILE *fv = fopen(path, "w");
    if (!fe || !fv) {
        fprintf(stderr, "cannot open csv output (run from repo root)\n");
        exit(2);
    }
    fprintf(fe, "t_s,rpm_true,rpm_est,state,outputs_enabled,tooth_idx,phase,"
                "angle_err_deg,spurious\n");
    fprintf(fv, "t_s,event_id,rpm_true,err_deg,err_us,transient\n");

    vec_t rpm_s = {0}, rpm_t = {0}, rpm_r = {0}, ev_s = {0}, ev_t = {0}, ev_a = {0};
    pending_t pend[MAX_PEND];
    int npend = 0;

    bool drop_done[4] = {0}, spur_done[4] = {0};
    long edge_count = 0;
    long first_edge_no = -1;
    long synced_at_edge = -1;
    bool prev_synced = false;
    double t_lost_s = -1;
    bool seen_sync = false;
    double sync_time_s = -1;
    size_t gate_viol = 0;

    double theta = 123.0;                 /* start part-way round a rev */
    double t_us  = 0.0;
    const double dur_us = sc->duration_s * 1e6;
    long poll_next = 1000;

    while (t_us < dur_us) {
        double t_s = t_us * 1e-6;
        double rpm = rpm_at(sc, t_s);
        double dth = rpm * 6e-6;                     /* deg per us */
        double theta_new = theta + dth;

        long m_prev = (long)floor(theta / 6.0);
        long m_new  = (long)floor(theta_new / 6.0);

        for (long m = m_prev + 1; m <= m_new; m++) {
            long slot = m % 60;
            if (slot >= 58) {
                continue;                           /* missing teeth */
            }
            double b     = 6.0 * (double)m;
            double te_us = t_us + (b - theta) / dth;
            double te_s  = te_us * 1e-6;

            /* sensor-signal loss window */
            if (sc->stall_from_s >= 0 && te_s >= sc->stall_from_s &&
                te_s < sc->stall_to_s) {
                continue;
            }
            /* dropped-tooth fault */
            bool dropped = false;
            for (int k = 0; k < sc->n_drop; k++) {
                if (!drop_done[k] && te_s >= sc->drop_at_s[k]) {
                    drop_done[k] = true;
                    dropped = true;
                    break;
                }
            }
            if (dropped) {
                continue;
            }

            double slot_us = (60.0 / rpm) * 1e6 / 60.0;   /* 1/60 rev */

            /* up to two deliveries: the real edge and an optional spurious */
            int    ndel = 1;
            double del_us[2]  = { te_us + rnd_n() * sc->jitter_frac * slot_us, 0 };
            double del_th[2]  = { b, 0 };
            bool   del_sp[2]  = { false, false };
            for (int k = 0; k < sc->n_spur; k++) {
                if (!spur_done[k] && te_s >= sc->spur_at_s[k]) {
                    spur_done[k] = true;
                    del_us[1] = te_us + sc->spur_frac[k] * slot_us;
                    del_th[1] = b + sc->spur_frac[k] * 6.0;
                    del_sp[1] = true;
                    ndel = 2;
                    break;
                }
            }

            for (int di = 0; di < ndel; di++) {
                double   cyc = fmod(del_th[di], 720.0);
                bool     cam = cam_level_at(cyc);
                if (sc->cam_inv_from_s >= 0 && te_s >= sc->cam_inv_from_s &&
                    te_s < sc->cam_inv_to_s) {
                    cam = true;                      /* cam signal stuck high */
                }
                uint32_t tick = (uint32_t)((uint64_t)((int64_t)sc->t0_ticks +
                                (int64_t)llround(del_us[di])));
                cd_on_edge(&dec, tick, cam);
                edge_count++;
                if (first_edge_no < 0) {
                    first_edge_no = edge_count;
                }

                /* ---- sync bookkeeping -------------------------------- */
                bool synced = (dec.state == CD_SYNCED);
                if (synced && !prev_synced) {
                    if (!seen_sync) {
                        seen_sync = true;
                        synced_at_edge = edge_count;
                        sync_time_s = te_s;
                        res->first_sync_revs =
                            (double)(synced_at_edge - first_edge_no + 1) / 58.0;
                    } else if (t_lost_s >= 0 && res->n_resync < 16) {
                        resync_t *r = &res->resync[res->n_resync++];
                        /* recovery time counts from the moment the fault
                           clears (cam signal restored / sensor back)      */
                        double from = t_lost_s;
                        if (sc->cam_inv_from_s >= 0 && from >= sc->cam_inv_from_s &&
                            from < sc->cam_inv_to_s) {
                            from = sc->cam_inv_to_s;
                        }
                        if (sc->stall_from_s >= 0 && from >= sc->stall_from_s &&
                            from < sc->stall_to_s) {
                            from = sc->stall_to_s;
                        }
                        r->t_lost = t_lost_s;
                        r->t_back = te_s;
                        r->revs = (te_s - from) * rpm / 60.0;
                        t_lost_s = -1;
                    }
                }
                if (!synced && prev_synced) {
                    t_lost_s = te_s;
                }
                prev_synced = synced;

                /* ---- accuracy of decoder belief ----------------------- */
                double ang_err = 0.0;
                if (synced && !del_sp[di]) {
                    ang_err = wrap_pm((double)cd_tooth_cycle_angle(&dec) - cyc, 720.0);
                }
                bool trans = fabs(accel_at(sc, te_s)) > 100.0;
                if (synced && !del_sp[di] && dec.rpm > 0.0f) {
                    double e = ((double)dec.rpm - rpm) / rpm * 100.0;
                    /* run-up = fast change below 800 rpm (cranking to idle) */
                    vec_push(!trans ? &rpm_s : (rpm < 800.0 ? &rpm_r : &rpm_t), e);
                }
                /* over-speed gate: outputs must be off once true speed is
                   clearly above the limit (margin covers estimator lag)   */
                if (rpm > dec.cfg.rpm_limit + 150.0 && dec.outputs_enabled) {
                    gate_viol++;
                }

                fprintf(fe, "%.6f,%.2f,%.2f,%d,%d,%u,%u,%.3f,%d\n",
                        te_s, rpm, (double)dec.rpm, (int)dec.state,
                        (int)dec.outputs_enabled, (unsigned)dec.tooth_idx,
                        (unsigned)dec.phase, ang_err, del_sp[di] ? 1 : 0);

                /* ---- schedule events from the timing map -------------- */
                if (dec.outputs_enabled) {
                    float inj = tm_inj_start_btdc(&TM_DEMO_MAP, dec.rpm,
                                                  (float)sc->load_pct);
                    float spk = tm_spark_btdc(&TM_DEMO_MAP, dec.rpm,
                                              (float)sc->load_pct);
                    for (int c = 0; c < N_CYL; c++) {
                        float tdc = (float)c * (720.0f / N_CYL);
                        for (int ty = 0; ty < 2; ty++) {
                            float a = (ty == 0) ? (tdc + 720.0f - inj)
                                                : (tdc + 720.0f - spk);
                            a = fmodf(a, 720.0f);
                            uint32_t ft;
                            int id = c * 2 + ty;
                            if (cd_event_update(&ev[id], &dec, a, &ft) &&
                                npend < MAX_PEND) {
                                /* truth: absolute crank angle at which the
                                   requested cycle angle really occurs */
                                double rel = wrap_pm((double)a - cyc, 720.0);
                                if (rel < 0) {
                                    rel += 720.0;
                                }
                                pend[npend].target_theta = del_th[di] + rel;
                                pend[npend].fire_tick = ft;
                                pend[npend].id = id;
                                pend[npend].transient = trans;
                                npend++;
                            }
                        }
                    }
                }
            }
        }

        /* ---- true crossings of pending events -------------------------- */
        for (int i = 0; i < npend; ) {
            if (pend[i].target_theta > theta && pend[i].target_theta <= theta_new) {
                double tc_us = t_us + (pend[i].target_theta - theta) / dth;
                uint32_t tick_true = (uint32_t)((uint64_t)((int64_t)sc->t0_ticks +
                                                (int64_t)llround(tc_us)));
                int32_t err_ticks = (int32_t)(pend[i].fire_tick - tick_true);
                double err_us  = (double)err_ticks;
                double err_deg = err_us * rpm * 6e-6;
                vec_push(pend[i].transient ? &ev_t : &ev_s, err_deg);
                vec_push(&ev_a, err_deg);
                res->events_total++;
                if (fabs(err_deg) > 1.0) {
                    res->events_gt1deg++;
                }
                if (fabs(err_deg) > 2.0) {
                    res->events_gt2deg++;
                }
                fprintf(fv, "%.6f,%d,%.2f,%.4f,%.1f,%d\n", tc_us * 1e-6,
                        pend[i].id, rpm, err_deg, err_us,
                        pend[i].transient ? 1 : 0);
                pend[i] = pend[--npend];
            } else {
                i++;
            }
        }

        theta = theta_new;
        t_us += 1.0;
        if ((long)t_us >= poll_next) {
            uint32_t stalls_before = dec.stall_count;
            cd_poll(&dec, (uint32_t)((uint64_t)((int64_t)sc->t0_ticks + (int64_t)t_us)));
            poll_next += 1000;
            if (dec.stall_count != stalls_before) {
                /* log the instant the stall is declared (spurious = 2) */
                fprintf(fe, "%.6f,%.2f,%.2f,%d,%d,%u,%u,%.3f,%d\n", t_us * 1e-6,
                        rpm, (double)dec.rpm, (int)dec.state,
                        (int)dec.outputs_enabled, (unsigned)dec.tooth_idx,
                        (unsigned)dec.phase, 0.0, 2);
                if (prev_synced) {
                    prev_synced = false;
                    t_lost_s = t_us * 1e-6;
                }
            }
        }
    }
    (void)sync_time_s;

    res->rpm_err_steady = vec_stats(&rpm_s);
    res->rpm_err_trans  = vec_stats(&rpm_t);
    res->rpm_err_runup  = vec_stats(&rpm_r);
    res->ev_err_steady  = vec_stats(&ev_s);
    res->ev_err_trans   = vec_stats(&ev_t);
    res->ev_err_all     = vec_stats(&ev_a);
    res->gated_while_over_limit_violations = gate_viol;
    res->dec = dec;

    free(rpm_s.v); free(rpm_t.v); free(rpm_r.v); free(ev_s.v); free(ev_t.v); free(ev_a.v);
    fclose(fe);
    fclose(fv);
}

static void print_stats(const char *label, const stats_t *s, const char *unit)
{
    printf("  %-28s n=%-7zu mean=%+8.4f sd=%7.4f p99=%7.4f max=%7.4f %s\n",
           label, s->n, s->mean, s->sd, s->p99, s->maxabs, unit);
}

/* --------------------------------------------------------------------- */
/* scenario definitions                                                   */
/* --------------------------------------------------------------------- */
static const pt_t PROF_A[] = {
    {0.0, 200}, {1.0, 200}, {2.0, 800}, {3.0, 800}, {4.0, 1500}, {6.0, 1500},
    {6.65, 2800}, {8.0, 2800}, {9.3, 1500}, {11.0, 1500}
};

static const pt_t PROF_B[] = { {0.0, 1500}, {7.5, 1500} };

static const pt_t PROF_C[] = {
    {0.0, 1500}, {1.0, 1500}, {2.0, 3400}, {3.0, 3400}, {4.0, 2700}, {5.0, 2700}
};

int main(void)
{
    result_t r;
    const double INF = -1.0;

    /* ================= Scenario A ===================================== */
    printf("\n[A] start-up, ramps, hard acceleration, timer wrap-around\n");
    scenario_t A = {
        .name = "A", .duration_s = 11.0, .prof = PROF_A,
        .nprof = (int)(sizeof PROF_A / sizeof PROF_A[0]),
        .load_pct = 60.0, .jitter_frac = 0.002,
        .t0_ticks = 4294967296ULL - 6200000ULL,    /* wraps at t = 6.2 s */
        .cam_inv_from_s = INF, .cam_inv_to_s = INF,
        .stall_from_s = INF, .stall_to_s = INF,
    };
    run(&A, &r, "results/data/scenario_A");
    print_stats("RPM error, steady", &r.rpm_err_steady, "%");
    print_stats("RPM error, transient >=800", &r.rpm_err_trans, "%");
    print_stats("RPM error, run-up <800", &r.rpm_err_runup, "%");
    print_stats("event timing, steady", &r.ev_err_steady, "deg");
    print_stats("event timing, transient", &r.ev_err_trans, "deg");
    printf("  sync acquired after %.2f crank revolutions of signal\n",
           r.first_sync_revs);
    printf("  events scheduled: %zu  (>1 deg: %zu, >2 deg: %zu)\n",
           r.events_total, r.events_gt1deg, r.events_gt2deg);
    printf("  sync losses: %u, cam faults: %u, stalls: %u\n",
           (unsigned)r.dec.sync_loss_count, (unsigned)r.dec.cam_fault_count,
           (unsigned)r.dec.stall_count);

    CHECK(r.first_sync_revs <= 3.2, "sync acquired within 3.2 revs (%.2f)",
          r.first_sync_revs);
    CHECK(r.rpm_err_steady.maxabs < 0.5, "steady RPM error < 0.5%% (max %.3f%%)",
          r.rpm_err_steady.maxabs);
    CHECK(r.rpm_err_trans.maxabs < 1.0,
          "transient RPM error (>=800 rpm, up to 2000 rpm/s) < 1%% (max %.3f%%)",
          r.rpm_err_trans.maxabs);
    CHECK(r.rpm_err_runup.maxabs < 6.0,
          "run-up RPM lag (200->800 rpm in 1 s) < 6%% (max %.3f%%)",
          r.rpm_err_runup.maxabs);
    CHECK(r.ev_err_steady.maxabs < 0.5, "steady event timing < 0.5 deg (max %.3f)",
          r.ev_err_steady.maxabs);
    CHECK(r.ev_err_trans.maxabs < 1.5, "transient event timing < 1.5 deg (max %.3f)",
          r.ev_err_trans.maxabs);
    CHECK(r.events_gt2deg == 0, "no event off by more than 2 deg (%zu)",
          r.events_gt2deg);
    CHECK(r.dec.sync_loss_count == 0, "no spurious sync loss in clean run (%u)",
          (unsigned)r.dec.sync_loss_count);
    CHECK(r.events_total > 1000, "event volume plausible (%zu)", r.events_total);

    /* ================= Scenario B ===================================== */
    printf("\n[B] fault injection at 1500 rpm\n");
    scenario_t B = {
        .name = "B", .duration_s = 7.5, .prof = PROF_B, .nprof = 2,
        .load_pct = 60.0, .jitter_frac = 0.002, .t0_ticks = 1000u,
        .n_drop = 1, .drop_at_s = {1.0},
        .n_spur = 3, .spur_at_s = {2.0, 3.0, 3.5}, .spur_frac = {0.2, 0.5, 0.75},
        .cam_inv_from_s = 4.0, .cam_inv_to_s = 4.2,
        .stall_from_s = 5.5, .stall_to_s = 6.0,
    };
    run(&B, &r, "results/data/scenario_B");
    print_stats("event timing, all", &r.ev_err_all, "deg");
    printf("  glitches rejected: %u, dropouts: %u, sync losses: %u, "
           "cam faults: %u, stalls: %u\n",
           (unsigned)r.dec.glitch_count, (unsigned)r.dec.dropout_count,
           (unsigned)r.dec.sync_loss_count, (unsigned)r.dec.cam_fault_count,
           (unsigned)r.dec.stall_count);
    for (int i = 0; i < r.n_resync; i++) {
        printf("  resync %d: lost at %.3f s, back at %.3f s (%.1f revs after fault cleared)\n", i + 1,
               r.resync[i].t_lost, r.resync[i].t_back, r.resync[i].revs);
    }
    printf("  events scheduled: %zu  (>1 deg: %zu, >2 deg: %zu)\n",
           r.events_total, r.events_gt1deg, r.events_gt2deg);

    CHECK(r.dec.dropout_count == 1, "dropped tooth detected (%u)",
          (unsigned)r.dec.dropout_count);
    CHECK(r.dec.glitch_count >= 1, "short glitch rejected by filter (%u)",
          (unsigned)r.dec.glitch_count);
    CHECK(r.dec.cam_fault_count >= 1, "stuck cam signal detected (%u)",
          (unsigned)r.dec.cam_fault_count);
    CHECK(r.dec.stall_count == 1, "signal loss detected as stall (%u)",
          (unsigned)r.dec.stall_count);
    bool all_recovered = (r.n_resync >= 3) && (r.dec.state == CD_SYNCED);
    for (int i = 0; i < r.n_resync; i++) {
        if (r.resync[i].revs > 4.5) {
            all_recovered = false;
        }
    }
    CHECK(all_recovered, "every fault re-acquired within 4.5 revs, final state SYNCED");
    CHECK(r.events_gt2deg == 0, "no event fired >2 deg off during/after faults (%zu)",
          r.events_gt2deg);

    /* ================= Scenario C ===================================== */
    printf("\n[C] over-speed gating (limit 3200 rpm, hysteresis 200)\n");
    scenario_t C = {
        .name = "C", .duration_s = 5.0, .prof = PROF_C,
        .nprof = (int)(sizeof PROF_C / sizeof PROF_C[0]),
        .load_pct = 60.0, .jitter_frac = 0.002, .t0_ticks = 0u,
        .cam_inv_from_s = INF, .cam_inv_to_s = INF,
        .stall_from_s = INF, .stall_to_s = INF,
    };
    run(&C, &r, "results/data/scenario_C");
    printf("  outputs enabled while true speed > limit+150: %zu edges\n",
           r.gated_while_over_limit_violations);
    printf("  sync losses during over-speed: %u\n", (unsigned)r.dec.sync_loss_count);
    CHECK(r.gated_while_over_limit_violations == 0,
          "outputs gated off whenever speed exceeds limit");
    CHECK(r.dec.sync_loss_count == 0, "decoder stays synced through 3400 rpm");
    CHECK(r.events_gt2deg == 0, "events stay accurate up to limit (%zu >2 deg)",
          r.events_gt2deg);

    printf("\n%d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
