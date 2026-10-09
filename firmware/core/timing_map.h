/*
 * timing_map.h - 2-D calibration map (speed x load) with bilinear
 * interpolation and axis clamping.
 *
 * The values in timing_map.c are ILLUSTRATIVE DEMONSTRATION VALUES used to
 * exercise the lookup and scheduling code. They are not an engine
 * calibration.
 */
#ifndef TIMING_MAP_H
#define TIMING_MAP_H

#define TM_RPM_POINTS  7
#define TM_LOAD_POINTS 5

typedef struct {
    float rpm_axis[TM_RPM_POINTS];
    float load_axis[TM_LOAD_POINTS];              /* percent               */
    float inj_start_btdc[TM_LOAD_POINTS][TM_RPM_POINTS]; /* deg BTDC      */
    float spark_btdc[TM_LOAD_POINTS][TM_RPM_POINTS];     /* deg BTDC      */
} tm_map_t;

extern const tm_map_t TM_DEMO_MAP;

/* Bilinear lookup. Inputs outside the axes are clamped to the edge. */
float tm_lookup(const float rpm_axis[TM_RPM_POINTS],
                const float load_axis[TM_LOAD_POINTS],
                const float table[TM_LOAD_POINTS][TM_RPM_POINTS],
                float rpm, float load);

float tm_inj_start_btdc(const tm_map_t *m, float rpm, float load);
float tm_spark_btdc(const tm_map_t *m, float rpm, float load);

#endif
