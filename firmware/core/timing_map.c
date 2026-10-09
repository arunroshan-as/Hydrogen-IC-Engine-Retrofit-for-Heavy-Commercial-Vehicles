#include "timing_map.h"

/* Illustrative demonstration values only. */
const tm_map_t TM_DEMO_MAP = {
    .rpm_axis  = { 800.f, 1200.f, 1500.f, 1800.f, 2400.f, 3000.f, 3600.f },
    .load_axis = { 20.f, 40.f, 60.f, 80.f, 100.f },
    .inj_start_btdc = {
        /* 20 % */ { 70.f, 72.f, 74.f, 76.f, 80.f, 84.f, 88.f },
        /* 40 % */ { 68.f, 70.f, 72.f, 74.f, 78.f, 82.f, 86.f },
        /* 60 % */ { 66.f, 68.f, 70.f, 72.f, 76.f, 80.f, 84.f },
        /* 80 % */ { 64.f, 66.f, 68.f, 70.f, 74.f, 78.f, 82.f },
        /* 100% */ { 62.f, 64.f, 66.f, 68.f, 72.f, 76.f, 80.f },
    },
    .spark_btdc = {
        /* 20 % */ { 14.f, 16.f, 18.f, 20.f, 24.f, 28.f, 32.f },
        /* 40 % */ { 12.f, 14.f, 16.f, 18.f, 22.f, 26.f, 30.f },
        /* 60 % */ { 10.f, 12.f, 14.f, 16.f, 20.f, 24.f, 28.f },
        /* 80 % */ {  8.f, 10.f, 12.f, 14.f, 18.f, 22.f, 26.f },
        /* 100% */ {  6.f,  8.f, 10.f, 12.f, 16.f, 20.f, 24.f },
    },
};

static void axis_locate(const float *axis, int n, float x, int *i, float *f)
{
    if (x <= axis[0]) { *i = 0; *f = 0.0f; return; }
    if (x >= axis[n - 1]) { *i = n - 2; *f = 1.0f; return; }
    int k = 0;
    while (k < n - 2 && x > axis[k + 1]) {
        k++;
    }
    *i = k;
    *f = (x - axis[k]) / (axis[k + 1] - axis[k]);
}

float tm_lookup(const float rpm_axis[TM_RPM_POINTS],
                const float load_axis[TM_LOAD_POINTS],
                const float table[TM_LOAD_POINTS][TM_RPM_POINTS],
                float rpm, float load)
{
    int ir, il;
    float fr, fl;
    axis_locate(rpm_axis, TM_RPM_POINTS, rpm, &ir, &fr);
    axis_locate(load_axis, TM_LOAD_POINTS, load, &il, &fl);

    float v00 = table[il][ir];
    float v01 = table[il][ir + 1];
    float v10 = table[il + 1][ir];
    float v11 = table[il + 1][ir + 1];
    float lo = v00 + (v01 - v00) * fr;
    float hi = v10 + (v11 - v10) * fr;
    return lo + (hi - lo) * fl;
}

float tm_inj_start_btdc(const tm_map_t *m, float rpm, float load)
{
    return tm_lookup(m->rpm_axis, m->load_axis, m->inj_start_btdc, rpm, load);
}

float tm_spark_btdc(const tm_map_t *m, float rpm, float load)
{
    return tm_lookup(m->rpm_axis, m->load_axis, m->spark_btdc, rpm, load);
}
