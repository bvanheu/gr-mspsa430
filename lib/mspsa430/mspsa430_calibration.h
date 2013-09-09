#ifndef MSPSA430_CALIBRATION_H
#define MSPSA430_CALIBRATION_H

#include <stdint.h>

struct reference_level_gain {
    int8_t reference_level;
    uint8_t gain;
};

struct calibration_coefficient {
    uint8_t dc_select;
    double values[8];
};

struct calibration_freq_range {
    uint32_t start;
    uint32_t stop;
    uint32_t samples;
};

struct calibration_data {
    uint16_t format_version;
    uint8_t calibration_date[16];
    uint16_t software_version;
    uint8_t prod_side;
    struct calibration_freq_range freq_range[3];
    struct reference_level_gain ref_gain_table[8];
    uint32_t hardware_id;
    uint8_t usb_serial_number[16];
    uint32_t fxtal;
    uint16_t fxtalppm;
    uint8_t dev_temp_start[6];
    uint8_t dev_temp_stop[6];
    struct calibration_coefficient coeff_freq_gain[3][8];
};

#endif
