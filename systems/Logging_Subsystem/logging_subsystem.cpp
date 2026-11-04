
#include "logging_subsystem.h"

static probe_entry_t probe_table[] = {
    {SENSOR, SUB_IMU, sizeof(imu_raw_t), CALL_IMU, 0},
    {SENSOR, SUB_BARO, sizeof(baro_raw_t), CALL_BARO, 0}};

// void probe()
// {
//     for (size_t i = 0; i < probe_table.size(); i++)
//     {
//         probe_entry_t &e = probe_table[i];
//         run_probe(e);
//     }
// };

// void run_probe(probe_entry_t e)
// {
// }
