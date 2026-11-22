#ifndef GOOSE_SENSORS_SENSOR_SUITE_H_
#define GOOSE_SENSORS_SENSOR_SUITE_H_
// Picks real or dummy sensors at compile time.
//   GNC_HARDWARE_BUILD defined, GOOSE_DUMMY_SENSORS not -> real drivers
//   otherwise (native tests, or -D GOOSE_DUMMY_SENSORS)  -> DummySensors
#include "sensor_manager.h"

#if defined(GNC_HARDWARE_BUILD) && !defined(GOOSE_DUMMY_SENSORS)
#include "Barometer.h"
#include "Gps.h"
#include "ImuK.h"
#include "Pito.h"
#define GOOSE_SENSORS_ARE_REAL 1
namespace sensors {
using ImuSensor = ImuK;
using BaroSensor = Barometer;
using GpsSensor = Gps;
using PitotSensor = Pito;
} // namespace sensors
#else // goose-lint: disable=braces  (preprocessor, not an if/else)
#include "DummySensors.h"
#define GOOSE_SENSORS_ARE_REAL 0
namespace sensors {
using ImuSensor = DummyImu;
using BaroSensor = DummyBaro;
using GpsSensor = DummyGps;
using PitotSensor = DummyPitot;
} // namespace sensors
#endif

namespace sensors {
using SensorSuite =
    SensorManager<ImuSensor, BaroSensor, GpsSensor, PitotSensor>;
}

#endif // GOOSE_SENSORS_SENSOR_SUITE_H_
