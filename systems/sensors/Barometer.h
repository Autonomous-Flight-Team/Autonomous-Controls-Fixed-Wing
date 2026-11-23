#ifndef BARO_
#define BARO_

#include <cstdint>

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"
#include "logging_interface.h"

#define BMP390_BASIC_DEFAULT_SPI_WIRE BMP390_SPI_WIRE_4                           /**< 4 wire spi */
#define BMP390_BASIC_DEFAULT_IIC_WATCHDOG_TIMER BMP390_BOOL_TRUE                  /**< enable iic watchdog timer */
#define BMP390_BASIC_DEFAULT_IIC_WATCHDOG_PERIOD BMP390_IIC_WATCHDOG_PERIOD_40_MS /**< set watchdog timer period 40ms */
#define BMP390_BASIC_DEFAULT_PRESSURE BMP390_BOOL_TRUE                            /**< enable pressure **/
#define BMP390_BASIC_DEFAULT_TEMPERATURE BMP390_BOOL_TRUE                         /**< enable temperature */
#define BMP390_BASIC_DEFAULT_PRESSURE_OVERSAMPLING BMP390_OVERSAMPLING_x32        /**< pressure oversampling x32 */
#define BMP390_BASIC_DEFAULT_TEMPERATURE_OVERSAMPLING BMP390_OVERSAMPLING_x2      /**< temperature oversampling x2 */
#define BMP390_BASIC_DEFAULT_ODR BMP390_ODR_12P5_HZ                               /**< output data rate 12.5Hz */
#define BMP390_BASIC_DEFAULT_FILTER_COEFFICIENT BMP390_FILTER_COEFFICIENT_15      /**< set filter coefficient 15 */

#define BarometerPin 0

#define baro_small_fail_threshold 10
#define baro_full_fail_threshold 10
#define baro_small_recover_threshold 10
#define baro_full_recover_threshold 10

namespace sensors
{

    struct BaroData
    {
        float temp_C;      // Temperature in Celsius 🌡️
        float pressure_Pa; // Pressure in Pascals 🧯
    };

    class Barometer : public logging
    {
    public:
        Barometer();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        BaroData get();

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

    private:
        BaroData m_stats;

        Adafruit_BMP3XX m_sensor;

        SensorHealth m_health;
    };
} 
#endif  //BARO_
