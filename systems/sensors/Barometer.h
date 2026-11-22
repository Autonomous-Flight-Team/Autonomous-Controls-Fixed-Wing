#ifndef GOOSE_SENSORS_BAROMETER_H_
#define GOOSE_SENSORS_BAROMETER_H_

// HARDWARE ONLY (GNC_HARDWARE_BUILD). See systems/README.MD.
#ifndef GNC_HARDWARE_BUILD
#error "Barometer.h is hardware-only. Include sensor_suite.h instead."
#endif

#include <cstdint>

#include "driver_bmp390.h"
#include "driver_bmp390_interface.h"
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"
#include "sensor_packets.h"
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

namespace sensors
{
    // BMP390 bring-up and read helpers (defined in Barometer.cpp).
    // Both return 0 on success and 1 on failure, like the libdriver API.
    uint8_t sensor_init(bmp390_interface_t interface,
                        bmp390_address_t addr_pin,
                        bmp390_handle_t *m_sensor);
    uint8_t bmp390_basic_read(float *temperature_c, float *pressure_pa,
                              bmp390_handle_t *m_sensor);

    class Barometer : public logging
    {
    public:
        Barometer();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        BaroData get();

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

        SensorHealth &health() { return m_health; }

    private:
        BaroData m_stats;

        bmp390_handle_t m_sensor;

        SensorHealth m_health;
    };
} 
#endif // GOOSE_SENSORS_BAROMETER_H_
