#ifndef BARO_
#define BARO_

#include <cstdint>

#include "driver_bmp390_shot.h"
#include "driver_bmp390_interface.h"
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

namespace sensors
{

    struct BaroData
    {
        float temp_C;      // Temperature in Celsius 🌡️
        float pressure_Pa; // Pressure in Pascals 🧯
    };

    uint8_t sensor_init(bmp390_interface_t interface, bmp390_address_t addr_pin, bmp390_handle_t *m_sensor)
    {
        uint8_t to_return;

        DRIVER_BMP390_LINK_INIT(m_sensor, bmp390_handle_t);
        DRIVER_BMP390_LINK_IIC_INIT(m_sensor, bmp390_interface_iic_init);
        DRIVER_BMP390_LINK_IIC_DEINIT(m_sensor, bmp390_interface_iic_deinit);
        DRIVER_BMP390_LINK_IIC_READ(m_sensor, bmp390_interface_iic_read);
        DRIVER_BMP390_LINK_IIC_WRITE(m_sensor, bmp390_interface_iic_write);
        DRIVER_BMP390_LINK_SPI_INIT(m_sensor, bmp390_interface_spi_init);
        DRIVER_BMP390_LINK_SPI_DEINIT(m_sensor, bmp390_interface_spi_deinit);
        DRIVER_BMP390_LINK_SPI_READ(m_sensor, bmp390_interface_spi_read);
        DRIVER_BMP390_LINK_SPI_WRITE(m_sensor, bmp390_interface_spi_write);
        DRIVER_BMP390_LINK_DELAY_MS(m_sensor, bmp390_interface_delay_ms);
        DRIVER_BMP390_LINK_DEBUG_PRINT(m_sensor, bmp390_interface_debug_print);
        DRIVER_BMP390_LINK_RECEIVE_CALLBACK(m_sensor, bmp390_interface_receive_callback);

        to_return = bmp390_set_interface(m_sensor, interface);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set interface failed.\n");
            return 1;
        }
        to_return = bmp390_set_addr_pin(m_sensor, addr_pin);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set addr pin failed.\n");
            return 1;
        }
        /* bmp390 init */
        to_return = bmp390_init(m_sensor);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: init failed.\n");
            return 1;
        }
        /* set default spi wire */
        to_return = bmp390_set_spi_wire(m_sensor, BMP390_BASIC_DEFAULT_SPI_WIRE);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set spi wire failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* set default iic watchdog timer */
        to_return = bmp390_set_iic_watchdog_timer(m_sensor, BMP390_BASIC_DEFAULT_IIC_WATCHDOG_TIMER);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set iic watchdog timer failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* set default iic watchdog period */
        to_return = bmp390_set_iic_watchdog_period(m_sensor, BMP390_BASIC_DEFAULT_IIC_WATCHDOG_PERIOD);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set iic watchdog period failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* disable fifo */
        to_return = bmp390_set_fifo(m_sensor, BMP390_BOOL_FALSE);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set fifo failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* disable interrupt fifo watermark */
        to_return = bmp390_set_interrupt_fifo_watermark(m_sensor, BMP390_BOOL_FALSE);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set interrupt fifo watermark failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* disable interrupt fifo full */
        to_return = bmp390_set_interrupt_fifo_full(m_sensor, BMP390_BOOL_FALSE);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set interrupt fifo full failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* disable interrupt data ready */
        to_return = bmp390_set_interrupt_data_ready(m_sensor, BMP390_BOOL_FALSE);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set interrupt data ready failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* set default pressure */
        to_return = bmp390_set_pressure(m_sensor, BMP390_BASIC_DEFAULT_PRESSURE);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set pressure failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* set default temperature */
        to_return = bmp390_set_temperature(m_sensor, BMP390_BASIC_DEFAULT_TEMPERATURE);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set temperature failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* set default pressure oversampling */
        to_return = bmp390_set_pressure_oversampling(m_sensor, BMP390_BASIC_DEFAULT_PRESSURE_OVERSAMPLING);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set pressure oversampling failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* set default temperature oversampling */
        to_return = bmp390_set_temperature_oversampling(m_sensor, BMP390_BASIC_DEFAULT_TEMPERATURE_OVERSAMPLING);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set temperature oversampling failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* set default odr */
        to_return = bmp390_set_odr(m_sensor, BMP390_BASIC_DEFAULT_ODR);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set odr failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* set default filter coefficient */
        to_return = bmp390_set_filter_coefficient(m_sensor, BMP390_BASIC_DEFAULT_FILTER_COEFFICIENT);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set filter coefficient failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        /* set default mode */
        to_return = bmp390_set_mode(m_sensor, BMP390_MODE_NORMAL_MODE);
        if (to_return != 0)
        {
            bmp390_interface_debug_print("bmp390: set mode failed.\n");
            (void)bmp390_deinit(m_sensor);
            return 1;
        }
        return 0;
    };

    uint8_t bmp390_basic_read(float *temperature_c, float *pressure_pa, bmp390_handle_t *m_sensor)
    {
        uint32_t temperature_raw;
        uint32_t pressure_raw;
        /* read temperature and pressure */
        if (bmp390_read_temperature_pressure(m_sensor, (uint32_t *)&temperature_raw, temperature_c,
                                             (uint32_t *)&pressure_raw, pressure_pa) != 0)
        {
            return 1;
        }
        else
        {
            return 0;
        }
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

        bmp390_handle_t m_sensor;

        SensorHealth m_health;
    };
} 
#endif  //BARO_
