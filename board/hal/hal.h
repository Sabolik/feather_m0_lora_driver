//////////////////////////////////////////////////////////////////////
// EEPROM
//////////////////////////////////////////////////////////////////////

#include <stdint.h>

#include "../../Config/nv_storage_config.h"

typedef void (*hal_samd_usb_tx_done_cb_t)(void);
typedef void (*hal_samd_usb_rx_done_cb_t)(uint8_t* data, uint16_t len);

#define HAL_EEPROM_BASE     CONF_STORAGE_MEMORY_START

void hal_samd_system_init( void );
void hal_samd_sleep( void );

void hal_samd_io_irq_init (void* cb);
void hal_samd_set_pin_level_rfm_cs( void );
void hal_samd_clr_pin_level_rfm_cs( void );
void hal_samd_set_pin_level_rfm_rst( void );
void hal_samd_clr_pin_level_rfm_rst( void );
void hal_samd_set_pin_output_rfm_rst( void );
void hal_samd_set_pin_input_rfm_rst( void );
void hal_samd_set_pin_level_red_led( void );
void hal_samd_clr_pin_level_red_led( void );

uint8_t hal_samd_spi (uint8_t out);

uint32_t hal_samd_rtc_get_time( void );
void hal_samd_rtc_set_wakeuptime( uint32_t val );

void hal_samd_enable_irq( void );
void hal_samd_disable_irq( void );

void hal_samd_i2c_ioInit(int16_t addr);
void hal_samd_i2c_writeByte(uint8_t reg_addr, uint8_t data);
void hal_samd_i2c_readBlock(uint8_t reg_addr, uint8_t *read_buff, uint8_t length);

void hal_samd_usb_serial_init(void);
void hal_samd_usb_serial_deinit(void);
uint8_t hal_samd_usb_serial_initialized(void);
void hal_samd_usb_serial_start(hal_samd_usb_rx_done_cb_t rx_done_cb, hal_samd_usb_tx_done_cb_t tx_done_cb);
uint8_t hal_samd_usb_serial_is_off(void);
void hal_samd_usb_serial_startrx(void);
uint8_t hal_samd_usb_serial_starttx(uint8_t* data, uint16_t len);

void hal_samd_persistance_init(void);
void hal_samd_persistance_write(void* dst, const void* src, uint16_t len, uint16_t storage_size);

void hal_samd_sensor_set_power(void);
void hal_samd_sensor_clr_power(void);