#include "atmel_start.h"
#include "nv.h"
#include "hal.h"

// -----------------------------------------------------------------------------
// HW

void hal_samd_system_init( void ) {
    system_init();
    // RTC interrupt globally enabled, for some reason not enabled via atmel start
    NVIC_EnableIRQ(RTC_IRQn);
}

void hal_samd_sleep( void ) {
    int32_t d = hri_rtcmode0_read_COMP_reg(RTC, 0) - hri_rtcmode0_read_COUNT_reg(RTC);
    // we are far from now enough to go to sleep
    if( d > 10 )
    {
        // clear any pending interrupts before sleep
        hri_rtcmode0_clear_INTFLAG_OVF_bit(RTC);
        hri_rtcmode0_clear_INTFLAG_CMP0_bit(RTC);
        sleep(3);
    }
}

// -----------------------------------------------------------------------------
// I/O

void hal_samd_io_irq_init (void* cb) {
    ext_irq_register(PIN_PA09, (ext_irq_cb_t)cb);
}

void hal_samd_set_pin_level_rfm_cs( void )
{
    gpio_set_pin_level(RFM_CS, true);
}

void hal_samd_clr_pin_level_rfm_cs( void )
{
    gpio_set_pin_level(RFM_CS, false);
}

void hal_samd_set_pin_level_rfm_rst( void )
{
    gpio_set_pin_level(RFM_RST, true);
}

void hal_samd_clr_pin_level_rfm_rst( void )
{
    gpio_set_pin_level(RFM_RST, false);
}

void hal_samd_set_pin_output_rfm_rst( void )
{
    gpio_set_pin_direction(RFM_RST, GPIO_DIRECTION_OUT);
}

void hal_samd_set_pin_input_rfm_rst( void )
{
    gpio_set_pin_direction(RFM_RST, GPIO_DIRECTION_IN);
}

void hal_samd_set_pin_level_red_led( void )
{
    gpio_set_pin_level(RedLed, true);
}

void hal_samd_clr_pin_level_red_led( void )
{
    gpio_set_pin_level(RedLed, false);
}

// -----------------------------------------------------------------------------
// SPI

uint8_t hal_samd_spi (uint8_t out) {
    return (uint8_t) SPI_0_exchange_data((uint8_t) out);
}

// -----------------------------------------------------------------------------
// TIME

void RTC_Handler(void) {
    // just wake up and clear flag
    hri_rtcmode0_clear_INTFLAG_CMP0_bit(RTC);
}

uint32_t hal_samd_rtc_get_time( void ) {
    return hri_rtcmode0_read_COUNT_reg(RTC);
}

void hal_samd_rtc_set_wakeuptime( uint32_t val ) {
    hri_rtcmode0_write_COMP_reg(RTC, 0, val);
}

// -----------------------------------------------------------------------------
// IRQ

void hal_samd_enable_irq( void ) {
    __enable_irq();
}

void hal_samd_disable_irq( void ) {
    __disable_irq();
}

// -----------------------------------------------------------------------------
// I2C

static struct io_descriptor *I2C_0_io;

void hal_samd_i2c_ioInit(int16_t addr)
{
    i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
    i2c_m_sync_enable(&I2C_0);
    i2c_m_sync_set_slaveaddr(&I2C_0, addr, I2C_M_SEVEN);
}

void hal_samd_i2c_writeByte(uint8_t reg_addr, uint8_t data)
{
    uint8_t writeData[2] = {reg_addr, data};
    uint8_t length = sizeof(writeData);

    io_write(I2C_0_io, writeData, length);
}

void hal_samd_i2c_readBlock(uint8_t reg_addr, uint8_t *read_buff, uint8_t length)
{
    uint8_t addrLength = sizeof(reg_addr);
    
    io_write(I2C_0_io, &reg_addr, addrLength);
    io_read(I2C_0_io, read_buff, length);
}

// -----------------------------------------------------------------------------
// USB

static uint8_t terminalReady = 0;
static uint8_t receptionEnabled = 0;
static hal_samd_usb_rx_done_cb_t hal_rx_done_cb = NULL;
static hal_samd_usb_tx_done_cb_t hal_tx_done_cb = NULL;

#if CONF_USBD_HS_SP
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
CDCD_ACM_HS_DESCES_LS_FS};
static uint8_t single_desc_bytes_hs[] = {
    /* Device descriptors and Configuration descriptors list. */
CDCD_ACM_HS_DESCES_HS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
CDCD_ACM_DESCES_LS_FS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif

static struct usbd_descriptors single_desc[]
= {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
,
{single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};

#define INIT_SEQUENCE           "LMiC Modem\r\n"

/** Buffers to receive and echo the communication bytes. */
static uint8_t usbd_cdc_buffer[CDCD_ECHO_BUF_SIZ] = {INIT_SEQUENCE};

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

/**
 * \brief Callback invoked when bulk OUT data received
 */
static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
    hal_rx_done_cb( (uint8_t *)usbd_cdc_buffer, (uint16_t)count );

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when bulk IN data received
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
    cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));
    
	/* all data sent */
    hal_tx_done_cb();

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	if (state.rs232.DTR) {
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_out);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_in);
		/* Start Rx */
        //cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));
        /* Start Tx with initial string to be printed */
        memcpy((uint8_t *)usbd_cdc_buffer, (uint8_t *)INIT_SEQUENCE, sizeof(INIT_SEQUENCE));
        cdcdf_acm_write((uint8_t *)usbd_cdc_buffer, sizeof(INIT_SEQUENCE));
        
        terminalReady = 1;
	}
    else
    {
        terminalReady = 0;
    }

	/* No error. */
	return false;
}

/**
 * \brief CDC ACM Init
 */
void hal_samd_usb_serial_init(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();

	usbdc_start(single_desc);
	usbdc_attach();
}

void hal_samd_usb_serial_deinit(void)
{
    usbdc_detach();
    usbdc_stop();
    cdcdf_acm_deinit();
    usbdc_deinit();
}

uint8_t hal_samd_usb_serial_initialized(void)
{
    return cdcdf_acm_is_enabled();
}

uint8_t hal_samd_usb_serial_is_off(void)
{
    return usbdc_get_state() == USBD_S_OFF;
}

void hal_samd_usb_serial_start(hal_samd_usb_rx_done_cb_t rx_done_cb, hal_samd_usb_tx_done_cb_t tx_done_cb)
{
    hal_tx_done_cb = tx_done_cb;
    hal_rx_done_cb = rx_done_cb;
    
    // Rx, TX done callback required
    if ( hal_rx_done_cb != NULL &&
         hal_tx_done_cb != NULL )
    {
        cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);
    }
}

void hal_samd_usb_serial_startrx(void) {
    receptionEnabled = 1;
    cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));
}

uint8_t hal_samd_usb_serial_starttx(uint8_t* data, uint16_t len) {
    if ( terminalReady )
    {
        memcpy(usbd_cdc_buffer, data, len);
        cdcdf_acm_write((uint8_t *)usbd_cdc_buffer, len);
    }
    
    return terminalReady;
}
// -----------------------------------------------------------------------------
// EEPROM

void hal_samd_persistance_init(void)
{
    nv_init(&FLASH_INSTANCE);
}

void hal_samd_persistance_write(void* dst, const void* src, uint16_t len, uint16_t storage_size)
{
    uint8_t tempData[storage_size];
    nv_read(0, 0, tempData, sizeof(tempData));
    nv_erase_sector(0);
    uint16_t offsetAddrr = (uint32_t)((uint32_t*)dst) - HAL_EEPROM_BASE;
    memcpy(tempData + offsetAddrr, src, len);
    nv_write(0, 0, tempData, sizeof(tempData));
}

// -----------------------------------------------------------------------------
// Sensor

void hal_samd_sensor_set_power(void) {
    // GPIO pins initialized only when power requested, otherwise free to use for any purpose
    gpio_set_pin_level(Sensor_Gnd, 0);
    gpio_set_pin_level(Sensor_Vcc, 1);
    gpio_set_pin_direction(Sensor_Gnd, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(Sensor_Vcc, GPIO_DIRECTION_OUT);
}

void hal_samd_sensor_clr_power(void) {
    gpio_set_pin_level(Sensor_Vcc, 0);
}
