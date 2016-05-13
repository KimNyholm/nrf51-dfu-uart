#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "softdevice_handler.h"
#include "bta_ble.h"
#include "app_uart.h"

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_MAX_TIMERS             (6+BSP_APP_TIMERS_NUMBER)                  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define WAKEUP_BUTTON_ID                 0                                          /**< Button used to wake up the application. */
#define BOND_DELETE_ALL_BUTTON_ID        1                                          /**< Button used for deleting all bonded centrals during startup. */



/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void bta_bleUartReceived(uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
    bta_bleUartTransmit(p_data, length);
}

void bta_bleConnected(void){
  printf("\nBluetooth connect.");
}
void bta_bleDisconnected(void){
  printf("\nBluetooth disconnect.");
  bta_bleAdvertiseBegin("Genstart", 8);
  
}

void bta_bleAdvertiseTimeout(void){
  uint32_t                         err_code;
  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);
  // Configure buttons with sense level low as wakeup source.
  err_code = bsp_buttons_enable(1 << WAKEUP_BUTTON_ID);
  APP_ERROR_CHECK(err_code);
  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();    
  APP_ERROR_CHECK(err_code);
}
void bta_bleResetPrepare(void){
  uint32_t                         err_code;
  printf("\nClosing down for DFU");
  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);
}
/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
  // Initialize.
    timers_init();
    APP_GPIOTE_INIT(1);

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        NULL);
    APP_ERROR_CHECK(err_code);
    uart_init();
    printf("\nStarting...");
    bta_bleInit(false);
    bta_bleSetDeviceName("dfu uart test");
    printf("\nAdvertising...");
    bta_bleAdvertiseBegin("Hej kim", 7);

    // Enter main loop.
    uint32_t i= 0;
    for (;;)
    {
      i++;
      printf("%4d", i);
      nrf_delay_ms(1000);
      power_manage();
    }
}
