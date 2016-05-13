#ifndef BTA_BLE_H__
#define BTA_BLE_H__

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */

void bta_ConnectionDisconnect(void);
void bta_bleInit(bool clearBonds);
void bta_SetDeviceName(void);
void advertising_start(void);
void uart_init(void);
void bta_bleAdvertiseBegin(uint8_t *data, uint8_t size);
void bta_bleAdvertiseStop(void);
void bta_bleUartTransmit(uint8_t * p_data, uint16_t length);
void bta_bleSetDeviceName(char * name);

// External call back
void bta_bleConnected(void);
void bta_bleDisconnected(void);
void bta_bleAdvertiseTimeout(void);
void bta_bleResetPrepare(void);

extern void bta_bleUartReceived(uint8_t * p_data, uint16_t length);

#endif
