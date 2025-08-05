

#ifndef INC_NRF24_H_
#define INC_NRF24_H_

#include "main.h"
#include "stm32f1xx_hal.h"


// 핀 제어 매크로 (적절한 GPIO/핀으로 수정)
#define NRF24_CSN_GPIO   GPIOC
#define NRF24_CSN_PIN    GPIO_PIN_1
#define NRF24_CE_GPIO    GPIOC
#define NRF24_CE_PIN     GPIO_PIN_0

// 외부에서 선언된 SPI 핸들 사용
extern SPI_HandleTypeDef hspi1;

// 주요 함수 선언
void     nrf24_init(void);
void     nrf24_set_tx_addr(uint8_t *addr, uint8_t len);
void     nrf24_set_rx_addr(uint8_t *addr, uint8_t len);
void     nrf24_set_channel(uint8_t ch);
void     nrf24_tx_mode(void);
void     nrf24_rx_mode(void);
void     nrf24_send_data(uint8_t *data, uint8_t len);
uint8_t  nrf24_data_ready(void);
void     nrf24_receive(uint8_t *data, uint8_t len);

// 내부 레지스터 접근 함수(필요하면 외부 공개)
void nrf24_write_reg(uint8_t reg, uint8_t value);
uint8_t nrf24_read_reg(uint8_t reg);
void nrf24_write_reg_multi(uint8_t reg, uint8_t *data, uint8_t len);





#endif /* INC_NRF24_H_ */
