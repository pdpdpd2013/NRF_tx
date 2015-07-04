#ifndef _nRF24L01_h_
#define _nRF24L01_h_

/* ================= define nRF24L01 module ================ */
// basic instruction
#define R_REG			0x00
#define W_REG			0x20
#define R_RX_PAYLOAD 	0x61
#define W_TX_PAYLOAD 	0xA0
#define FLUSH_TX 		0xE1
#define FLUSH_RX 		0xE2
#define REUSE_TX_PL 	0xE3


// register define
#define CONF_REG 		0x00
#define EN_AA	 		0x01
#define EN_RXADDR 		0x02
#define SETUP_AW		0x03
#define SETUP_RETR		0x04
#define RF_CH			0x05
#define RF_SETUP		0x06
#define STATUS			0x07
#define OBSERVE_TX		0x08
#define CD				0x09
#define RX_ADDR_P0		0x0A

#define TX_ADDR			0x10
#define RX_PW_P0		0x11
#define FIFO_STATUS		0x17

#endif
