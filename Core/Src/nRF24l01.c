/*
 * nRF24l01.c
 *
 *  Created on: Apr 9, 2024
 *      Author: Cody
 */
#include "stm32f1xx_hal.h"
#include "nRF24l01.h"
#include <stdbool.h>

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1

#define NRF24_CE_PORT	GPIOA
#define NRF24_CE_PIN	GPIO_PIN_0

#define NRF24_CS_PORT	GPIOA
#define NRF24_CS_PIN	GPIO_PIN_1

/**
 * Writes a single byte to a particular register on the nRF24.
 *
 * @param reg: The nRF24 register to write to.
 * @param data: The data to write to the register.
 */
void nRF24WriteReg(uint8_t reg, uint8_t data) {
	uint8_t buf[2];
	buf[0] = reg | (1 << 5);
	buf[1] = data;

	// Pull CS pin low
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);

	// Transmit reg address + data
	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

	// Pull CS pin high; data transfer is complete
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);
}

/**
 * Writes multiple bytes to a particular register on the nRF24.
 *
 * @param reg: The nRF24 register to write to.
 * @param data: The address of the data to write to the register.
 * @param size: The size (in bytes) of the data to be written.
 */
void nRF24WriteRegMulti(uint8_t reg, uint8_t* data, int size) {
	uint8_t buf[1];
	buf[0] = reg | (1 << 5);

	// Pull CS pin low
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);

	// Transmit data
	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100); // reg address
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000); // data

	// Pull CS pin high; data transfer is complete
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);
}

/**
 * Reads a single byte from a particular register on the nRF24.
 *
 * @param reg: The nRF24 register to read from.
 *
 * @returns: The data that was read.
 */
uint8_t nRF24ReadReg(uint8_t reg) {
	uint8_t data=0;

	// Pull CS pin low
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);

	// Transmit address
	HAL_SPI_Transmit(NRF24_SPI, &reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);

	// Pull CS pin high; data transfer is complete
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);

	return data;
}

/**
 * Reads multiple bytes from a particular register on the nRF24.
 *
 * @param reg: The nRF24 register to read from.
 * @param data: The address of where you want to store the data read.
 * @param size: The number of bytes to read.
 *
 * @returns: The data that was read.
 */
void nRF24ReadRegMulti(uint8_t reg, uint8_t* data, int size) {
	// Pull CS pin low
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);

	// Transmit address
	HAL_SPI_Transmit(NRF24_SPI, &reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	// Pull CS pin high; data transfer is complete
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);
}

/**
 * Sends an arbitrary command to the nRF24.
 *
 * @param cmd: The command to send.
 */
void nRF24SendCmd(uint8_t cmd) {
	// Pull CS pin low
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);

	// Transmit command
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	// Pull CS pin high; data transfer is complete
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);
}

/**
 * Sets nRF24 registers to a default value.
 * Can specify just the status register, just the FIFO status register, or all.
 *
 * @param reg: The register address to reset (or a random number to reset all).
 */
void nRF24Reset(uint8_t reg) {
  if (reg == STATUS) {
    nRF24WriteReg(STATUS, 0x00);
  } else if (reg == FIFO_STATUS) {
    nRF24WriteReg(FIFO_STATUS, 0x11);
  } else {
    nRF24WriteReg(CONFIG, 0x08);
    nRF24WriteReg(EN_AA, 0x3F);
    nRF24WriteReg(EN_RXADDR, 0x03);
    nRF24WriteReg(SETUP_AW, 0x03);
    nRF24WriteReg(SETUP_RETR, 0x03);
    nRF24WriteReg(RF_CH, 0x02);
    nRF24WriteReg(RF_SETUP, 0x0E);
    nRF24WriteReg(STATUS, 0x00);
    nRF24WriteReg(OBSERVE_TX, 0x00);
    nRF24WriteReg(CD, 0x00);
    uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nRF24WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
    uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
    nRF24WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
    nRF24WriteReg(RX_ADDR_P2, 0xC3);
    nRF24WriteReg(RX_ADDR_P3, 0xC4);
    nRF24WriteReg(RX_ADDR_P4, 0xC5);
    nRF24WriteReg(RX_ADDR_P5, 0xC6);
    uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nRF24WriteRegMulti(TX_ADDR, tx_addr_def, 5);
    nRF24WriteReg(RX_PW_P0, 0);
    nRF24WriteReg(RX_PW_P1, 0);
    nRF24WriteReg(RX_PW_P2, 0);
    nRF24WriteReg(RX_PW_P3, 0);
    nRF24WriteReg(RX_PW_P4, 0);
    nRF24WriteReg(RX_PW_P5, 0);
    nRF24WriteReg(FIFO_STATUS, 0x11);
    nRF24WriteReg(DYNPD, 0);
    nRF24WriteReg(FEATURE, 0);
  }
}

/**
 * Initializes the nRF24's configuration registers.
 */
void nRF24Init() {
	// disable the chip before configuring the device
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);

	uint8_t hello = nRF24ReadReg(STATUS);

	nRF24Reset(0);

	nRF24WriteReg(CONFIG, 0);

	uint8_t test = nRF24ReadReg(CONFIG);

	nRF24WriteReg(EN_AA, 0); // don't need auto-ack

	nRF24WriteReg(EN_RXADDR, 0);

	nRF24WriteReg(SETUP_AW, 0b11); // 5 bytes for TX/RX address

	nRF24WriteReg(SETUP_RETR, 0); // not using auto-ack

	nRF24WriteReg(RF_CH, 0); // will be setup during TX or RX

	nRF24WriteReg(RF_SETUP, 0b1110); // power = 2db, data rate = 2Mbps

	// enable the chip again after configuring
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

/**
 * Prepares the nRF24 to transmit to a particular address and channel.
 *
 * @param address: The address to transmit to.
 * @param channel: The channel to transmit to.
 */
void nRF24TxMode(uint8_t* address, uint8_t channel) {
  // disable the chip before configuring the Tx mode
  HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);

  nRF24WriteReg(RF_CH, channel);

  nRF24WriteRegMulti(TX_ADDR, address, 5);

  // power up the device
  uint8_t config = nRF24ReadReg(CONFIG);
  config = config | (0b10);
  nRF24WriteReg(CONFIG, config);

  uint8_t configCheck = nRF24ReadReg(CONFIG);

  // enable the chip again after configuring
  HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

/**
 * Transmits data to the address and channel that was initialized by calling nRF24TxMode().
 * nRF24TxMode() must be called prior to this function to ensure correctness.
 *
 * @param data: The data to transmit.
 *
 * @returns: 0 if transmission was OK, -1 if transmission failed.
 */
int nRF24Transmit(uint8_t* data) {
  HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);

  // tell the nRF24 that you want to send a payload
  // note: can't use nRF24SendCmd() here because we want to keep CS pin low
  uint8_t cmdToSend = W_TX_PAYLOAD;
  HAL_SPI_Transmit(NRF24_SPI, &cmdToSend, 1, 100);

  // send the payload
  HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

  HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);

  HAL_Delay(1);

  // check that the nRF24's outgoing FIFO buffer is empty
  // note that the "buffer empty" bit will be set if the device is disconnected,
  //  so also check that one of the reserved 0 bits is not set

  uint8_t configCheck = nRF24ReadReg(CONFIG);
  uint8_t fifoStatus = nRF24ReadReg(FIFO_STATUS);
  if (!(fifoStatus & 0b10000) || (fifoStatus & 0b1000)) {
    return -1;
  }

  // flush the Tx register to ensure extra data is not accidentally sent
  nRF24SendCmd(FLUSH_TX);

  return 0;
}

/**
 * Prepares the nRF24 to receive data on a particular address and channel.
 *
 * @param address: The address to receive on.
 * @param channel: The channel to receive on.
 */
void nRF24RxMode(uint8_t* address, uint8_t channel) {
  // disable the chip before configuring the Rx mode
  HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);

  nRF24WriteReg(RF_CH, channel);

  // get current enabled pipes so we don't disable ones that are running
  uint8_t en_rxaddr = nRF24ReadReg(EN_RXADDR);

  // enable data pipe 1
  nRF24WriteReg(EN_RXADDR, en_rxaddr | 0b10);
  nRF24WriteRegMulti(RX_ADDR_P1, address, 5);

  nRF24WriteReg(RX_PW_P1, 32); // write 32 byte payload size

  // power up the device
  uint8_t config = nRF24ReadReg(CONFIG);
  config = config | 0b11;
  nRF24WriteReg(CONFIG, config);

  uint8_t configCheck = nRF24ReadReg(CONFIG);

  // enable the chip again after configuring
  HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

/**
 * Checks whether data is present and ready to be processed in a particular data pipe.
 *
 * @param pipeNum: The data pipe to check.
 *
 * @returns: True if data is available, false otherwise.
 */
bool isDataAvailable(uint8_t pipeNum) {
  uint8_t status = nRF24ReadReg(STATUS);

  // return false if data isn't ready or pipe has no data in it
  if (!(status & 0b01000000) || !(status & (pipeNum << 1))) {
    return false;
  }

  // have to write 1 to clear data ready bit
  nRF24WriteReg(STATUS, 0b01000000);
  return true;
}

/**
 * Copies data from the data pipe to a target address.
 *
 * @param data: The address of the buffer to store the pipe data.
 */
void nRF24Receive(uint8_t* data) {
  HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);

  // tell the nRF24 that you want to receive data from data pipe
  // note: can't use nRF24SendCmd() here because we want to keep CS pin low
  uint8_t cmdToSend = R_RX_PAYLOAD;
  HAL_SPI_Transmit(NRF24_SPI, &cmdToSend, 1, 100);

  // read the payload
  HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

  HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);

  HAL_Delay(1);

  // flush the receive data pipe
  nRF24SendCmd(FLUSH_RX);
}
