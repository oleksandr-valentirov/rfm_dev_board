#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "main.h"  /* for MCU related SPI libs */

#include "rfm69.h"
#include "rfm69_registers.h"

/* defines */

/* basic SPI RFM functions */
void rfm_write(uint8_t addr, uint8_t *ptr, uint8_t len);
void rfm_read(uint8_t addr, uint8_t *ptr, uint8_t len);

void rfm_set_modulation(uint8_t data_mode, uint8_t type, uint8_t filter) {
    uint8_t data = (((data_mode & 3) << 5) | ((type & 3) << 3) | (filter & 3));

    rfm_write(RFM69_RegDataModul, & data, 1);
}

void rfm_run_osc_calib(void) {
    uint8_t data = 1 << 7;

    rfm_write(RFM69_RegOsc1, &data, 1);
}

uint8_t rfm_is_calib_finished(void) {
    uint8_t res = 0;

    rfm_read(RFM69_RegOsc1, &res, 1);

    return !!(res & 64);
}

void rfm_config_fei(void) {
    uint8_t temp = 3 << 2;

    rfm_write(RFM69_RegAfcFei, &temp, 1);
}

void rfm_get_rssi(uint8_t *dst) {
    rfm_read(RFM69_RegRssiValue, (uint8_t *)dst, 1);
}

void rfm_get_irq_flags(uint16_t *dst) {
    rfm_read(RFM69_RegIrqFlags1, (uint8_t *)dst, 2);
}

/* @brief
 * @param   impedance
 *          0 - 50 Ohm
 *          1 - 200 Ohm
 */
void rfm_set_lna(uint8_t impedance, uint8_t gain) {
    uint8_t temp = (impedance & 1) << 7 | (gain & 7);

    rfm_write(RFM69_RegLna, &temp, 1);
}

void rfm_set_pa(uint8_t pa, uint8_t out) {
    uint8_t temp = (pa << 5) | (out & 31);

    rfm_write(RFM69_RegPaLevel, &temp, 1);
}


/*
 *  @brief  changes carrier frequency. default freq is 915 MHz
 *  @param  calculated_carrier = freq / 61.03515625
 */
void rfm_set_carrier(uint32_t calculated_carrier) {
    uint8_t data[3] = {0, 0, 0};

    /* freq = freqs[Random_FromTS(HAL_GetTick()) % CH_NUM]; */
    data[0] = (uint8_t)((calculated_carrier >> 16) & 0xFF);
    data[1] = (uint8_t)((calculated_carrier >> 8) & 0xFF);
    data[2] = (uint8_t)(calculated_carrier & 0xFF);

    rfm_write(RFM69_RegFrfMsb, data, 3);
}

/*
 *  @brief  changes mode of operation - sleep, standby, fs, tx, rx
 */
void rfm_set_mode(rfm69_mode_t mode) {
    uint8_t data = 0;

    if (mode != LISTEN)
        data = ((uint8_t) mode) << 2;
    else
        data = 1 << 6;

    rfm_write(RFM69_RegOpMode, &data, 1);
}

/*
 *  @brief  sets payload length (for which mode ??)
 */
void rfm_set_payload_length(uint8_t value) {
    rfm_write(RFM69_RegPayloadLength, &value, 1);
}

/*
 *  @brief  sets broadcast address
 */
void rfm_set_broadcast_addr(uint8_t addr) {
    rfm_write(RFM69_RegBroadcastAdrs, &addr, 1);
}

/*
 *  @brief  sets current device address
 */
void rfm_set_node_addr(uint8_t addr) {
    rfm_write(RFM69_RegNodeAdrs, &addr, 1);
}

/*
 *  @brief  configs Sync word
 ToDo - check if it works fine
 */
uint8_t rfm_config_sync(uint8_t enable, uint8_t length, uint8_t err_tol, uint8_t *data_ptr) {
    uint8_t data[9] = {0};

    /* actual size in RFM is length + 1 because 1 sync byte is always enabled */
    if (length < 1 || length > 8)
        return 1;

    data[0] = ((enable & 1) << 7) | ((length - 1) << 3) | (err_tol & 7);
    memcpy(data + 1, data_ptr, length);

    rfm_write(RFM69_RegSyncConfig, data, length + 1);

    return 0;
}

/*
 *  @brief  Configs DIO functions; ClockOut remains OFF
 *  @param  dio DIO num 0-5
 *  @param  val dio state value 0-3
 *  @retval 0 - OK
 *  @retval 1 - bad dio value
 */
uint8_t rfm_set_dio_mapping(uint8_t dio, uint8_t val) {
    uint8_t dio_map1_state = 0, dio_map2_state = 7;  /* default values of DIO registers */
    uint8_t data = 0;
    uint8_t addr = 0;

    if (dio <= 3)
        addr = RFM69_RegDioMapping1;
    else if (dio <= 5)
        addr = RFM69_RegDioMapping2;
    else
        return 1;

    switch (dio) {
    case 0:
        dio_map1_state |= ((val & 3) << 6);
        data = dio_map1_state;
        break;
    case 1:
        dio_map1_state |= ((val & 3) << 4);
        data = dio_map1_state;
        break;
    case 2:
        dio_map1_state |= ((val & 3) << 2);
        data = dio_map1_state;
        break;
    case 3:
        dio_map1_state |= (val & 3);
        data = dio_map1_state;
        break;
    case 4:
        dio_map2_state |= ((val & 3) << 6);
        data = dio_map2_state;
        break;
    case 5:
        dio_map2_state |= ((val & 3) << 4);
        data = dio_map2_state;
        break;
    default:
        return 1;
    }

    rfm_write(addr, &data, 1);

    return 0;
}

/*
 *  @brief  set preable length (a length of sequence of 1 an 0 in bytes)
 *  @param  len length of the preamble
 */
void rfm_set_preamble_length(uint16_t len) {
    uint8_t data[2] = {(uint8_t)((len >> 8) & 0xFF), (uint8_t)(len & 0xFF)};

    rfm_write(RFM69_RegPreambleMsb, data, 2);
}

/*
 *  @brief  set preable length (a length of sequence of 1 an 0 in bytes)
 *  @param  len length value
 */
void rfm_set_bit_rate(uint8_t msb, uint8_t lsb) {
    uint8_t data[2] = {msb, lsb};

    rfm_write(RFM69_RegBitrateMsb, data, 2);
}


/*
 *  @brief  configs packet mode
 *  @param  pm_fixed_payload_length
 *          0 - Fixed length; 1 - Variable length
 *  @param  dc_free
 *  @param  crc_on
 *          0 - disables CRC; 1 - enables CRC
 *  @param  crc_auto_clear_off
 * 
 *  @param  addr_filtering Defines address based filtering in Rx
 *          00 → None (Off)
 *          01 → Address field must match NodeAddress
 *          10 → Address field must match NodeAddress or BroadcastAddress
 */
void rfm_set_packet_config1(uint8_t pm_fixed_payload_length, uint8_t dc_free, uint8_t crc_on, uint8_t crc_auto_clear_off, uint8_t addr_filtering) {
    uint8_t data = 0;

    data = (pm_fixed_payload_length & 1) << 7;
    data |= (dc_free & 3) << 5;
    data |= (crc_on & 1) << 4;
    data |= (crc_auto_clear_off & 1) << 3;
    data |= (addr_filtering & 3) << 1;

    rfm_write(RFM69_RegPacketConfig1, &data, 1);
}

void rfm_set_packet_config2(uint8_t delay, uint8_t force_rx, uint8_t auto_rx, uint8_t aes_on) {
    uint8_t data = ((delay & 15) << 4 | (force_rx & 1) << 3 | (auto_rx & 1) << 1 | (aes_on & 1));

    rfm_write(RFM69_RegPacketConfig2, &data, 1);
}

/*
 *  @brief  configs fifo workflow
 *  @param  fifo_mode - Defines the condition to start packet transmission 
 *          0 - the number of bytes in the FIFO exceeds FifoThreshold
 *          1 - FifoNotEmpty
 */
void rfm_set_config_fifo(uint8_t fifo_mode, uint8_t fifo_threshold) {
    uint8_t data = ((fifo_mode & 1) << 7) | (fifo_threshold & 127);

    rfm_write(RFM69_RegFifoThresh, &data, 1);
}

/*
 * @brief   writes data to RFM FIFO
 */
void rfm_transmit_data(uint8_t *data_ptr, uint8_t len) {
    rfm_write(RFM69_RegFifo, data_ptr, len);
}

/*
 * @brief   read data from RFM FIFO
 */
void rfm_receive_data(uint8_t *data_ptr, uint8_t len) {
    rfm_read(RFM69_RegFifo, data_ptr, len);
}

__weak void rfm_write(uint8_t addr, uint8_t *ptr, uint8_t len) {
    (void)addr;
    (void)(ptr);
    (void)(len);
}

__weak void rfm_read(uint8_t addr, uint8_t *ptr, uint8_t len) {
    (void)addr;
    (void)(ptr);
    (void)(len);
}
