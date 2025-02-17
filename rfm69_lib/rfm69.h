#pragma once

#include <stdint.h>


/* types */
typedef enum rfm69_mode {
    SLEEP = 0,
    STANDBY,
    FS,
    TRANSMIT,
    RECEIVE,
    LISTEN
} rfm69_mode_t;

typedef enum rfm69_state {
    IDLE = 0,
    CONFIG,
    TX,
    RX
} rfm69_state_t;

/* getters */
void rfm_get_rssi(uint8_t *dst);
void rfm_get_irq_flags(uint16_t *dst);
uint8_t rfm_is_calib_finished(void);
/* config functions */
void rfm_set_modulation(uint8_t data_mode, uint8_t type, uint8_t filter);
void rfm_run_osc_calib(void);
void rfm_set_packet_config2(uint8_t delay, uint8_t force_rx, uint8_t auto_rx, uint8_t aes_on);
void rfm_config_fei(void);
void rfm_set_lna(uint8_t impedance, uint8_t gain);
void rfm_set_pa(uint8_t pa, uint8_t out);
void rfm_set_carrier(uint32_t calculated_carrier);
void rfm_set_mode(rfm69_mode_t mode);
void rfm_set_payload_length(uint8_t value);
void rfm_set_broadcast_addr(uint8_t addr);
void rfm_set_node_addr(uint8_t addr);
uint8_t rfm_config_sync(uint8_t enable, uint8_t length, uint8_t err_tol, uint8_t *data_ptr);
void rfm_set_config_fifo(uint8_t fifo_mode, uint8_t fifo_threshold);
void rfm_set_packet_config1(uint8_t pm_fixed_payload_length, uint8_t dc_free, uint8_t crc_on, uint8_t crc_auto_clear_off, uint8_t addr_filtering);
uint8_t rfm_set_dio_mapping(uint8_t dio, uint8_t val);
void rfm_set_preamble_length(uint16_t len);
void rfm_set_bit_rate(uint8_t msb, uint8_t lsb);
/* data tx/rx */
void rfm_transmit_data(uint8_t *data_ptr, uint8_t len);
void rfm_receive_data(uint8_t *data_ptr, uint8_t len);