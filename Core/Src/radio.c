#include <stdio.h>
#include "radio.h"

/* defines */
#define BROADCAST_ADDR  255

/* types */
typedef enum radio_state {
    IDLE = 0,
    CONFIG,
    TX,
    RX
} radio_state_t;

typedef struct radio_header {
    uint8_t length;
    uint8_t addr;
} __attribute__((packed)) radio_header_t;

typedef struct radio_broadcast {
    uint32_t flags;
    uint32_t clock;
} __attribute__((packed)) radio_broadcast_t;


void RFM_Routine(void) {
    static radio_state_t state = IDLE;
    uint8_t c[sizeof(radio_header_t) + sizeof(radio_broadcast_t)] = {0};
    radio_header_t *header = (radio_header_t *)c;
    radio_broadcast_t *payload = (radio_broadcast_t *)(c + sizeof(header));
    static uint16_t irq_flags = 0, irq_flags_old = 0;
    uint8_t rssi = 0;

    switch (state) {
        case IDLE:
            break;
        case CONFIG:
            break;
        case TX:
            break;
        case RX:
            break;
    }

    // while (!LL_GPIO_IsInputPinSet(RFM_DIO3_GPIO_Port, RFM_DIO3_Pin)) {}

    rfm_get_irq_flags(&irq_flags);
    // if (irq_flags != irq_flags_old) {
    //     printf("flags 0x%04X\r\n", irq_flags);
    //     irq_flags_old = irq_flags;
    // }

    if (irq_flags & 0x0001) {
        rfm_get_rssi(&rssi);
        printf("rssi -%d dBm\r\n", rssi / 2);
    }

    if (irq_flags & 0x0200) {
        rfm_receive_data(c, 10);
        printf("clock %d ms\r\n", payload->clock);
    }
}

uint8_t RFM_Init(uint8_t network_id, uint8_t node_id) {
    (void)network_id;
    uint8_t version = 0;
    uint8_t sync_val[] = {'h', 'e', 'l', 'l'};

    // LL_SPI_Enable(RFM_SPI);

    rfm_read_version(&version);
    if (version != RFM_VERSION)
      return 1;

    rfm_set_node_addr(node_id);
    rfm_set_broadcast_addr(BROADCAST_ADDR);
    rfm_set_packet_config1(1, 1, 1, 0, 2);
    rfm_set_carrier(14221312);  /* 868 MHz */
    // rfm_set_payload_length(9);
    rfm_set_config_fifo(0, 9);
    if(rfm_config_sync(1, 4, 0, sync_val))
        return 1;
    rfm_set_bit_rate(0x0D, 0x05);
    rfm_set_preamble_length(5);

    rfm_set_dio_mapping(0, 1);
    rfm_set_dio_mapping(3, 2);
    rfm_set_pa(3, 10);
    rfm_set_lna(0, 0);
    rfm_config_fei();

    rfm_set_modulation(0, 0, 2);

    rfm_run_osc_calib();
    while (!rfm_is_calib_finished()) {}

    rfm_set_mode(RECEIVE);

    return 0;
}
