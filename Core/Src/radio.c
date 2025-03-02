#include <stdio.h>
#include "radio.h"
#include "main.h"
#include "radio_protocol.h"

/* defines */
#define BROADCAST_ADDR      255
#define PAIRING_TIMEOUT_MS  60000000

/* types */
typedef enum radio_state {
    CONN = 0,
    NO_CONN
} radio_state_t;

typedef struct rfm_header {
    uint8_t length;
} __attribute__((packed)) rfm_header_t;

typedef struct radio_broadcast {
    uint8_t addr;
    uint32_t flags;
    uint32_t clock;
} __attribute__((packed)) rfm_broadcast_t;

/* variables */
static uint8_t tx_buffer[256] = {0};


void RFM_Routine(void) {
    radio_state_t state = NO_CONN;  /* make static */
    uint32_t hub_id = 0;
    rfm_header_t *header = (rfm_header_t *)tx_buffer;
    rfm_broadcast_t *payload = (rfm_broadcast_t *)(tx_buffer + sizeof(header));
    protocol_pairing_t *pairing = (protocol_pairing_t *)(tx_buffer + sizeof(header));

    static uint16_t irq_flags = 0;
    uint8_t rssi = 0;

    switch (state) {
        case CONN:
            rfm_set_dio_mapping(4, 2);
            rfm_set_dio_mapping(0, 2);
            rfm_set_mode(RECEIVE);
            while (!LL_GPIO_IsInputPinSet(RFM_DIO4_GPIO_Port, RFM_DIO4_Pin)) {}
            delay_ms_it(50);  /* todo - fix this shit */
            while (!LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin) && !get_delay_ms_flag()) {}
        
            if (!LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin)) {
                rfm_set_mode(STANDBY);
                do {
                    rfm_get_irq_flags(&irq_flags);
                } while ((irq_flags & 128) == 0);
                return;
            }
        
            rfm_set_dio_mapping(0, 0);
            delay_ms_it(50);  /* todo - fix this shit */
            while (!LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin) && !get_delay_ms_flag()) {}
            if (LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin)) {
                rfm_receive_data(tx_buffer, 12);
                printf("clock %lu ms %lu ms\r\n", payload->clock, get_rfm_counter());
            } else
                printf("crc timeout\r\n");
        
            rfm_set_mode(STANDBY);
            do {
                rfm_get_irq_flags(&irq_flags);
            } while ((irq_flags & 128) == 0);

            break;
        
        case NO_CONN:
            /* config */

            /* wait for a msg on 868 MHz */
            do {
                if (hub_id) {
                    /* wait for a broadcast on specific channel during a time slot */
                    rfm_set_packet_config1(1, 1, 1, 0, 2);  /* enable RFM addr filter */
                }

                rfm_set_dio_mapping(4, 2);
                rfm_set_dio_mapping(0, 2);
                rfm_set_packet_config1(1, 1, 1, 0, 0);  /* disable RFM addr filter */

                /* wait for sync */
                rfm_set_mode(RECEIVE);
                while (!LL_GPIO_IsInputPinSet(RFM_DIO4_GPIO_Port, RFM_DIO4_Pin)) {}
                delay_ms_it(50);  /* todo - fix this shit */
                while (!LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin) && !get_delay_ms_flag()) {}

                if (LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin)) {
                    /* get payload */
                    rfm_set_dio_mapping(0, 0);
                    delay_ms_it(50);  /* todo - fix this shit */
                    while (!LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin) && !get_delay_ms_flag()) {}
                    if (LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin)) {
                        rfm_receive_data(tx_buffer, 12);
                        printf("hub id 0x%08X\r\n", pairing->header.hub_id);
                        state = CONN;
                    }
                    break;
                } else {
                    rfm_set_mode(STANDBY);
                    do {
                        rfm_get_irq_flags(&irq_flags);
                    } while ((irq_flags & 128) == 0);
                }

                rfm_set_mode(STANDBY);
                do {
                    rfm_get_irq_flags(&irq_flags);
                } while ((irq_flags & 128) == 0);

            } while (!state);

            break;
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
    // rfm_set_packet_config2(5, 0, 1, 0);
    rfm_set_carrier(14221312);  /* 868 MHz */
    // rfm_set_payload_length(10);
    // rfm_set_config_fifo(0, 9);
    if(rfm_config_sync(1, 4, 0, sync_val))
        return 1;
    rfm_set_bit_rate(0x0D, 0x05);
    rfm_set_preamble_length(6);

    rfm_set_pa(3, 10);
    rfm_set_lna(0, 0);
    rfm_config_fei();

    rfm_set_modulation(0, 0, 2);

    rfm_run_osc_calib();
    while (!rfm_is_calib_finished()) {}

    return 0;
}
