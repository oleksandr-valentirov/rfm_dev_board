#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "rfm69_registers.h"

/* defines */
#define RFM_SPI         (SPI2)
#define rfm_cs_low()    LL_GPIO_ResetOutputPin(RFM_CS_GPIO_Port, RFM_CS_Pin)
#define rfm_cs_high()   LL_GPIO_SetOutputPin(RFM_CS_GPIO_Port, RFM_CS_Pin)

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

/* static variables */
// extern SPI_HandleTypeDef hspi2;

/* basic SPI RFM functions */
static void rfm_write(uint8_t addr, uint8_t *ptr, uint8_t len);
static void rfm_read(uint8_t addr, uint8_t *ptr, uint8_t len);
/* config functions */
static void rfm_set_pa(uint8_t pa, uint8_t out);
static void rfm_set_carrier(uint32_t calculated_carrier);
static void rfm_set_mode(rfm69_mode_t mode);
static void rfm_set_payload_length(uint8_t value);
static void rfm_set_broadcast_addr(uint8_t addr);
static void rfm_set_node_addr(uint8_t addr);
static uint8_t rfm_config_sync(uint8_t enable, uint8_t length, uint8_t err_tol, uint8_t *data_ptr);
static void rfm_set_config_fifo(uint8_t fifo_mode, uint8_t fifo_threshold);
static void rfm_set_packet_config1(uint8_t pm_fixed_payload_length, uint8_t dc_free, uint8_t crc_on, uint8_t crc_auto_clear_off, uint8_t addr_filtering);
static uint8_t rfm_set_dio_mapping(uint8_t dio, uint8_t val);
static void rfm_set_preamble_length(uint16_t len);
static void rfm_set_bit_rate(uint8_t msb, uint8_t lsb);
/* data tx/rx */
static void rfm_transmit_data(uint8_t *data_ptr, uint8_t len);
static void rfm_receive_data(uint8_t *data_ptr, uint8_t len);

uint8_t RFM_Init(uint8_t network_id, uint8_t node_id) {
    uint8_t version = RFM69_RegVersion;
    uint8_t sync_val[] = {73, 27, 27, 73};
    uint8_t c = 0;

    LL_SPI_Enable(RFM_SPI);

    rfm_read(RFM69_RegVersion, &version, 1);
    if (version != RFM_VERSION)
        return 1;

    rfm_set_node_addr(node_id);
    rfm_set_broadcast_addr(255);
    rfm_set_packet_config1(0, 0, 0, 0, 0);
    rfm_set_carrier(14221312);  /* 868 MHz */
    rfm_set_payload_length(5);
    rfm_set_config_fifo(1, 5);
    if(rfm_config_sync(1, 4, 0, sync_val))
        return 1;
    rfm_set_bit_rate(0x0D, 0x05);
    rfm_set_preamble_length(4);

    rfm_set_dio_mapping(0, 0);
    rfm_set_pa(3, 10);

    rfm_set_mode(RECEIVE);

    return 0;
}

void RFM_Routine(void) {
    static rfm69_state_t state = IDLE;
    uint8_t c[5] = {0};

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

    /* TX example */
    // rfm_transmit_data((uint8_t *)"hello", 5);
    // rfm_set_mode(TRANSMIT);
    // while(!LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin)) {}
    // rfm_set_mode(STANDBY);

    /* RX example */
    if (!LL_GPIO_IsInputPinSet(RFM_DIO0_GPIO_Port, RFM_DIO0_Pin)) {
        rfm_receive_data(c, 5);
        printf(">>>%s\r\n", c);
    }
}

static void rfm_set_pa(uint8_t pa, uint8_t out) {
    uint8_t temp = (pa << 5) | (out & 31);

    rfm_write(RFM69_RegPaLevel, &temp, 1);
}


/*
 *  @brief  changes carrier frequency. default freq is 915 MHz
 *  @param  calculated_carrier = freq / 61.03515625
 */
static void rfm_set_carrier(uint32_t calculated_carrier) {
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
static void rfm_set_mode(rfm69_mode_t mode) {
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
static void rfm_set_payload_length(uint8_t value) {
    rfm_write(RFM69_RegPayloadLength, &value, 2);
}

/*
 *  @brief  sets broadcast address
 */
static void rfm_set_broadcast_addr(uint8_t addr) {
    rfm_write(RFM69_RegBroadcastAdrs, &addr, 1);
}

/*
 *  @brief  sets current device address
 */
static void rfm_set_node_addr(uint8_t addr) {
    rfm_write(RFM69_RegNodeAdrs, &addr, 1);
}

/*
 *  @brief  configs Sync word
 ToDo - check if it works fine
 */
static uint8_t rfm_config_sync(uint8_t enable, uint8_t length, uint8_t err_tol, uint8_t *data_ptr) {
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
static uint8_t rfm_set_dio_mapping(uint8_t dio, uint8_t val) {
    static uint8_t dio_map1_state = 0, dio_map2_state = 7;  /* default values of DIO registers */
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
        break;
    }

    rfm_write(addr, &data, 1);

    return 0;
}

/*
 *  @brief  set preable length (a length of sequence of 1 an 0 in bytes)
 *  @param  len length of the preamble
 */
static void rfm_set_preamble_length(uint16_t len) {
    uint8_t data[2] = {(uint8_t)((len >> 8) & 0xFF), (uint8_t)(len & 0xFF)};

    rfm_write(RFM69_RegPreambleMsb, data, 2);
}

/*
 *  @brief  set preable length (a length of sequence of 1 an 0 in bytes)
 *  @param  len length value
 */
static void rfm_set_bit_rate(uint8_t msb, uint8_t lsb) {
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
static void rfm_set_packet_config1(uint8_t pm_fixed_payload_length, uint8_t dc_free, uint8_t crc_on, uint8_t crc_auto_clear_off, uint8_t addr_filtering) {
    uint8_t data = 0;

    data = (pm_fixed_payload_length & 1) << 7;
    data |= (dc_free & 3) << 5;
    data |= (crc_on & 1) << 4;
    data |= (crc_auto_clear_off & 1) << 3;
    data |= (addr_filtering & 3) << 1;

    rfm_write(RFM69_RegPacketConfig1, &data, 1);
}

/*
 *  @brief  configs fifo workflow
 *  @param  fifo_mode - Defines the condition to start packet transmission 
 *          0 - the number of bytes in the FIFO exceeds FifoThreshold
 *          1 - FifoNotEmpty
 */
static void rfm_set_config_fifo(uint8_t fifo_mode, uint8_t fifo_threshold) {
    uint8_t data = ((fifo_mode & 1) << 7) | (fifo_threshold & 127);

    rfm_write(RFM69_RegFifoThresh, &data, 1);
}

/*
 * @brief   writes data to RFM FIFO
 */
static void rfm_transmit_data(uint8_t *data_ptr, uint8_t len) {
    rfm_write(RFM69_RegFifo, data_ptr, len);
}

/*
 * @brief   read data from RFM FIFO
 */
static void rfm_receive_data(uint8_t *data_ptr, uint8_t len) {
    rfm_read(RFM69_RegFifo, data_ptr, len);
}

static void rfm_write(uint8_t addr, uint8_t *ptr, uint8_t len) {
    rfm_cs_low();
    // uint8_t temp = addr | 128;
    // delay_ms_poll(10);
    // /* send addr with write bit */
    while (!LL_SPI_IsActiveFlag_TXE(RFM_SPI)) {}
    LL_SPI_TransmitData8(RFM_SPI, addr | 128);
    while (LL_SPI_IsActiveFlag_BSY(RFM_SPI)) {}

    while (len--) {
        while (!LL_SPI_IsActiveFlag_TXE(RFM_SPI)) {}
        LL_SPI_TransmitData8(RFM_SPI, *(ptr++));
    }

    while (!LL_SPI_IsActiveFlag_TXE(RFM_SPI)) {}
    while (LL_SPI_IsActiveFlag_BSY(RFM_SPI)) {}

    // HAL_SPI_Transmit(&hspi2, &temp, 1, 100);
    // HAL_SPI_Transmit(&hspi2, ptr, len, 100);

    rfm_cs_high();
}

static void rfm_read(uint8_t addr, uint8_t *ptr, uint8_t len) {
    rfm_cs_low();
    // delay_ms_poll(10);
    while (!LL_SPI_IsActiveFlag_TXE(RFM_SPI)) {}
    LL_SPI_TransmitData8(RFM_SPI, addr);
    while (LL_SPI_IsActiveFlag_BSY(RFM_SPI)) {}

    /* dummy byte reading */
    (void)RFM_SPI->DR;

    while (len--) {
        /* dummy data to generate clock */
        while (!LL_SPI_IsActiveFlag_TXE(RFM_SPI)) {}
        LL_SPI_TransmitData8(RFM_SPI, 0xFF);
        while (LL_SPI_IsActiveFlag_BSY(RFM_SPI)) {}

        while (!LL_SPI_IsActiveFlag_RXNE(RFM_SPI)) {}
        *(ptr++) = LL_SPI_ReceiveData8(RFM_SPI);
    }

    // HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
    // HAL_SPI_Receive(&hspi2, ptr, len, 100);

    rfm_cs_high();
}
