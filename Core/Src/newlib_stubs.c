#include <stdio.h>
#include "stm32l4xx_ll_usart.h"


int _read(int file, char *ptr, int len) {
    (void)file;

    for (int i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_RXNE_RXFNE(USART1)) {}
        *(ptr++) = (char)LL_USART_ReceiveData8(USART1);
    }

    return len;
}

int _write(int file, char *ptr, int len) {
    (void)file;

    for (int i = 0; i < len; i++) {
        while (!(LL_USART_IsActiveFlag_TC(USART1) || LL_USART_IsActiveFlag_TXE_TXFNF(USART1))) {}
        LL_USART_TransmitData8(USART1, *(ptr++));
    }

    return len;
}

int getchar(void) {
    int c = 0;
    _read(0, (char *)(&c), 1);
    return c;
}
