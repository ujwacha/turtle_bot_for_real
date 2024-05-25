#include "stm32f4xx_hal.h"

#ifdef STDIO_USB

#include "usbd_cdc_if.h"

int _write(int file, char *data, int len)
{
    CDC_Transmit_FS((uint8_t*)data, (uint16_t)len);
    return len;
}

#else

int _write(int file, char *data, int len)
{
    int i  = 0;
    for (; i < len; ++i)
    {
        ITM_SendChar(data[i]);
    }
    return i;
}

#endif
