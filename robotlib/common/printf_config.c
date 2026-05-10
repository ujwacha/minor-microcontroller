#include "processor_type.h"

#ifdef STDIO_USB
#include "usbd_cdc_if.h"

/*Redirect printf to USB CDC. We can see output on Serial Monitor*/
int _write(int file, char *data, int len)
{
    CDC_Transmit_FS((uint8_t *)data, (uint16_t)len);
    return len;
}

#else
/*Redirect printf to ITM. We can see output on Serial Wire Viewer on STM32CubeProgrammer*/
int _write(int file, char *data, int len)
{
    for (int i = 0; i < len; ++i)
    {
        ITM_SendChar(data[i]);
    }
    return len;
}
#endif