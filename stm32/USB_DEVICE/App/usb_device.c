#include "usb_device.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"
#include "main.h"
USBD_HandleTypeDef hUsbDeviceFS;
void MX_USB_DEVICE_Init(void) {
  if (USBD_Init(&hUsbDeviceFS, &VCP_Desc, 0) != USBD_OK ||
      USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK ||
      USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK ||
      USBD_Start(&hUsbDeviceFS) != USBD_OK) Error_Handler();
}
