/**
 ******************************************************************************
 * @file    usbh_hid_core.c
 * @author  MCD Application Team
 * @version V2.2.1
 * @date    17-March-2018
 * @brief   This file is the HID Layer Handlers for USB Host HID class.
 *
 * @verbatim
 *
 *          ===================================================================
 *                                HID Class  Description
 *          ===================================================================
 *           This module manages the MSC class V1.11 following the "Device Class Definition
 *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
 *           This driver implements the following aspects of the specification:
 *             - The Boot Interface Subclass
 *             - The Mouse and Keyboard protocols
 *
 *  @endverbatim
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                      <http://www.st.com/SLA0044>
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbh_hid_core.h"
#include "usbh_hid_mouse.h"
#include "usbh_hid_keybd.h"
#include "usbh_hid_gamepad.h"
/** @addtogroup USBH_LIB
 * @{
 */

/** @addtogroup USBH_CLASS
 * @{
 */

/** @addtogroup USBH_HID_CLASS
 * @{
 */

/** @defgroup USBH_HID_CORE
 * @brief    This file includes HID Layer Handlers for USB Host HID class.
 * @{
 */

/** @defgroup USBH_HID_CORE_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBH_HID_CORE_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup USBH_HID_CORE_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup USBH_HID_CORE_Private_Variables
 * @{
 */
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN HID_Machine_TypeDef HID_Machine __ALIGN_END;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN HID_Report_TypeDef HID_Report __ALIGN_END;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_Setup_TypeDef HID_Setup __ALIGN_END;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USBH_HIDDesc_TypeDef HID_Desc __ALIGN_END;

__IO uint8_t start_toggle = 0;
/**
 * @}
 */

/** @defgroup USBH_HID_CORE_Private_FunctionPrototypes
 * @{
 */

static USBH_Status USBH_HID_InterfaceInit(USB_OTG_CORE_HANDLE *pdev,
                                          void *phost);

static void USBH_ParseHIDDesc(USBH_HIDDesc_TypeDef *desc, uint8_t *buf);

static void USBH_HID_InterfaceDeInit(USB_OTG_CORE_HANDLE *pdev,
                                     void *phost);

static USBH_Status USBH_HID_Handle(USB_OTG_CORE_HANDLE *pdev,
                                   void *phost);

static USBH_Status USBH_HID_ClassRequest(USB_OTG_CORE_HANDLE *pdev,
                                         void *phost);

static USBH_Status USBH_Get_HID_ReportDescriptor(USB_OTG_CORE_HANDLE *pdev,
                                                 USBH_HOST *phost,
                                                 uint16_t length);

static USBH_Status USBH_Get_HID_Descriptor(USB_OTG_CORE_HANDLE *pdev,
                                           USBH_HOST *phost,
                                           uint16_t length);

static USBH_Status USBH_Set_Idle(USB_OTG_CORE_HANDLE *pdev,
                                 USBH_HOST *phost,
                                 uint8_t duration,
                                 uint8_t reportId);

static USBH_Status USBH_Set_Protocol(USB_OTG_CORE_HANDLE *pdev,
                                     USBH_HOST *phost,
                                     uint8_t protocol);

USBH_Class_cb_TypeDef HID_cb =
    {
        USBH_HID_InterfaceInit,
        USBH_HID_InterfaceDeInit,
        USBH_HID_ClassRequest,
        USBH_HID_Handle};
/**
 * @}
 */

/** @defgroup USBH_HID_CORE_Private_Functions
 * @{
 */

/**
 * @brief  USBH_HID_InterfaceInit
 *         The function init the HID class.
 * @param  pdev: Selected device
 * @param  hdev: Selected device property
 * @retval  USBH_Status :Response for USB HID driver initialization
 */
static USBH_Status USBH_HID_InterfaceInit(USB_OTG_CORE_HANDLE *pdev,
                                          void *phost)
{
  uint8_t maxEP;
  USBH_HOST *pphost = phost;

  uint8_t num = 0;
  USBH_Status status = USBH_BUSY;
  HID_Machine.state = HID_ERROR;

  // Print all interfaces info
  printf("\r\n[HID_Init] Scanning interfaces:\r\n");
  for(int i=0; i<4; i++)
  {
    if(pphost->device_prop.Itf_Desc[i].bLength != 0)
    {
      printf("  Interface[%d]: Num=%d, Class=0x%02x, SubClass=0x%02x, Protocol=0x%02x, EPs=%d\r\n",
             i,
             pphost->device_prop.Itf_Desc[i].bInterfaceNumber,
             pphost->device_prop.Itf_Desc[i].bInterfaceClass,
             pphost->device_prop.Itf_Desc[i].bInterfaceSubClass,
             pphost->device_prop.Itf_Desc[i].bInterfaceProtocol,
             pphost->device_prop.Itf_Desc[i].bNumEndpoints);
    }
  }

  printf(" ///   USBH_HID_InterfaceInit SubClass=%d   Protocol=%d   ///  ", 
         pphost->device_prop.Itf_Desc[0].bInterfaceSubClass, 
         pphost->device_prop.Itf_Desc[0].bInterfaceProtocol);

  if (pphost->device_prop.Itf_Desc[0].bInterfaceSubClass == HID_BOOT_CODE)
  {
    /*Decode Bootclass Protocol: Mouse or Keyboard*/
    if (pphost->device_prop.Itf_Desc[0].bInterfaceProtocol == HID_KEYBRD_BOOT_CODE)
    {
      HID_Machine.cb = &HID_KEYBRD_cb;
      printf("\r\n[HID_Init] Callback: Keyboard");
    }
    else if (pphost->device_prop.Itf_Desc[0].bInterfaceProtocol == HID_MOUSE_BOOT_CODE)
    {
      HID_Machine.cb = &HID_MOUSE_cb;
      printf("\r\n[HID_Init] Callback: Mouse");
    }

    HID_Machine.state = HID_IDLE;
    printf("\r\n[HID_Init] State set to HID_IDLE(%d)", HID_Machine.state);
    
    HID_Machine.ctl_state = HID_REQ_IDLE;
    HID_Machine.ep_addr = pphost->device_prop.Ep_Desc[0][0].bEndpointAddress;
    HID_Machine.length = pphost->device_prop.Ep_Desc[0][0].wMaxPacketSize;
    HID_Machine.poll = pphost->device_prop.Ep_Desc[0][0].bInterval;
    printf("\r\n[HID_Init] EP: Addr=0x%02x, Size=%d, Interval=%d",
           HID_Machine.ep_addr, HID_Machine.length, HID_Machine.poll);

    if (HID_Machine.poll < HID_MIN_POLL)
    {
      HID_Machine.poll = HID_MIN_POLL;
      printf("\r\n[HID_Init] Adjust interval to min=%d", HID_Machine.poll);
    }

    maxEP = ((pphost->device_prop.Itf_Desc[0].bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ? 
             pphost->device_prop.Itf_Desc[0].bNumEndpoints : USBH_MAX_NUM_ENDPOINTS);
    printf("\r\n[HID_Init] Scanning %d endpoints", maxEP);

    /* Decode endpoint IN and OUT address from interface descriptor */
    for (num = 0; num < maxEP; num++)
    {
      // Use the existing endpoint descriptor type
      USBH_EpDesc_TypeDef *curr_ep = &pphost->device_prop.Ep_Desc[0][num];
      printf("\r\n  Endpoint[%d]: Addr=0x%02x, Attr=0x%02x, Size=%d, Interval=%d",
             num, curr_ep->bEndpointAddress, curr_ep->bmAttributes,
             curr_ep->wMaxPacketSize, curr_ep->bInterval);

      if (curr_ep->bEndpointAddress & 0x80)
      {
        printf(" (IN)");
        HID_Machine.HIDIntInEp = curr_ep->bEndpointAddress;
        HID_Machine.hc_num_in = USBH_Alloc_Channel(pdev, curr_ep->bEndpointAddress);
        printf("\r\n    Alloc IN channel: %d (0xFF=fail)", HID_Machine.hc_num_in);

        /* Open channel for IN endpoint */
        USBH_Status open_st = USBH_Open_Channel(pdev,
                                                HID_Machine.hc_num_in,
                                                pphost->device_prop.address,
                                                pphost->device_prop.speed,
                                                EP_TYPE_INTR,
                                                HID_Machine.length);
        printf("\r\n    Open IN channel: status=%d (0=OK)", open_st);
      }
      else
      {
        printf(" (OUT)");
        HID_Machine.HIDIntOutEp = curr_ep->bEndpointAddress;
        HID_Machine.hc_num_out = USBH_Alloc_Channel(pdev, curr_ep->bEndpointAddress);
        printf("\r\n    Alloc OUT channel: %d (0xFF=fail)", HID_Machine.hc_num_out);

        /* Open channel for OUT endpoint */
        USBH_Status open_st = USBH_Open_Channel(pdev,
                                                HID_Machine.hc_num_out,
                                                pphost->device_prop.address,
                                                pphost->device_prop.speed,
                                                EP_TYPE_INTR,
                                                HID_Machine.length);
        printf("\r\n    Open OUT channel: status=%d (0=OK)", open_st);
      }
    }

    start_toggle = 0;
    status = USBH_OK;
    printf("\r\n[HID_Init] Boot class init done: status=%d", status);
  }
  else if (pphost->device_prop.Itf_Desc[0].bInterfaceSubClass == 0X00 || 
           pphost->device_prop.Itf_Desc[0].bInterfaceSubClass == 93) // Custom HID
  {
    // Custom protocol for gamepad
    if (pphost->device_prop.Itf_Desc[0].bInterfaceProtocol == 0X00 || 
        pphost->device_prop.Itf_Desc[0].bInterfaceProtocol == 1)
    {
			printf("\r\nok");
      HID_Machine.cb = &HID_GAMEPAD_cb; // Gamepad
      printf("\r\n[HID_Init] Callback: Gamepad");
    }

    HID_Machine.state = HID_IDLE;
    printf("\r\n[HID_Init] State set to HID_IDLE(%d)", HID_Machine.state);
    
    HID_Machine.ctl_state = HID_REQ_IDLE;
    HID_Machine.ep_addr = pphost->device_prop.Ep_Desc[0][0].bEndpointAddress;
    HID_Machine.length = pphost->device_prop.Ep_Desc[0][0].wMaxPacketSize;
    HID_Machine.poll = pphost->device_prop.Ep_Desc[0][0].bInterval;
    printf("\r\n[HID_Init] EP: Addr=0x%02x, Size=%d, Interval=%d",
           HID_Machine.ep_addr, HID_Machine.length, HID_Machine.poll);

    if (HID_Machine.poll < HID_MIN_POLL)
    {
      HID_Machine.poll = HID_MIN_POLL;
      printf("\r\n[HID_Init] Adjust interval to min=%d", HID_Machine.poll);
    }

    maxEP = ((pphost->device_prop.Itf_Desc[0].bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ? 
             pphost->device_prop.Itf_Desc[0].bNumEndpoints : USBH_MAX_NUM_ENDPOINTS);
    printf("\r\n[HID_Init] Scanning %d endpoints", maxEP);

    /* Decode endpoint IN and OUT address from interface descriptor */
    for (num = 0; num < maxEP; num++)
    {
      // Use the existing endpoint descriptor type
      USBH_EpDesc_TypeDef *curr_ep = &pphost->device_prop.Ep_Desc[0][num];
      printf("\r\n  Endpoint[%d]: Addr=0x%02x, Attr=0x%02x, Size=%d, Interval=%d",
             num, curr_ep->bEndpointAddress, curr_ep->bmAttributes,
             curr_ep->wMaxPacketSize, curr_ep->bInterval);

      if (curr_ep->bEndpointAddress & 0x80)
      {
        printf(" (IN)");
        HID_Machine.HIDIntInEp = curr_ep->bEndpointAddress;
        HID_Machine.hc_num_in = USBH_Alloc_Channel(pdev, curr_ep->bEndpointAddress);
        printf("\r\n    Alloc IN channel: %d (0xFF=fail)", HID_Machine.hc_num_in);

        /* Open channel for IN endpoint */
        USBH_Status open_st = USBH_Open_Channel(pdev,
                                                HID_Machine.hc_num_in,
                                                pphost->device_prop.address,
                                                pphost->device_prop.speed,
                                                EP_TYPE_INTR,
                                                HID_Machine.length);
        printf("\r\n    Open IN channel: status=%d (0=OK)", open_st);
      }
      else
      {
        printf(" (OUT)");
        HID_Machine.HIDIntOutEp = curr_ep->bEndpointAddress;
        HID_Machine.hc_num_out = USBH_Alloc_Channel(pdev, curr_ep->bEndpointAddress);
        printf("\r\n    Alloc OUT channel: %d (0xFF=fail)", HID_Machine.hc_num_out);

        /* Open channel for OUT endpoint */
        USBH_Status open_st = USBH_Open_Channel(pdev,
                                                HID_Machine.hc_num_out,
                                                pphost->device_prop.address,
                                                pphost->device_prop.speed,
                                                EP_TYPE_INTR,
                                                HID_Machine.length);
        printf("\r\n    Open OUT channel: status=%d (0=OK)", open_st);
      }
    }

    start_toggle = 0;
    status = USBH_OK;
    printf("\r\n[HID_Init] Custom HID init done: status=%d", status);
  }
  else
  {
    pphost->usr_cb->DeviceNotSupported();
    printf("\r\n[HID_Init] Unsupported SubClass=%d", pphost->device_prop.Itf_Desc[0].bInterfaceSubClass);
  }

  printf("\r\n[HID_Init] Return status=%d (0=OK)\r\n", status);
  return status;
}


/**
 * @brief  USBH_HID_InterfaceDeInit
 *         The function DeInit the Host Channels used for the HID class.
 * @param  pdev: Selected device
 * @param  hdev: Selected device property
 * @retval None
 */
void USBH_HID_InterfaceDeInit(USB_OTG_CORE_HANDLE *pdev,
                              void *phost)
{
  // USBH_HOST *pphost = phost;

  if (HID_Machine.hc_num_in != 0x00)
  {
    USB_OTG_HC_Halt(pdev, HID_Machine.hc_num_in);
    USBH_Free_Channel(pdev, HID_Machine.hc_num_in);
    HID_Machine.hc_num_in = 0; /* Reset the Channel as Free */
  }

  if (HID_Machine.hc_num_out != 0x00)
  {
    USB_OTG_HC_Halt(pdev, HID_Machine.hc_num_out);
    USBH_Free_Channel(pdev, HID_Machine.hc_num_out);
    HID_Machine.hc_num_out = 0; /* Reset the Channel as Free */
  }

  start_toggle = 0;
}

/**
 * @brief  USBH_HID_ClassRequest
 *         The function is responsible for handling HID Class requests
 *         for HID class.
 * @param  pdev: Selected device
 * @param  hdev: Selected device property
 * @retval  USBH_Status :Response for USB Set Protocol request
 */
static USBH_Status USBH_HID_ClassRequest(USB_OTG_CORE_HANDLE *pdev, void *phost)
{
  USBH_HOST *pphost = phost;
  USBH_Status status = USBH_BUSY;
  USBH_Status classReqStatus = USBH_BUSY;

  // 英文打印当前状态，解决乱码
  printf("[HID_ClassRequest] Current state: ");
  switch(HID_Machine.ctl_state)
  {
    case HID_IDLE: printf("HID_IDLE\r\n"); break;
    case HID_REQ_GET_HID_DESC: printf("HID_REQ_GET_HID_DESC\r\n"); break;
    case HID_REQ_GET_REPORT_DESC: printf("HID_REQ_GET_REPORT_DESC\r\n"); break;
    case HID_REQ_SET_IDLE: printf("HID_REQ_SET_IDLE\r\n"); break;
    case HID_REQ_SET_PROTOCOL: printf("HID_REQ_SET_PROTOCOL\r\n"); break;
    default: printf("UNKNOWN\r\n");
  }

  switch (HID_Machine.ctl_state)
  {
    // 关键修改：从HID_IDLE直接推进到设置IDLE状态
    case HID_IDLE:
      printf("[HID_ClassRequest] Move from HID_IDLE to HID_REQ_SET_IDLE\r\n");
      HID_Machine.ctl_state = HID_REQ_SET_IDLE;  // 强制切换状态
      break;

    // 跳过获取描述符的步骤（避免失败）
    case HID_REQ_GET_HID_DESC:
    case HID_REQ_GET_REPORT_DESC:
      printf("[HID_ClassRequest] Skip descriptor requests, move to SET_IDLE\r\n");
      HID_Machine.ctl_state = HID_REQ_SET_IDLE;
      break;

    // 处理设置IDLE（保留原有容错）
    case HID_REQ_SET_IDLE:
      classReqStatus = USBH_Set_Idle(pdev, pphost, 0, 0);
      if (classReqStatus == USBH_OK || classReqStatus == USBH_NOT_SUPPORTED)
      {
        printf("[HID_ClassRequest] SET_IDLE done, move to SET_PROTOCOL\r\n");
        HID_Machine.ctl_state = HID_REQ_SET_PROTOCOL;
      }
      break;

    // 处理设置协议（完成后返回成功）
    case HID_REQ_SET_PROTOCOL:
      if (USBH_Set_Protocol(pdev, pphost, 0) == USBH_OK)
      {
        printf("[HID_ClassRequest] All requests done, return USBH_OK\r\n");
        HID_Machine.ctl_state = HID_IDLE;
        status = USBH_OK;  // 最终返回成功，推进到HOST_CLASS
      }
      break;

    default:
      break;
  }

  return status;
}
/**
 * @brief  USBH_HID_Handle
 *         The function is for managing state machine for HID data transfers
 * @param  pdev: Selected device
 * @param  hdev: Selected device property
 * @retval USBH_Status
 */
static USBH_Status USBH_HID_Handle(USB_OTG_CORE_HANDLE *pdev,
                                   void *phost)
{
  USBH_HOST *pphost = phost;
  USBH_Status status = USBH_OK;
	
//	  printf("Enter USBH_HID_Handle, current state: %d\r\n", HID_Machine.state);  // 新增日志

  switch (HID_Machine.state)
  {

  case HID_IDLE:
//		printf("Entering HID_IDLE state\r\n");
    HID_Machine.cb->Init();
    HID_Machine.state = HID_SYNC;
		 break;
  case HID_SYNC:

    /* Sync with start of Even Frame */
    if (USB_OTG_IsEvenFrame(pdev) == TRUE)
    {
      HID_Machine.state = HID_GET_DATA;
    }
    break;

  case HID_GET_DATA:

    USBH_InterruptReceiveData(pdev,
                              HID_Machine.buff,
                              HID_Machine.length,
                              HID_Machine.hc_num_in);
    start_toggle = 1;

    HID_Machine.state = HID_POLL;
    HID_Machine.timer = HCD_GetCurrentFrame(pdev);
    break;

  case HID_POLL:
    if ((HCD_GetCurrentFrame(pdev) - HID_Machine.timer) >= HID_Machine.poll)
    {
      HID_Machine.state = HID_GET_DATA;
    }
    else if (HCD_GetURB_State(pdev, HID_Machine.hc_num_in) == URB_DONE)
    {
      if (start_toggle == 1) /* handle data once */
      {
        start_toggle = 0;
        HID_Machine.cb->Decode(HID_Machine.buff);
      }
    }
    else if (HCD_GetURB_State(pdev, HID_Machine.hc_num_in) == URB_STALL) /* IN Endpoint Stalled */
    {

      /* Issue Clear Feature on interrupt IN endpoint */
      if ((USBH_ClrFeature(pdev,
                           pphost,
                           HID_Machine.ep_addr,
                           HID_Machine.hc_num_in)) == USBH_OK)
      {
        /* Change state to issue next IN token */
        HID_Machine.state = HID_GET_DATA;
      }
    }
    break;

  default:
    break;
  }
  return status;
}

/**
 * @brief  USBH_Get_HID_ReportDescriptor
 *         Issue report Descriptor command to the device. Once the response
 *         received, parse the report descriptor and update the status.
 * @param  pdev   : Selected device
 * @param  Length : HID Report Descriptor Length
 * @retval USBH_Status : Response for USB HID Get Report Descriptor Request
 */
static USBH_Status USBH_Get_HID_ReportDescriptor(USB_OTG_CORE_HANDLE *pdev,
                                                 USBH_HOST *phost,
                                                 uint16_t length)
{

  USBH_Status status;

  status = USBH_GetDescriptor(pdev,
                              phost,
                              USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_STANDARD,
                              USB_DESC_HID_REPORT,
                              pdev->host.Rx_Buffer,
                              length);

  /* HID report descriptor is available in pdev->host.Rx_Buffer.
  In case of USB Boot Mode devices for In report handling ,
  HID report descriptor parsing is not required.
  In case, for supporting Non-Boot Protocol devices and output reports,
  user may parse the report descriptor*/

  return status;
}

/**
 * @brief  USBH_Get_HID_Descriptor
 *         Issue HID Descriptor command to the device. Once the response
 *         received, parse the report descriptor and update the status.
 * @param  pdev   : Selected device
 * @param  Length : HID Descriptor Length
 * @retval USBH_Status : Response for USB HID Get Report Descriptor Request
 */
static USBH_Status USBH_Get_HID_Descriptor(USB_OTG_CORE_HANDLE *pdev,
                                           USBH_HOST *phost,
                                           uint16_t length)
{

  USBH_Status status;

  status = USBH_GetDescriptor(pdev,
                              phost,
                              USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_STANDARD,
                              USB_DESC_HID,
                              pdev->host.Rx_Buffer,
                              length);

  return status;
}

/**
 * @brief  USBH_Set_Idle
 *         Set Idle State.
 * @param  pdev: Selected device
 * @param  duration: Duration for HID Idle request
 * @param  reportID : Targeted report ID for Set Idle request
 * @retval USBH_Status : Response for USB Set Idle request
 */
static USBH_Status USBH_Set_Idle(USB_OTG_CORE_HANDLE *pdev,
                                 USBH_HOST *phost,
                                 uint8_t duration,
                                 uint8_t reportId)
{

  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE |
                                         USB_REQ_TYPE_CLASS;

  phost->Control.setup.b.bRequest = USB_HID_SET_IDLE;
  phost->Control.setup.b.wValue.w = (duration << 8) | reportId;

  phost->Control.setup.b.wIndex.w = 0;
  phost->Control.setup.b.wLength.w = 0;

  return USBH_CtlReq(pdev, phost, 0, 0);
}

/**
 * @brief  USBH_Set_Report
 *         Issues Set Report
 * @param  pdev: Selected device
 * @param  reportType  : Report type to be sent
 * @param  reportID    : Targeted report ID for Set Report request
 * @param  reportLen   : Length of data report to be send
 * @param  reportBuff  : Report Buffer
 * @retval USBH_Status : Response for USB Set Idle request
 */
USBH_Status USBH_Set_Report(USB_OTG_CORE_HANDLE *pdev,
                            USBH_HOST *phost,
                            uint8_t reportType,
                            uint8_t reportId,
                            uint8_t reportLen,
                            uint8_t *reportBuff)
{

  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE |
                                         USB_REQ_TYPE_CLASS;

  phost->Control.setup.b.bRequest = USB_HID_SET_REPORT;
  phost->Control.setup.b.wValue.w = (reportType << 8) | reportId;

  phost->Control.setup.b.wIndex.w = 0;
  phost->Control.setup.b.wLength.w = reportLen;

  return USBH_CtlReq(pdev, phost, reportBuff, reportLen);
}

/**
 * @brief  USBH_Set_Protocol
 *         Set protocol State.
 * @param  pdev: Selected device
 * @param  protocol : Set Protocol for HID : boot/report protocol
 * @retval USBH_Status : Response for USB Set Protocol request
 */
static USBH_Status USBH_Set_Protocol(USB_OTG_CORE_HANDLE *pdev,
                                     USBH_HOST *phost,
                                     uint8_t protocol)
{

  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE |
                                         USB_REQ_TYPE_CLASS;

  phost->Control.setup.b.bRequest = USB_HID_SET_PROTOCOL;

  if (protocol != 0)
  {
    /* Boot Protocol */
    phost->Control.setup.b.wValue.w = 0;
  }
  else
  {
    /*Report Protocol*/
    phost->Control.setup.b.wValue.w = 1;
  }

  phost->Control.setup.b.wIndex.w = 0;
  phost->Control.setup.b.wLength.w = 0;

  return USBH_CtlReq(pdev, phost, 0, 0);
}

/**
 * @brief  USBH_ParseHIDDesc
 *         This function Parse the HID descriptor
 * @param  buf: Buffer where the source descriptor is available
 * @retval None
 */
static void USBH_ParseHIDDesc(USBH_HIDDesc_TypeDef *desc, uint8_t *buf)
{

  desc->bLength = *(uint8_t *)(buf + 0);
  desc->bDescriptorType = *(uint8_t *)(buf + 1);
  desc->bcdHID = LE16(buf + 2);
  desc->bCountryCode = *(uint8_t *)(buf + 4);
  desc->bNumDescriptors = *(uint8_t *)(buf + 5);
  desc->bReportDescriptorType = *(uint8_t *)(buf + 6);
  desc->wItemLength = LE16(buf + 7);
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
