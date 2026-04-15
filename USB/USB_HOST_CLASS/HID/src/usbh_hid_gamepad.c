#include "usbh_hid_gamepad.h"

uint8_t ps2_buf[12];
uint8_t ps2_do_ok = 0;
static void GAMEPAD_Init(void);
static void GAMEPAD_Decode(u8 *data);

HID_cb_TypeDef HID_GAMEPAD_cb =
	{
		GAMEPAD_Init,
		GAMEPAD_Decode,
};

// game pad 初始化
static void GAMEPAD_Init(void)
{
	USR_GAMEPAD_Init();
}

// game pad数据解析
static void GAMEPAD_Decode(uint8_t *data)
{
	u8 i;

//	if (data[0] == 0X07) /* 第1，2个字节开始 */
//	{
		for (i = 0; i < 12; i++)
		{
//			 printf("%02x ", data[i]);
			ps2_buf[i] = data[i ];
		}
		ps2_do_ok = 1;
//		 printf("\r\n");
//	}
}
