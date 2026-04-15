#include "y_global.h"

u8 cmd_return[CMD_RETURN_SIZE];
eeprom_info_t eeprom_info;
u8 AI_mode = 255;
u8 group_do_ok = 1;
u8 forbid_turn = 0;
int do_start_index;  // ������ִ�� ��ʼ���
int do_time;         // ������ִ�� ִ�д���
int group_num_start; // ������ִ�� ��ʼ���
int group_num_end;   // ������ִ�� ��ֹ���
int group_num_times; // ������ִ�� ��ʼ����

u8 needSaveFlag = 0;         // ƫ����־
u32 bias_systick_ms_bak = 0; // ƫ����־ʱ��
u32 action_time = 0;

u8 uart_receive_buf[1024];
uint16_t uart1_get_ok;
u8 uart1_mode;

uint8_t uart_receive_num = 0; /* ��¼���ĸ��ӿڽ��յ�������,����ָ���ʱ�򲻻��ڷ��� */


/* ��Ƭ��������λ */
void soft_reset(void)
{
    printf("stm32 reset\r\n");
    // �ر������ж�
    __set_FAULTMASK(1);
    // ��λ
    NVIC_SystemReset();
}

// ��ʼ������
void others_init(void)
{
    uint8_t i = 0;

    w25x_read((u8 *)(&eeprom_info), W25Q64_INFO_ADDR_SAVE_STR, sizeof(eeprom_info)); // ��ȡȫ�ֱ���

    if (eeprom_info.version != VERSION) // �жϰ汾�Ƿ��ǵ�ǰ�汾
    {
        eeprom_info.version = VERSION; // ���Ƶ�ǰ�汾
        eeprom_info.dj_record_num = 0; // ѧϰ�����������ֵ0

        eeprom_info.ps2_cmd_size = 0;
    }

    if (eeprom_info.dj_bias_pwm[DJ_NUM] != FLAG_VERIFY)
    {
        for (i = 0; i < DJ_NUM; i++)
        {
            eeprom_info.dj_bias_pwm[i] = 0;
        }
        eeprom_info.dj_bias_pwm[DJ_NUM] = FLAG_VERIFY;
    }
    // printf("\r\npre_cmd = %u  FLAG_VERIFY=37\r\n", eeprom_info.pre_cmd[PRE_CMD_SIZE]);

	for (i = 0; i < DJ_NUM; i++)
    {
        duoji_doing[i].aim = 1500+eeprom_info.dj_bias_pwm[i];
        duoji_doing[i].cur = 1500+eeprom_info.dj_bias_pwm[i];
        duoji_doing[i].inc = 0;
        duoji_doing[i].time = 5000;
    }
	
    // ִ��Ԥ������ {G0000#000P1500T1000!#000P1500T1000!}
    if (eeprom_info.pre_cmd[PRE_CMD_SIZE] == FLAG_VERIFY)
    {
        if (eeprom_info.pre_cmd[0] == '$')
        {
            parse_cmd(eeprom_info.pre_cmd);
        }
    }
}

void zx_uart_send_str(u8 *str)
{
    uart1_get_ok = 1;
    if (uart_receive_num != 1)
        uart1_send_str(str);
    else
        uart_receive_num = 0;

    if (uart_receive_num != 3)
        uart3_send_str(str);
    else
        uart_receive_num = 0;
    uart1_get_ok = 0;
}

void uart6_report_tel(void)
{
    static uint8_t tel_buf[192];
    const int ix = (int)Vel.RT_IX;
    const int iy = (int)Vel.RT_IY;
    const int iw = (int)Vel.RT_IW;
    const int wa = (int)(Wheel_A.RT * 1000.0f);
    const int wb = (int)(Wheel_B.RT * 1000.0f);
    const int wc = (int)(Wheel_C.RT * 1000.0f);
    const int wd = (int)(Wheel_D.RT * 1000.0f);
    const int ax = (int)imu_acc_data[0];
    const int ay = (int)imu_acc_data[1];
    const int az = (int)imu_acc_data[2];
    const int gx = (int)imu_gyro_data[0];
    const int gy = (int)imu_gyro_data[1];
    const int gz = (int)imu_gyro_data[2];

    sprintf(
        (char *)tel_buf,
        "$TEL:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d!",
        ix, iy, iw, wa, wb, wc, wd, ax, ay, az, gx, gy, gz);
    uart6_send_str(tel_buf);
}

uint16_t str_contain_str(unsigned char *str, unsigned char *str2)
{
    unsigned char *str_temp, *str_temp2;
    str_temp = str;
    str_temp2 = str2;
    while (*str_temp)
    {
        if (*str_temp == *str_temp2)
        {
            while (*str_temp2)
            {
                if (*str_temp++ != *str_temp2++)
                {
                    str_temp = str_temp - (str_temp2 - str2) + 1;
                    str_temp2 = str2;
                    break;
                }
            }
            if (!*str_temp2)
            {
                return (str_temp - str);
            }
        }
        else
        {
            str_temp++;
        }
    }
    return 0;
}

void selection_sort(int *a, int len)
{
    int i, j, mi, t;
    for (i = 0; i < len - 1; i++)
    {
        mi = i;
        for (j = i + 1; j < len; j++)
        {
            if (a[mi] > a[j])
            {
                mi = j;
            }
        }

        if (mi != i)
        {
            t = a[mi];
            a[mi] = a[i];
            a[i] = t;
        }
    }
}

// int�� ȡ����ֵ����
int abs_int(int int1)
{
    if (int1 > 0)
        return int1;
    return (-int1);
}

float abs_float(float value)
{
    if (value > 0)
    {
        return value;
    }
    return (-value);
}

// �ַ����е��ַ�������� ��str�ַ��������е�ch1����ch2
void replace_char(u8 *str, u8 ch1, u8 ch2)
{
    while (*str)
    {
        if (*str == ch1)
        {
            *str = ch2;
        }
        str++;
    }
    return;
}

// ����int��������
void int_exchange(int *int1, int *int2)
{
    int int_temp;
    int_temp = *int1;
    *int1 = *int2;
    *int2 = int_temp;
}

// �����鱣�溯��
// ֻ����<>�������ַ��������ڴ˺����н��н���
void save_action(u8 *str)
{
    s32 action_index = 0;

    if (str[1] == '$' && str[2] == '!')
    {
        eeprom_info.pre_cmd[PRE_CMD_SIZE] = 0;
        rewrite_eeprom();
        uart1_send_str((u8 *)"@CLEAR PRE_CMD OK!");
        return;
    }
    else if (str[1] == '$')
    {
        memset(eeprom_info.pre_cmd, 0, sizeof(eeprom_info.pre_cmd));
        strcpy((char *)eeprom_info.pre_cmd, (char *)str + 1); // ���ַ������и���
        eeprom_info.pre_cmd[strlen((char *)str) - 2] = '\0';  // ��ֵ�ַ�0
        eeprom_info.pre_cmd[PRE_CMD_SIZE] = FLAG_VERIFY;
        rewrite_eeprom();
        zx_uart_send_str((u8 *)"@SET PRE_CMD OK!");
        zx_uart_send_str((u8 *)eeprom_info.pre_cmd); // ��ӡ�洢��ȥ��ָ��
        return;
    }

    // ��ȡ����������������ȷ�����ǵ�6���ַ�����#����Ϊ�ַ�������
    action_index = (str[2] - '0') * 1000 + (str[3] - '0') * 100 + (str[4] - '0') * 10 + (str[5] - '0');

    //<G0000#000P1500T1000!>
    if ((action_index == -1) || str[6] != '#')
    {
        uart1_send_str((u8 *)"E");
        return;
    }

    if ((action_index * ACTION_SIZE % 4096) == 0)
    {
        w25x_erase_sector(action_index * ACTION_SIZE / 4096);
    }
    // �Ѽ������滻�ɴ�����ֱ�Ӵ洢���洢оƬ����ȥ������ִ�ж������ʱ��ֱ���ó��������Ϳ�����
    replace_char(str, '<', '{');
    replace_char(str, '>', '}');

    w25x_write(str, action_index * ACTION_SIZE, strlen((char *)str) + 1);

    // memset(str, 0, sizeof((char *)str));
    // w25x_read(str, action_index * ACTION_SIZE, ACTION_SIZE);
    // uart1_send_str(str);

    // ����һ��A������λ�����Ѿ����յ���
    uart1_send_str((u8 *)"A");
    uart3_send_str((u8 *)"A");

    return;
}

// ���� #000P1500T1000! ���Ƶ��ַ���
void parse_action(u8 *uart_receive_buf)
{
    u16 index, time, i = 0,j=0;
    int bias, len;
    float pwm;
    zx_uart_send_str(uart_receive_buf);
	
	len = strlen((char *)uart_receive_buf); // ��ȡ���ڽ������ݵĳ���

    if (len>=12 && uart_receive_buf[0] == '#' && uart_receive_buf[4] == 'P' && uart_receive_buf[5] == 'S' && uart_receive_buf[6] == 'C' && uart_receive_buf[7] == 'K' && uart_receive_buf[12] == '!')
    {
        index = (uart_receive_buf[1] - '0') * 100 + (uart_receive_buf[2] - '0') * 10 + (uart_receive_buf[3] - '0');
        bias = (uart_receive_buf[9] - '0') * 100 + (uart_receive_buf[10] - '0') * 10 + (uart_receive_buf[11] - '0');
        if ((bias >= -500) && (bias <= 500) && (index < DJ_NUM))
        {
            if (uart_receive_buf[8] == '+')
            {
                duoji_doing[index].cur = duoji_doing[index].cur - eeprom_info.dj_bias_pwm[index] + bias;
                eeprom_info.dj_bias_pwm[index] = bias;
            }
            else if (uart_receive_buf[8] == '-')
            {
                duoji_doing[index].cur = duoji_doing[index].cur - eeprom_info.dj_bias_pwm[index] - bias;
                eeprom_info.dj_bias_pwm[index] = -bias;
            }
            duoji_doing[index].aim = duoji_doing[index].cur;
            duoji_doing[index].inc = 0.001;
            rewrite_eeprom();
        }
    }
    else if (len>=8 && uart_receive_buf[0] == '#' && uart_receive_buf[4] == 'P' && uart_receive_buf[5] == 'D' && uart_receive_buf[6] == 'S' && uart_receive_buf[7] == 'T' && uart_receive_buf[8] == '!')
    {
        index = (uart_receive_buf[1] - '0') * 100 + (uart_receive_buf[2] - '0') * 10 + (uart_receive_buf[3] - '0');
        if (index < DJ_NUM)
        {
            duoji_doing[index].inc = 0;
            duoji_doing[index].aim = duoji_doing[index].cur;
        }
        else if (index == 255)
        {
            for (index = 0; index < DJ_NUM; index++)
            {
                duoji_doing[index].inc = 0;
                duoji_doing[index].aim = duoji_doing[index].cur;
            }
        }
        return;
    }

    while ((len >= i) && uart_receive_buf[i])
    {
        if (uart_receive_buf[i] == '#')
        {
            index = 0;
            i++;
			for(j=0;j<3;j++)
			{
				if((len >= i) && uart_receive_buf[i]<='9' && uart_receive_buf[i]>='0')
				{	
					index = index * 10 + uart_receive_buf[i] - '0';
					i++;
				}
				else
				{
					goto err;
				}
			}
        }
        else if (uart_receive_buf[i] == 'P')
        {
            pwm = 0;
            i++;
			for(j=0;j<4;j++)
			{
				if((len >= i) && uart_receive_buf[i]<='9' && uart_receive_buf[i]>='0')
				{	
					pwm = pwm * 10 + uart_receive_buf[i] - '0';
					i++;
				}
				else
				{
					goto err;
				}
				
			}
        }
        else if (uart_receive_buf[i] == 'T')
        {
            time = 0;
            i++;
			for(j=0;j<4;j++)
			{
				if((len >= i) && uart_receive_buf[i]<='9' && uart_receive_buf[i]>='0')
				{	
					time = time * 10 + uart_receive_buf[i] - '0';
					i++;
				}
				else
				{
					goto err;
				}
			}

            duoji_doing_set(index, pwm, time);
			
        }
        else
        {
err:
            i++;
        }

    }
}

/*
    ���ж��ֹͣ���    $DST!
    ��x�����ֹͣ���   $DST:x!
    ��Ƭ���������$RST!
    ��鶯����x��y�����$CGP:x-y!
    ִ�е�x��������       $DGS:x!
    ִ�е�x��y�鶯��z�Σ� $DGT:x-y,z!
    С����x������y�ٶ�:   $DCR:x,y!
    ���ж����λ���    $DJR!
    ��ȡӦ���źţ�        $GETA!
    ��ȡ�����źţ�        $SMODE1!
    $KINEMATICS:x,y,z,time! //���굥λmm��ʱ�䵥λms

*/
// �����������
void parse_cmd(u8 *cmd)
{
    int pos, i, index, int1, int2;
	float kinematics_x=0,kinematics_y=0,kinematics_z=0;
	
	uart_receive_num = 0;
    uart1_send_str(cmd);

    if (pos = str_contain_str(cmd, (u8 *)"$DRS!"), pos)
    {
        uart1_send_str((u8 *)"hello word!");
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$DST!"), pos)
    {
        group_do_ok = 1;
        for (i = 0; i < DJ_NUM; i++)
        {
            duoji_doing[i].inc = 0;
            duoji_doing[i].aim = duoji_doing[i].cur;
        }
        zx_uart_send_str((u8 *)"#255PDST!"); // ����ֹͣ
        Vel.TG_IX = 0;
		Vel.TG_IY = 0;
		Vel.TG_IW = 0;
		AI_mode = 255;
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$DST:"), pos)
    {
        if (sscanf((char *)cmd, "$DST:%d!", &index))
        {
            duoji_doing[index].inc = 0;
            duoji_doing[index].aim = duoji_doing[index].cur;
            sprintf((char *)cmd_return, "#%03dPDST!", (int)index);
            zx_uart_send_str(cmd_return);
            memset(cmd_return, 0, sizeof(cmd_return));
        }
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$RST!"), pos)
    {
        soft_reset();
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$PTG:"), pos)
    {
        if (sscanf((char *)cmd, "$PTG:%d-%d!", &int1, &int2))
        {
            print_group(int1, int2);
        }
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$DGS:"), pos)
    {
        if (sscanf((char *)cmd, "$DGS:%d!", &int1))
        {
            group_do_ok = 1;
            do_group_once(int1);
        }
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$DGT:"), pos)
    {
        if (sscanf((char *)cmd, "$DGT:%d-%d,%d!", &group_num_start, &group_num_end, &group_num_times))
        {
            group_do_ok = 1;
            if (group_num_start != group_num_end)
            {
                do_start_index = group_num_start;
                do_time = group_num_times;
                group_do_ok = 0;
            }
            else
            {
                do_group_once(group_num_start);
            }
        }
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$DJR!"), pos)
    {
        zx_uart_send_str((u8 *)"#255P1500T2000!");
        AI_mode = 255;
        for (i = 0; i < DJ_NUM; i++)
        {
            duoji_doing[i].aim = 1500 + eeprom_info.dj_bias_pwm[i];
            duoji_doing[i].time = 2000;
            duoji_doing[i].inc = (duoji_doing[i].aim - duoji_doing[i].cur) / (duoji_doing[i].time / 20.000);
        }
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$GETA!"), pos)
    {
        uart1_send_str((u8 *)"AAA");
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$SMODE"), pos)
    {
        if (sscanf((char *)uart_receive_buf, "$SMODE%d!", &int1))
        {
            AI_mode = int1;
            if (int1 == 0)
                zx_uart_send_str((u8 *)"#010P1512T0000!");
            if (int1 == 2)
                zx_uart_send_str((u8 *)"#010P1511T0000!");

            // uart1_send_str((u8 *)"@OK!");
            beep_on_times(1, 100);
        }
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$CAR:"), pos)
    {
        /* С���˶����� */
        int x = 0, y = 0, z = 0;
        if (sscanf((char *)cmd, "$CAR:%d,%d,%d!", &x, &y, &z))
        {
            Vel.TG_IX = x;
            Vel.TG_IY = y;
            Vel.TG_IW = z;
        }
    }
	else if (pos = str_contain_str(cmd, (u8 *)"$ZNXJ!"), pos)
    {
		/* ����ѭ��ģʽ */
        AI_mode=1;
		    beep_on_times(1,100);
			  forbid_turn = 0;
    }
	else if (pos = str_contain_str(cmd, (u8 *)"$ZYBZ!"), pos)
    {
        beep_on_times(1, 100);
        // ���ɱ���
        AI_mode = 2;
        // ǰ��
        Vel.TG_IX = 500;
        Vel.TG_IY = 0;
        Vel.TG_IW = 0;
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$GSGN!"), pos)
    {
        beep_on_times(1, 100);
        // �������
        AI_mode = 3;
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$QJ!"), pos)
    {
        // ǰ��
        Vel.TG_IX = 300;
        Vel.TG_IY = 0;
        Vel.TG_IW = 0;
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$HT!"), pos)
    {
        // ����
        Vel.TG_IX = -300;
        Vel.TG_IY = 0;
        Vel.TG_IW = 0;
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$ZZ!"), pos)
    {
        // ��ת
        Vel.TG_IX = 0;
        Vel.TG_IY = 0;
        Vel.TG_IW = 1200;
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$YZ!"), pos)
    {
        // ��ת
        Vel.TG_IX = 0;
        Vel.TG_IY = 0;
        Vel.TG_IW = -1200;
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$ZPY!"), pos)
    {
        // ��ƽ��
        Vel.TG_IX = 0;
        Vel.TG_IY = 300;
        Vel.TG_IW = 0;
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$YPY!"), pos)
    {
        // ��ƽ��
        Vel.TG_IX = 0;
        Vel.TG_IY = -300;
        Vel.TG_IW = 0;
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$TZ!"), pos)
    {
        // ֹͣ
        Vel.TG_IX = 0;
        Vel.TG_IY = 0;
        Vel.TG_IW = 0;
        AI_mode = 255;
    }
	else if (pos = str_contain_str(cmd, (u8 *)"$KMS:"), pos)
    {
        if (sscanf((char *)cmd, "$KMS:%f,%f,%f,%d!", &kinematics_x, &kinematics_y, &kinematics_z, &int1))
        {
            //uart1_send_str((u8 *)"Try to find best pos:\r\n");
            if (kinematics_move(kinematics_x, kinematics_y, kinematics_z, int1))
            {
            }
            else
            {
                uart1_send_str((u8 *)"Can't find best pos!!!");
            }
        }
    }
	else if (pos = str_contain_str(cmd, (u8 *)"$oled_red!"), pos)
    {
		/* oled��ʾ��ɫ�� */
		OLED_CLS();
		OLED_ShowColorFont(64-16,2,0);
		OLED_ShowColorFont(64,2,4);
    }
	else if (pos = str_contain_str(cmd, (u8 *)"$oled_green!"), pos)
    {
		/* oled��ʾ��ɫ�� */
		OLED_CLS();
		OLED_ShowColorFont(64-16,2,1);
		OLED_ShowColorFont(64,2,4);
    }
	else if (pos = str_contain_str(cmd, (u8 *)"$oled_blue!"), pos)
    {
		/* oled��ʾ��ɫ�� */
		OLED_CLS();
		OLED_ShowColorFont(64-16,2,2);
		OLED_ShowColorFont(64,2,4);
    }
	else if (pos = str_contain_str(cmd, (u8 *)"$oled_yellow!"), pos)
    {
		/* oled��ʾ��ɫ�� */
		OLED_CLS();
		OLED_ShowColorFont(64-16,2,3);
		OLED_ShowColorFont(64,2,4);
    }
    else if (pos = str_contain_str(cmd, (u8 *)"$BEEP!"), pos)
    {
		beep_on_times(2, 100);
    }
}
// ִ�ж�����1��
// �����Ƕ��������
void do_group_once(int group_num)
{
    // ��uart_receive_buf����
    memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
    // �Ӵ洢оƬ�ж�ȡ��group_num��������
    w25x_read(uart_receive_buf, group_num * ACTION_SIZE, ACTION_SIZE);
    // ��ȡ������ʱ��
    action_time = getMaxTime(uart_receive_buf);

    // �Ѷ�ȡ�����Ķ����鴫�ݵ�parse_actionִ��
    parse_action(uart_receive_buf);
}

// ����������ִ��
void loop_action(void)
{
    // ͨ���ж϶���Ƿ�ȫ��ִ����� ������ִ�ж�����group_do_ok��δ����������½��봦��
    static long long systick_ms_bak = 0;
    if (group_do_ok == 0)
    {
        if (millis() - systick_ms_bak > action_time)
        {
            systick_ms_bak = millis();
            if (group_num_times != 0 && do_time == 0)
            {
                group_do_ok = 1;
                uart1_send_str((u8 *)"@GroupDone!");
                return;
            }
            // ����do_start_index������
            do_group_once(do_start_index);

            if (group_num_start < group_num_end)
            {
                if (do_start_index == group_num_end)
                {
                    do_start_index = group_num_start;
                    if (group_num_times != 0)
                    {
                        do_time--;
                    }
                    return;
                }
                do_start_index++;
            }
            else
            {
                if (do_start_index == group_num_end)
                {
                    do_start_index = group_num_start;
                    if (group_num_times != 0)
                    {
                        do_time--;
                    }
                    return;
                }
                do_start_index--;
            }
        }
    }
    else
    {
        action_time = 10;
    }
}

// ��ȡ���ʱ��
int getMaxTime(u8 *str)
{
    int i = 0, max_time = 0, tmp_time = 0;
    while (str[i])
    {
        if (str[i] == 'T')
        {
            tmp_time = (str[i + 1] - '0') * 1000 + (str[i + 2] - '0') * 100 + (str[i + 3] - '0') * 10 + (str[i + 4] - '0');
            if (tmp_time > max_time)
                max_time = tmp_time;
            i = i + 4;
            continue;
        }
        i++;
    }
    return max_time;
}

// ��ȡ���������ţ��ַ���������ŷ�����ţ����򷵻�-1
int get_action_index(u8 *str)
{
    u16 index = 0;
    // uart_send_str(str);
    while (*str)
    {
        if (*str == 'G')
        {
            str++;
            while ((*str != '#') && (*str != '$'))
            {
                index = index * 10 + *str - '0';
                str++;
            }
            return index;
        }
        else
        {
            str++;
        }
    }
    return -1;
}

// ��ӡ�洢��оƬ��Ķ����飬�Ӵ���1�з��ͳ��� $CGP:x-y!����������
void print_group(int start, int end)
{
    if (start > end)
    {
        int_exchange(&start, &end);
    }

    for (; start <= end; start++)
    {
        memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
        w25x_read(uart_receive_buf, start * ACTION_SIZE, ACTION_SIZE);
        uart1_send_str(uart_receive_buf);
        uart1_send_str((u8 *)"\r\n");
    }
}

// ��eeprom_infoд�뵽W25Q64_INFO_ADDR_SAVE_STRλ��
void rewrite_eeprom(void)
{
    w25x_erase_sector(W25Q64_INFO_ADDR_SAVE_STR / 4096);
    w25x_writeS((u8 *)(&eeprom_info), W25Q64_INFO_ADDR_SAVE_STR, sizeof(eeprom_info_t));
}

void set_servo(int index, int pwm, int time)
{
	uart_receive_num=0;
	if(index==3)
	{
		pwm = 3000-pwm;
	}
    duoji_doing[index].aim = pwm;
    duoji_doing[index].time = time;
    duoji_doing[index].inc = (duoji_doing[index].aim - duoji_doing[index].cur) / (duoji_doing[index].time / 20.000);
    sprintf((char *)cmd_return, "#%03dP%04dT%04d!", index, pwm, time);
    parse_action(cmd_return);
}

int kinematics_move(float x, float y, float z, int time)
{
    int i, min = 0, flag = 0;

    if (y < 0)
        return 0;

    // Ѱ����ѽǶ�
    flag = 0;
    for (i = 0; i >= -135; i--)
    {
        if (0 == kinematics_analysis(x, y, z, i, &kinematics))
        {
            if (i < min)
                min = i;
            flag = 1;
        }
    }

    // ��3�Ŷ����ˮƽ���ļн���Ϊ���ֵ
    if (flag)
    {
        kinematics_analysis(x, y, z, min, &kinematics);

		kinematics.servo_pwm[3] = 3000-kinematics.servo_pwm[3];
		sprintf((char *)cmd_return, "{#000P%04dT%04d!#001P%04dT%04d!#002P%04dT%04d!#003P%04dT%04d!}", kinematics.servo_pwm[0], time,
																									 kinematics.servo_pwm[1], time,
																									 kinematics.servo_pwm[2],time,
																									 kinematics.servo_pwm[3],time);
		parse_action(cmd_return);

        return 1;
    }

    return 0;
}
