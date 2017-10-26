/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "ets_sys.h"
#include "osapi.h"
#include "ip_addr.h"
#include "espconn.h"
#include "mem.h"


#include "user_interface.h"
#include "smartconfig.h"
#include "airkiss.h"
#include "driver/uart.h"
#include "gpio.h"

#define QUEUE_SIZE		480
#define QUEUE_FULL		0
#define QUEUE_EMPTY		1
#define QUEUE_TURE		-1

#define PCM_IN_MUX		PERIPHS_IO_MUX_MTMS_U
#define PCM_IN_NUM		12
#define PCM_IN_FUNC		FUNC_GPIO14

#define PCM_OUT_MUX		PERIPHS_IO_MUX_MTDI_U
#define PCM_OUT_NUM		14
#define PCM_OUT_FUNC	FUNC_GPIO12

#define PCM_CLK_MUX		PERIPHS_IO_MUX_GPIO4_U
#define PCM_CLK_NUM		5
#define PCM_CLK_FUNC	FUNC_GPIO4

#define PCM_SYNC_MUX	PERIPHS_IO_MUX_GPIO5_U
#define PCM_SYNC_NUM	4
#define PCM_SYNC_FUNC	FUNC_GPIO5

uint8_t key_sta = 1;
uint8_t sync_sta = 0;
uint8_t pcm_out_sta = 0;
uint8_t fifo_pcmout = 0;
uint32_t clk_cnt = 0;
uint16_t pcm_writebyte = 0;
uint16_t pcm_readbyte = 0;
uint8_t pcmalreadyread = 0;

typedef struct QUEUE{
	int32_t head;
	int32_t tail;
	uint16_t data[QUEUE_SIZE];
	int32_t cnt	;
}fifo_queue;

fifo_queue pcm_queue;

#define DEVICE_TYPE 		"gh_9e2cff3dfa51" //wechat public number
#define DEVICE_ID 			"122475" //model ID

#define DEFAULT_LAN_PORT 	12476

LOCAL esp_udp ssdp_udp;
LOCAL struct espconn pssdpudpconn;
LOCAL os_timer_t ssdp_time_serv;

uint8_t  lan_buf[200];
uint16_t lan_buf_len;
uint8 	 udp_sent_cnt = 0;

const airkiss_config_t akconf =
{
	(airkiss_memset_fn)&memset,
	(airkiss_memcpy_fn)&memcpy,
	(airkiss_memcmp_fn)&memcmp,
	0,
};

LOCAL void ICACHE_FLASH_ATTR
airkiss_wifilan_time_callback(void)
{
	uint16 i;
	airkiss_lan_ret_t ret;

	if ((udp_sent_cnt++) >30) {
		udp_sent_cnt = 0;
		os_timer_disarm(&ssdp_time_serv);//s
		//return;
	}

	ssdp_udp.remote_port = DEFAULT_LAN_PORT;
	ssdp_udp.remote_ip[0] = 255;
	ssdp_udp.remote_ip[1] = 255;
	ssdp_udp.remote_ip[2] = 255;
	ssdp_udp.remote_ip[3] = 255;
	lan_buf_len = sizeof(lan_buf);
	ret = airkiss_lan_pack(AIRKISS_LAN_SSDP_NOTIFY_CMD,
		DEVICE_TYPE, DEVICE_ID, 0, 0, lan_buf, &lan_buf_len, &akconf);
	if (ret != AIRKISS_LAN_PAKE_READY) {
		os_printf("Pack lan packet error!");
		return;
	}

	ret = espconn_sendto(&pssdpudpconn, lan_buf, lan_buf_len);
	if (ret != 0) {
		os_printf("UDP send error!");
	}
	os_printf("Finish send notify!\n");
}

LOCAL void ICACHE_FLASH_ATTR
airkiss_wifilan_recv_callbk(void *arg, char *pdata, unsigned short len)
{
	uint16 i;
	remot_info* pcon_info = NULL;

	airkiss_lan_ret_t ret = airkiss_lan_recv(pdata, len, &akconf);
	airkiss_lan_ret_t packret;

	switch (ret){
	case AIRKISS_LAN_SSDP_REQ:
		espconn_get_connection_info(&pssdpudpconn, &pcon_info, 0);
		os_printf("remote ip: %d.%d.%d.%d \r\n",pcon_info->remote_ip[0],pcon_info->remote_ip[1],
			                                    pcon_info->remote_ip[2],pcon_info->remote_ip[3]);
		os_printf("remote port: %d \r\n",pcon_info->remote_port);

        pssdpudpconn.proto.udp->remote_port = pcon_info->remote_port;
		os_memcpy(pssdpudpconn.proto.udp->remote_ip,pcon_info->remote_ip,4);
		ssdp_udp.remote_port = DEFAULT_LAN_PORT;

		lan_buf_len = sizeof(lan_buf);
		packret = airkiss_lan_pack(AIRKISS_LAN_SSDP_RESP_CMD,
			DEVICE_TYPE, DEVICE_ID, 0, 0, lan_buf, &lan_buf_len, &akconf);

		if (packret != AIRKISS_LAN_PAKE_READY) {
			os_printf("Pack lan packet error!");
			return;
		}

		os_printf("\r\n\r\n");
		for (i=0; i<lan_buf_len; i++)
			os_printf("%c",lan_buf[i]);
		os_printf("\r\n\r\n");

		packret = espconn_sendto(&pssdpudpconn, lan_buf, lan_buf_len);
		if (packret != 0) {
			os_printf("LAN UDP Send err!");
		}

		break;
	default:
		os_printf("Pack is not ssdq req!%d\r\n",ret);
		break;
	}
}

void ICACHE_FLASH_ATTR
airkiss_start_discover(void)
{
	ssdp_udp.local_port = DEFAULT_LAN_PORT;
	pssdpudpconn.type = ESPCONN_UDP;
	pssdpudpconn.proto.udp = &(ssdp_udp);
	espconn_regist_recvcb(&pssdpudpconn, airkiss_wifilan_recv_callbk);
	espconn_create(&pssdpudpconn);

	os_timer_disarm(&ssdp_time_serv);
	os_timer_setfn(&ssdp_time_serv, (os_timer_func_t *)airkiss_wifilan_time_callback, NULL);
	os_timer_arm(&ssdp_time_serv, 1000, 1);//1s
}


void ICACHE_FLASH_ATTR
smartconfig_done(sc_status status, void *pdata)
{
    switch(status) {
        case SC_STATUS_WAIT:
            os_printf("SC_STATUS_WAIT\n");
            break;
        case SC_STATUS_FIND_CHANNEL:
            os_printf("SC_STATUS_FIND_CHANNEL\n");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            os_printf("SC_STATUS_GETTING_SSID_PSWD\n");
			sc_type *type = pdata;
            if (*type == SC_TYPE_ESPTOUCH) {
                os_printf("SC_TYPE:SC_TYPE_ESPTOUCH\n");
            } else {
                os_printf("SC_TYPE:SC_TYPE_AIRKISS\n");
            }
            break;
        case SC_STATUS_LINK:
            os_printf("SC_STATUS_LINK\n");
            struct station_config *sta_conf = pdata;

	        wifi_station_set_config(sta_conf);
	        wifi_station_disconnect();
	        wifi_station_connect();
            break;
        case SC_STATUS_LINK_OVER:
            os_printf("SC_STATUS_LINK_OVER\n");
            if (pdata != NULL) {
				//SC_TYPE_ESPTOUCH
                uint8 phone_ip[4] = {0};

                os_memcpy(phone_ip, (uint8*)pdata, 4);
                os_printf("Phone ip: %d.%d.%d.%d\n",phone_ip[0],phone_ip[1],phone_ip[2],phone_ip[3]);
            } else {
            	//SC_TYPE_AIRKISS - support airkiss v2.0
				airkiss_start_discover();
            }
            smartconfig_stop();
            break;
    }

}

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}


void ICACHE_FLASH_ATTR
user_rf_pre_init(void)
{
}

void ICACHE_FLASH_ATTR
queue_init(fifo_queue *queue)
{
	queue->head = queue->tail;
	queue->cnt = 0;
}

uint8_t ICACHE_FLASH_ATTR
queue_write(fifo_queue *queue, uint16_t data)
{
	if(queue->head == queue->tail && queue->cnt == QUEUE_SIZE)
	{
		return QUEUE_FULL;
	}
	else
	{
		queue->data[queue->tail] = data;
		queue->tail = (queue->tail+1)%QUEUE_SIZE;
		queue->cnt++;
		return QUEUE_TURE;
	}
}

uint8_t ICACHE_FLASH_ATTR
queue_read(fifo_queue queue, uint16_t data)
{
	if(queue->head == queue->tail && queue->cnt == 0)
	{
		return QUEUE_EMPTY;
	}
	else
	{
		data = queue->data[queue->head];
		queue->head = (queue->head+1)%QUEUE_SIZE;
		queue->cnt--;
		return QUEUE_TURE;
	}
}

void ICACHE_FLASH_ATTR
pcm_intr_handle(void *arg)
{
	uint32 gpio_status;
	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);
	if(gpio_status&BIT(PCM_SYNC_NUM))
	{
		sync_sta = 1;
	}
	else if((gpio_status&BIT(PCM_CLK_NUM))&&sync_sta)
	{
		pcm_out_sta = GPIO_INPUT_GET(GPIO_ID_PIN(PCM_IN_NUM));
		pcm_writebyte|=pcm_out_sta<<clk_cnt;	//存储在缓存pcm_byte中，
		if(clk_cnt == 15)
		{
			queue_write(&pcm_queue, pcm_writebyte);	//写入fifo队列
			pcm_writebyte = 0;
			if(pcm_queue.cnt == 320)
			{
				pcmalreadyread = 1;
			}
			clk_cnt = 0;
		}
		if(pcmalreadyread&&clk_cnt==0)	//写入320byte后，开始从队列读取并通过PCM_OUT输出给M26，只有当clk_cnt=0时，可以读取1个pcm_byte数据
		{
			queue_read(&pcm_queue, pcm_readbyte);
		}
		if(pcm_readbyte&(uint16_t)1<<clk_cnt)
		{
			GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1<<PCM_OUT_NUM);
		}
		else
		{
			GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1<<PCM_OUT_NUM);
		}
		clk_cnt++;
	}
}

void ICACHE_FLASH_ATTR
user_init(void)
{
	queue_init(&pcm_queue);
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_printf("\r\n");
	os_printf("Thank godsh, you come here!\r\n");
	os_printf("SDK version:%s\n", system_get_sdk_version());

	PIN_FUNC_SELECT(PCM_OUT_MUX, PCM_OUT_FUNC);
	GPIO_OUTPUT_SET(GPIO_ID_PIN(PCM_OUT_NUM), 0);

	PIN_FUNC_SELECT(PCM_IN_MUX, PCM_IN_FUNC);
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(PCM_IN_NUM));

	PIN_FUNC_SELECT(PCM_CLK_MUX, PCM_CLK_FUNC);
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(PCM_CLK_NUM));

	PIN_FUNC_SELECT(PCM_SYNC_MUX, PCM_SYNC_FUNC);
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(PCM_SYNC_NUM));

	ETS_GPIO_INTR_DISABLE();
	ETS_GPIO_INTR_ATTACH(pcm_intr_handle, NULL);
	gpio_pin_intr_state_set(GPIO_ID_PIN(PCM_CLK_NUM), GPIO_PIN_INTR_NEGEDGE);
	gpio_pin_intr_state_set(GPIO_ID_PIN(PCM_SYNC_NUM), GPIO_PIN_INTR_POSEDGE);
	ETS_GPIO_INTR_ENABLE();
}


