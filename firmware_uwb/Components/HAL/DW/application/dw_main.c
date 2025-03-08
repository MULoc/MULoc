///*! ----------------------------------------------------------------------------
// *  @file    main.c
// *  @brief   main loop for the DecaRanging application
// *
// * @attention
// *
// * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
// *
// * All rights reserved.
// *
// * @author DecaWave
// */
///* Includes */
//#include "compiler.h"
//#include "port.h"

////#include "instance.h"

//#include "deca_types.h"
//#include "deca_regs.h"

//#include "deca_spi.h"
//#include "dw_main.h"
//#include "dwm1000_timestamp.h"

//#include "hal_led.h"
//#include "hal_usb.h"
//#include "hal_spi.h"
//#include "hal_usart.h"

//#include "bphero_uwb.h"

//extern void usb_run(void);
//extern int usb_init(void);

///**
//**===========================================================================
//**
//**  Abstract: main program
//**
//**===========================================================================
//*/

//static int ret;

//static uint32 status_reg = 0;

//static uint8 distance_seqnum;
//static srd_msg_dsss *msg_f_recv ;

////static uint16 RX_ANT_DLY;
////#static uint16 TX_ANT_DLY;

//#define RX_ANT_DLY 0
//#define TX_ANT_DLY 32880

//#define CIR_LENGTH 3
//#define LCD_BUFF_LEN (500)

//static dwt_rxdiag_t rx_diag1;
//static uint8 usbVCOMout[LCD_BUFF_LEN * 8];

//static uint8 cir_buffer1[4 * CIR_LENGTH+1];
//static uint8 cir_buffer2[4 * CIR_LENGTH+1];
//static uint8 cir_buffer3[4 * CIR_LENGTH+1];
//static uint8 cir_buffer4[4 * CIR_LENGTH+1];

//extern dwt_config_t config;
//extern dwt_config_t config2;
//extern dwt_config_t config3;

//extern dwt_txconfig_t txconfig1;
//extern dwt_txconfig_t txconfig2;
//extern dwt_txconfig_t txconfig3;

//extern uint16 rfDelaysTREK[2];

//struct cir_tap_struct
//{
//		uint16 real;
//		uint16 imag;
//};

//int App_Module_Uart_USB_Send(uint8_t *buf, uint16_t len)
//{
//		int ret = 0;
//		char send_buf[2000];
//		memset(send_buf,0, sizeof(send_buf));
//		memcpy(send_buf, buf, len);

//		USART1_SendBuffer(send_buf, len, TRUE);
//		//HalUsbWrite(send_buf, len);
//		return ret;
//}

////#pragma GCC optimize ("O3")
//#ifdef TX_NODE

//static uint64 poll_tx_ts;
//static uint64 cur_ts;
//static uint64 resp_rx_ts;
//static uint64 final_tx_ts;

//static uint32 final_tx_time;
//static uint32 final2_tx_time;

//static int Final_Distance = 0;

//static uint8 board_num = 3;

//int dw_main(void)
//{
//	
//		//RX_ANT_DLY = rfDelaysTREK[1];
//	  //TX_ANT_DLY = rfDelaysTREK[1];
//	
//	
//	// 修改为我们自己的代码
//		reset_DW1000();
//	
//		SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);
//	
//		// 初始化DW1000
//		if(dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
//		{
//			while(1){};
//		}
//		
//		dwt_xtaltrim(16);
//		// 将SPI调整至18MHz
//		SPI_ConfigFastRate(SPI_BaudRatePrescaler_4);
//		// 配置工作频率
//		
////		config.txCode = 10;
////		config.rxCode = 10;
//		dwt_configure(&config);
//		// 启动DW1000状态指示灯
//		dwt_setleds(1);
//		// 配置发送功率
//		dwt_configuretxrf(&txconfig2);
//		// 设置工作网络ID
//		dwt_setpanid(NET_PANID);
//		// 设置自身短地址
//		dwt_setaddress16(1);
//		// 配置天线延迟
//		dwt_setrxaftertxdelay(RX_ANT_DLY);
//		dwt_settxantennadelay(TX_ANT_DLY);
//		
//		// 设置DW1000中断，虽然并没有用到
//		dwt_setinterrupt(DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
//		
//		BPhero_UWB_Message_Init();
//		
//		uint8 token = 0;
//		
//		msg_f_send.sourceAddr[0] = 1 & 0xFF; //copy the address
//    msg_f_send.sourceAddr[1] =(1>>8)& 0xFF; //copy the address
//		
//		while(1)
//		{
//				msg_f_send.destAddr[0] = (2+token) & 0xFF;
//				msg_f_send.destAddr[1] = ((2+token)>>8) & 0xFF;
//			
//				msg_f_send.seqNum = distance_seqnum;
//				msg_f_send.messageData[0] = 'P'; // Send Poll Message
//				
//				dwt_writetxdata(psduLength+1, (uint8 *)&msg_f_send, 0);
//			
//				dwt_writetxfctrl(psduLength+1,0);
//				// 设置发送到接收开启的时间
//				dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
//				// 设置超时时间
//				dwt_setrxtimeout(300);
//				// 设置Preamble超时时间
//				dwt_setpreambledetecttimeout(0);
//				// 启动立即发送
//				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
//			
//				//等待接收完成
//				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
//				{};
//				
//				if (status_reg & SYS_STATUS_RXFCG)
//				{
//						frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
//					
//						//cur_ts = get_cur_timestamp_u64();
//					
//						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
//						
//						if (frame_len <= FRAME_LEN_MAX)
//						{
//								// 读取传送来的数据
//								dwt_readrxdata(rx_buffer, frame_len, 0);
//								// 将传送来的数据转为消息格式
//								msg_f_recv = (srd_msg_dsss*)rx_buffer;
//								
//						}
//						
//						if ('A' == msg_f_recv->messageData[0])
//						{
//							
//								// 读取Poll发送时间和Resp接收时间
//								poll_tx_ts = get_tx_timestamp_u64();
//								resp_rx_ts = get_rx_timestamp_u64();
//							
//								// 设置Final 延迟发送时间
//								final_tx_time = (resp_rx_ts + ((RESP_RX_TO_FINAL_TX_DLY_UUS) * UUS_TO_DWT_TIME)) >> 8;
//							
//								dwt_setdelayedtrxtime(final_tx_time);
//							
//								final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
//							
//								// 设置Final 发送数据
//								msg_f_send.messageData[0]='F';//Final message
//								final_msg_set_ts(&msg_f_send.messageData[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
//								final_msg_set_ts(&msg_f_send.messageData[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
//								final_msg_set_ts(&msg_f_send.messageData[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
//								final_msg_set_ts(&msg_f_send.messageData[FINAL_MSG_FINAL_TX_TS_IDX+4], final_tx_ts);
//							
//								dwt_writetxdata(9+17, (uint8 *)&msg_f_send, 0) ; // write the frame data
//								dwt_writetxfctrl(9+17, 0);
//								// 延迟发送
//							
//								//cur_ts = get_cur_timestamp_u64();

//								ret = dwt_starttx(DWT_START_TX_DELAYED);
//								
//								//dwt_readdiagnostics(&rx_diag1);
//								//uint16 fp_int1 = rx_diag1.firstPath >> 6;
//								uint16 fp_int1 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET) >> 6;
//								dwt_readaccdata(cir_buffer1, CIR_LENGTH*4, (fp_int1)*4);
//								
//								if (DWT_SUCCESS == ret)
//								{
//									
//										msg_f_send.messageData[0]='X';//Final message
//														
//										for (int i = 0; i < CIR_LENGTH*4; i++)
//										{
//											// 需要使用FP的CIR
//											msg_f_send.messageData[FINAL_MSG_POLL_TX_TS_IDX+i] = cir_buffer1[i+5];
//										}
//										
//										/* Write and send final2 message. */
//										final2_tx_time = (resp_rx_ts + ((540*2) * UUS_TO_DWT_TIME)) >> 8;
//										
//										while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
//										{};
//											
//										dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
//											
//										dwt_setdelayedtrxtime(final2_tx_time);
//										
//										dwt_writetxdata(psduLength + 16, (uint8 *)&msg_f_send, 0) ; // write the frame data
//										dwt_writetxfctrl(psduLength +16, 0);

//										ret = dwt_starttx(DWT_START_TX_DELAYED);
//										
//										while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
//										{ };

//										/* Clear TX frame sent event. */
//										dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
//										
//										Final_Distance = (msg_f_recv->messageData[1]*100 + msg_f_recv->messageData[2]);//cm
//										
//										float uwb_rssi = 0;
//										//uwb_rssi = dwGetReceivePower();
//										
////										if (msg_f_send.seqNum % 3 == 2)
////										{
////												token = (token + 1) % board_num;
////										}
//										
//										token = (token + 1) % board_num;
//										
////										if (msg_f_send.seqNum % 3 == 2)
////										{
////												//HalDelay_nMs();
////										}
//										
//										//Sequence Num自增
//										if(distance_seqnum == 254)
//										{
//											distance_seqnum = 0;
//										}
//										else
//										{
//											distance_seqnum++;
//										}
//										//++distance_seqnum;
//										
//								}
//								else
//								{
//										//OLED_ShowString(0,0,"Final Fail");
//								}
//								
//						}
//	
//				}
//				else
//				{
//					
//						//OLED_ShowString(0,1,"Resp Fail");
//						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
//				}
//				
//		}
//		
//    return 0;
//}

//#else

//#ifdef RX_NODE

//static srd_msg_dsss *msg_f;
//static double tof;
//static double distance;


////定义保存时间戳
//static uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
//static uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;

//static uint64 poll_rx_ts;
//static uint64 resp_tx_ts;
//static uint64 final_rx_ts;
//static uint64 final2_rx_ts;


//static double Ra, Rb, Da, Db;
//static int64 tof_dtu;
//static int temp  = 0;//保存临时变量
//static float uwb_rssi = 0;//定义保存RSSI信号强度的变量
//	
//static int n = 0;

//static uint8 debug = 0;

//static uint8 uCurrentTrim_val = 19;

//static uint16 addr = 2;

//int dw_main(void)
//{

//		//RX_ANT_DLY = rfDelaysTREK[1];
//	  //TX_ANT_DLY = rfDelaysTREK[1];
//	
//		// 修改为我们自己的代码
//		reset_DW1000();
//	
//		SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);
//	
//		// 初始化DW1000
//		if(dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
//		{
//			while(1){};
//		}
//		
//		// 读取晶振校准参数
//		// dwt_xtaltrim(uCurrentTrim_val);

//		// dwt_readfromdevice(FS_CTRL_ID,FS_XTALT_OFFSET,1,&uCurrentTrim_val);
//		// uCurrentTrim_val &= 31;
//		
//		// 将SPI调整至18MHz
//		SPI_ConfigFastRate(SPI_BaudRatePrescaler_4);
//		
//		// 配置工作频率
//		// config.txCode = 7 + addr;
//		// config.rxCode = 7 + addr;		
//		dwt_configure(&config);
//		// 启动DW1000状态指示灯
//		dwt_setleds(1);
//		// 配置发送功率
//		dwt_configuretxrf(&txconfig2);
//		// 设置工作网络ID
//		dwt_setpanid(NET_PANID);
//		// 设置自身短地址
//		dwt_setaddress16(addr);
//		// 配置天线延迟
//		dwt_setrxaftertxdelay(RX_ANT_DLY);
//		dwt_settxantennadelay(TX_ANT_DLY);
//		
//		// 设置DW1000中断，虽然并没有用到
//		dwt_setinterrupt(DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
//		
//		BPhero_UWB_Message_Init();	
//		
//		uint8 err_cnt = 0;
//		
//		msg_f_send.sourceAddr[0] = (2+addr) & 0xFF; //copy the address
//    msg_f_send.sourceAddr[1] =((2+addr)>>8)& 0xFF; //copy the address
//		
//		uint16 counter = 0;
//		
//		while(1)
//		{
//		
//			//启动接收
//			//Step1:启动帧过滤功能 --> 只接收数据包，更多帧过滤功能相关内容可以参考51uwb.cn
//			//dwt_enableframefilter(DWT_FF_DATA_EN);
//			//Step2:设定接收延时，timeout参数为0表示一直处于接收状态
//			dwt_setrxtimeout(1000);
//			dwt_setpreambledetecttimeout(0);
//			//Step3:立刻启动接收，这个函数里的参数可以设置延时接收，可以优化，让接收机过段时间启动，减少能量损耗
//			dwt_rxenable(DWT_START_TX_IMMEDIATE);
//			
//			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
//			{ };
//			
//			//判断是否有完整数据接收完毕
//			if (status_reg & SYS_STATUS_RXFCG)
//			{
//					if (err_cnt > 0)
//					{
//							err_cnt=0;
//					}
//					//读取接收到的数据长度
//					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
//				
//					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
//				
//					//如果数据长度小于最大值，认为是合理的
//					if (frame_len <= FRAME_LEN_MAX)
//					{

//							//读取接收到的数据
//							dwt_readrxdata(rx_buffer, frame_len, 0);
//							//将数据强制转换成约定格式
//							msg_f = (srd_msg_dsss*)rx_buffer;
//						
//							if ((msg_f->destAddr[0] != msg_f_send.sourceAddr[0]) || (msg_f->destAddr[1] != msg_f_send.sourceAddr[1]))
//							{
//									continue;
//							}
//							
//							//提取发送该数据的短地址，并赋值到将要发送信息的目标地址
//							//哪里来的信息，后面回复给谁
//							msg_f_send.destAddr[0] = msg_f->sourceAddr[0];
//							msg_f_send.destAddr[1] = msg_f->sourceAddr[1];
//							//提取sequence Num
//							msg_f_send.seqNum = msg_f->seqNum;
//						
//							uint16 fp_int1 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET) >> 6;
//							
////							if (fp_int1 > 757 || fp_int1 < 730)
////							{
//////								uint16 firstPathAmp2 = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x2);
//////								n += sprintf((char *)&usbVCOMout[n], "f, %0x, %d, %d\r\n", msg_f_send.seqNum, fp_int1, firstPathAmp2);

//////								HalUsbWrite(usbVCOMout, n);
//////								n = 0;
////								n = 0;
////								n += sprintf((char *)&usbVCOMout[n], "error\r");	
////								HalUsbWrite(usbVCOMout, n);
////																					
////								n = 0;
////								// continue;
////							}

//							if ('P' == msg_f->messageData[0])
//							{
//								
//									uint32 resp_tx_time;
//									int ret;

//									//保存接收到这个信息的时间戳
//									poll_rx_ts = get_rx_timestamp_u64();
//								
//									resp_tx_time = (poll_rx_ts + ((POLL_RX_TO_RESP_TX_DLY_UUS) * UUS_TO_DWT_TIME)) >> 8;
//									dwt_setdelayedtrxtime(resp_tx_time);
//							
//									dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
//									dwt_setrxtimeout(500);
//							

//									msg_f_send.messageData[0]='A';//Poll ack message
//									//将上次测距信息打包发送
//									temp = (int)(distance*100);//convert m to cm
//									msg_f_send.messageData[1]=temp/100;
//									msg_f_send.messageData[2]=temp%100;
//									//将要发送的数据写入到UWB寄存器内
//									dwt_writetxdata(psduLength +3 , (uint8 *)&msg_f_send, 0) ; // write the frame data
//									//告知UWB发送数据偏移为0
//									dwt_writetxfctrl(psduLength +3, 0);
//								
//									//启动立即发送
//									ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
//									//等待发送完成
//																						
//									//uint16 fp_int1 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET) >> 6;
//									dwt_readaccdata(cir_buffer1, CIR_LENGTH*4, (fp_int1)*4);
//									
//									//MUST WAIT!!!!!
//									while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
//									{ };
//									
//									if (status_reg & SYS_STATUS_RXFCG)
//									{
//										
//											dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

//											frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
//										
//											// 60us
//											if (frame_len <= FRAME_LEN_MAX)
//											{
//										
//												//读取接收到的数据
//												dwt_readrxdata(rx_buffer, frame_len, 0);
//												//将数据强制转换成约定格式
//												msg_f = (srd_msg_dsss*)rx_buffer;
//												
//												//提取发送该数据的短地址，并赋值到将要发送信息的目标地址
//												//哪里来的信息，后面回复给谁
//												msg_f_send.destAddr[0] = msg_f->sourceAddr[0];
//												msg_f_send.destAddr[1] = msg_f->sourceAddr[1];
//												//提取sequence Num
//												// 80us
//												msg_f_send.seqNum = msg_f->seqNum;
//											
//										
//												if ('F' == msg_f->messageData[0])
//												{
//														//printf("Receive Final\r");
//														//保存发送A信息的时间戳
//														resp_tx_ts = get_tx_timestamp_u64();
//														//保存接收‘F'信息的时间戳
//														final_rx_ts = get_rx_timestamp_u64();
//													
//														//提取数据包载荷中时间戳信息
//														// 50us
//														final_msg_get_ts(&msg_f->messageData[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
//														final_msg_get_ts(&msg_f->messageData[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
//														final_msg_get_ts(&msg_f->messageData[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
//														
//														//dwt_readdiagnostics(&rx_diag2);
//														uint16 fp_int2 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET) >> 6;
//														//uint16 fp_int2 = rx_diag2.firstPath >> 6;
//														dwt_readaccdata(cir_buffer2, CIR_LENGTH*4, (fp_int2)*4);
//														//dwt_readaccdata(cir_buffer2, 4 * CIR_LENGTH, (fp_int2-6) * 4);
//												
//														uint32_t final2_rx_enable = (final_rx_ts + (430 * UUS_TO_DWT_TIME)) >> 8;

//														dwt_setdelayedtrxtime(final2_rx_enable);
//														dwt_setrxtimeout(500);

//														dwt_rxenable(1);
//														
//														/* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
//														while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
//														{};
//																												
//														if (status_reg & SYS_STATUS_RXFCG)
//														{
//															
//																final2_rx_ts = get_rx_timestamp_u64();
//																frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
//					
//																dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
//																											
//																for (uint32 i = 0 ; i < FRAME_LEN_MAX; i++ )
//																{
//																		rx_buffer[i] = '\0';
//																}
//																
//															
//																if (frame_len <= FRAME_LEN_MAX)
//																{
//																		//读取接收到的数据
//																		dwt_readrxdata(rx_buffer, frame_len, 0);
//																	
//																		//将数据强制转换成约定格式
//																		msg_f = (srd_msg_dsss*)rx_buffer;
//																		//提取发送该数据的短地址，并赋值到将要发送信息的目标地址
//																		//哪里来的信息，后面回复给谁
//																		msg_f_send.destAddr[0] = msg_f->sourceAddr[0];
//																		msg_f_send.destAddr[1] = msg_f->sourceAddr[1];
//																		//提取sequence Num
//																		msg_f_send.seqNum = msg_f->seqNum;
//																	
//																		if ('X' == msg_f->messageData[0])
//																		{
//																			
//																				for (int i = 0; i<CIR_LENGTH*4; i++)
//																				{
//																					cir_buffer3[i] = msg_f->messageData[FINAL_MSG_POLL_TX_TS_IDX+i];
//																				}
//												
//																				//dwt_readdiagnostics(&rx_diag4);
//																				//uint16 fp_int4 = rx_diag4.firstPath >> 6;
//																				uint16 fp_int4 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);
//																				dwt_readaccdata(cir_buffer4, CIR_LENGTH*4, ((fp_int4>>6))*4);
//																				
//																				//根据TWR算法计算距离，这个部分可以参考51uwb.cn的视频讲解说明
//																				poll_rx_ts_32 = (uint32)poll_rx_ts;
//																				resp_tx_ts_32 = (uint32)resp_tx_ts;
//																				final_rx_ts_32 = (uint32)final_rx_ts;
//																				Ra = (double)(resp_rx_ts - poll_tx_ts);
//																				Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
//																				Da = (double)(final_tx_ts - resp_rx_ts);
//																				Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
//																				tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

//																				tof = tof_dtu * DWT_TIME_UNITS;
//																				distance = tof * SPEED_OF_LIGHT;
//																				//官方给出偏移校正，用户可以适当根据环境调整偏移表格数据
//																				//distance = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//距离减去矫正系数
//																				//对计算的距离进行卡尔曼滤波
//																				//kalman filter
//																				//distance = KalMan(distance);
//																				
//																				int32 ci;
//																				float clockOffsetHertz;
//																				float clockOffsetPPM;
//																				
//																				ci = dwt_readcarrierintegrator();
//																				
//																				clockOffsetHertz = ci * FREQ_OFFSET_MULTIPLIER ;
//																				
//																				if (msg_f_send.seqNum % 3 == 0)
//																				{
//																					clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 ;
//																				}
//																				else if (msg_f_send.seqNum % 3 == 1)
//																				{
//																					clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_3 ;
//																				}
//																				else if (msg_f_send.seqNum % 3 == 2)
//																				{
//																					clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 ;
//																				}
//																				
////																				if (clockOffsetPPM > 1.3f)
////																				{
////																						if (uCurrentTrim_val >= 1 && uCurrentTrim_val <= 31)
////																						{
////																							
////																							SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);
////																							
////																							uCurrentTrim_val -= 1;
////																							dwt_xtaltrim(uCurrentTrim_val);
////																							SPI_ConfigFastRate(SPI_BaudRatePrescaler_4);
////																						
////																						}
////																				}
////																				else if(clockOffsetPPM < -1.3f)
////																				{
////																						if (uCurrentTrim_val >= 1 && uCurrentTrim_val <= 31)
////																						{
////																							
////																							SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);
////																							
////																							uCurrentTrim_val += 1;
////																							dwt_xtaltrim(uCurrentTrim_val);
////																							SPI_ConfigFastRate(SPI_BaudRatePrescaler_4);
////																						
////																						}
////																				}
//												
//																				//将距离信息发送到串口
//																				//printf("0x%04X <--> 0x%02X%02X :%d cm\r\n",SHORT_ADDR,msg_f_send.destAddr[1],msg_f_send.destAddr[0],(int)(100*distance));
//																				temp = (int)(distance*100);

//																				uwb_rssi = dwGetReceivePower();
//												
//																				struct cir_tap_struct *cir = (struct cir_tap_struct *)&cir_buffer1[1];
//																				
//																				for (int j = 1; j < 2; j++)
//																				{
//																						n += sprintf((char *)&usbVCOMout[n], "%04x,%04x,", cir[j].real, cir[j].imag);
//																				}
//																				
//																				n += sprintf((char *)&usbVCOMout[n], "%04x,%04x,%02x,%d\r", 0, 1, msg_f_send.seqNum, fp_int1);
//																				
//																				cir = (struct cir_tap_struct *)&cir_buffer2[1];;
//																				
//																				for (int j = 1; j < 2; j++)
//																				{
//																						n += sprintf((char *)&usbVCOMout[n], "%04x,%04x,", cir[j].real, cir[j].imag);
//																				}
//																				
//																				n += sprintf((char *)&usbVCOMout[n], "%04x,%04x,%02x,%f\r", 0, 2, msg_f_send.seqNum, clockOffsetPPM);										
//																				
//																				cir = (struct cir_tap_struct *)cir_buffer3;
//																				
//																				for (int j = 0; j < 1; j++)
//																				{
//																						n += sprintf((char *)&usbVCOMout[n], "%04x,%04x,", cir[j].real, cir[j].imag);
//																				}
//																				
//																				n += sprintf((char *)&usbVCOMout[n], "%04d,%04x,%02x,%6.5f\r", uCurrentTrim_val, 3, msg_f_send.seqNum, uwb_rssi);	
//																				
//																				
//																				cir = (struct cir_tap_struct *)&cir_buffer4[1];;
//																				
//																				for (int j = 1; j < 2; j++)
//																				{
//																						n += sprintf((char *)&usbVCOMout[n], "%04x,%04x,", cir[j].real, cir[j].imag);
//																				}
//																				
//																				n += sprintf((char *)&usbVCOMout[n], "%04x,%04x,%02x,%6.5f\r~", fp_int4, 4, msg_f_send.seqNum, distance);	

//																				//App_Module_Uart_USB_Send(usbVCOMout, n);
//																				
//																				//printf("%.*s", n, usbVCOMout);
//																				HalUsbWrite(usbVCOMout, n);
//																			
//																				n = 0;
//																				
//																		}
//																}
//														}
//														else
//														{
//																/* Clear RX error events in the DW1000 status register. */
//																dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

//														}
//														// freq_hooping(msg_f_send.seqNum, addr-2);
//														// freq_hooping(msg_f_send.seqNum, 0);
//												//if (msg_f_send.seqNum % 3 == 2)
//													//dwt_forcetrxoff();	
//													//HalDelay_nMs(7);
//												}
//											}
//									}
//									else
//									{
//											/* Clear RX error events in the DW1000 status register. */
//											dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

//									}
//							}
//					}
//			}
//			else
//			{
//					/* Clear RX error events in the DW1000 status register. */
//					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
//				
////					if(msg_f_send.seqNum % 3 != 2){
////						
////						err_cnt++;
////						if (err_cnt >= 2)
////						{
////							
////							freq_hooping(msg_f_send.seqNum,addr-2);
////							err_cnt = 0;
////							
////							n += sprintf((char *)&usbVCOMout[n], "change, %d\r\n", msg_f_send.seqNum);	

////							HalUsbWrite(usbVCOMout, n);
////							n = 0;
////						}
////					}
//			}
//		}


//		return 0;
//}

//#endif
//#endif
