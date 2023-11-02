/***************************************************************************** 
* 
* File Name : main.c
* 
* Description: main 
* 
* Copyright (c) 2014 Winner Micro Electronic Design Co., Ltd. 
* All rights reserved. 
* 
* Author : dave
* 
* Date : 2014-6-14
*****************************************************************************/ 
#include "wm_include.h"





void UserMain(void)
{
	printf("\n [d] W801 MQTT TEST - FreeRTOS - Built with Linux  By A.MJADI\n");

#if DEMO_CONSOLE
	//CreateDemoTask();
	CreateMqttTAsk();
	
#endif

}

