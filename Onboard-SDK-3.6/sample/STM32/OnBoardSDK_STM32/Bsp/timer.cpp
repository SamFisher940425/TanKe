/*! @file timer.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Timer helper functions and ISR for board STM32F4Discovery
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
 
#include "stm32f4xx.h"
#include "timer.h"
#include "main.h"

uint32_t tick = 0; // tick is the time stamp,which record how many ms since u
                   // initialize the system.
/*extern VirtualRC virtualrc;
extern VirtualRCData myVRCdata;
extern FlightData flightData;
extern Flight flight;
*/
extern uint8_t         Rx_buff[];
extern TerminalCommand myTerminal;

extern Control::CtrlData Flight_Command;

extern Telemetry::GlobalPosition Flight_GlobalPosition;
extern Telemetry::Quaternion     Flight_Quaternion;

extern struct Desire_LLA GPS_Ctrl_Desire_LLA;

extern bool Pre_Stop_Telemetry_Flag;

extern bool Computer_Controling_Flag;

extern Vehicle*       v;

void
Timer1Config()
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Period =
    (200 - 1); // t is the time between each Timer irq.
  TIM_TimeBaseInitStructure.TIM_Prescaler =
    (8400 - 1); // t = (1+TIM_Prescaler/SystemCoreClock)*(1+TIM_Period)
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter =
    0x00; // here configure TIM1 in 50Hz
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_UP_TIM10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM1, DISABLE);
}
void
Timer2Config()
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Period =
    (40 - 1); // t is the time between each Timer irq.
  TIM_TimeBaseInitStructure.TIM_Prescaler =
    (42000 - 1); // t = (1+TIM_Prescaler/SystemCoreClock)*(1+TIM_Period)
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter =
    0x00; // here configure TIM2 in 100Hz
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, DISABLE);
}
void
SystickConfig()
{
  if (SysTick_Config(SystemCoreClock / 1000)) // 1000 ticks per second.
  {
    while (1)
      ; // run here when error.
  }
}

void
delay_nms(uint16_t time)
{
  u32 i = 0;
  while (time--)
  {
    i = 30000;
    while (i--)
      ;
  }
}

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

union Float32toHex
{
	
	unsigned char C[4];
	float32_t F;
	
}F32_Hex_buf;

union Float64toHex
{
	
	unsigned char C[8];
	float64_t F;
	
}F64_Hex_buf;
	
void
SysTick_Handler(void)
{
  if (tick > 4233600000ll) // 49 days non-reset would cost a tick reset.
  {
    tick = 0;
  }
  tick++;
}

bool Origin_Position_Get_Flag=false;
bool Flight_Control_Flag=false;

float VolThresholdInMps=1.0;

static Telemetry::Quaternion currentBroadcastQ;
static Telemetry::GlobalPosition currentBroadcastGPS;

double originBroadcastGPS_latitude=0;
double originBroadcastGPS_longitude=0;
float originBroadcastGPS_altitude=0;

static Telemetry::Vector3f localOffset;

static Control::CtrlData Control_Command(0x51,0,0,0,0);

double Control_Accuracy_Limit=0.5;
double GPS_Control_Accuracy=0.0000005;//rad,about 3.18m

bool GPS_Control_Flag=false;

int Tele_Data_Cnt=0;

int RC_Mode_For_Emergency=4000;
int RC_Mode_Get_Cnt=0;

void
TIM1_UP_TIM10_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
  {
    //    virtualrc.sendData(myVRCdata);
		
		if(RC_Mode_Get_Cnt%50==0)
		{
			RC_Mode_For_Emergency=v->broadcast->getRC().mode;
		}
		RC_Mode_Get_Cnt++;
		if(RC_Mode_Get_Cnt>=100)RC_Mode_Get_Cnt=0;
		
  }
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
}

void
TIM2_IRQHandler()
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
  {
//    if ((myTerminal.cmdIn[2] == 0x04) && (myTerminal.cmdIn[3] == 0x01))
//    {
//      //      flight.setFlight(&flightData);
//    }
//    else
//    {
//      TIM_Cmd(TIM2, DISABLE);
//    }
		
		if(Tele_Data_Cnt%2==0)
		{
			Flight_GlobalPosition=v->broadcast->getGlobalPosition();
			Flight_Quaternion=v->broadcast->getQuaternion();
			printf("%c",'$');
			printf("%c",'P');
			F64_Hex_buf.F=Flight_GlobalPosition.latitude; printf("%c%c%c%c%c%c%c%c",F64_Hex_buf.C[0],F64_Hex_buf.C[1],F64_Hex_buf.C[2],F64_Hex_buf.C[3],F64_Hex_buf.C[4],F64_Hex_buf.C[5],F64_Hex_buf.C[6],F64_Hex_buf.C[7]);
			F64_Hex_buf.F=Flight_GlobalPosition.longitude; printf("%c%c%c%c%c%c%c%c",F64_Hex_buf.C[0],F64_Hex_buf.C[1],F64_Hex_buf.C[2],F64_Hex_buf.C[3],F64_Hex_buf.C[4],F64_Hex_buf.C[5],F64_Hex_buf.C[6],F64_Hex_buf.C[7]);
			F32_Hex_buf.F=Flight_GlobalPosition.height; printf("%c%c%c%c",F32_Hex_buf.C[0],F32_Hex_buf.C[1],F32_Hex_buf.C[2],F32_Hex_buf.C[3]);
			printf("%c",'Q');
			F32_Hex_buf.F=Flight_Quaternion.q0; printf("%c%c%c%c",F32_Hex_buf.C[0],F32_Hex_buf.C[1],F32_Hex_buf.C[2],F32_Hex_buf.C[3]);
			F32_Hex_buf.F=Flight_Quaternion.q1; printf("%c%c%c%c",F32_Hex_buf.C[0],F32_Hex_buf.C[1],F32_Hex_buf.C[2],F32_Hex_buf.C[3]);
			F32_Hex_buf.F=Flight_Quaternion.q2; printf("%c%c%c%c",F32_Hex_buf.C[0],F32_Hex_buf.C[1],F32_Hex_buf.C[2],F32_Hex_buf.C[3]);
			F32_Hex_buf.F=Flight_Quaternion.q3; printf("%c%c%c%c",F32_Hex_buf.C[0],F32_Hex_buf.C[1],F32_Hex_buf.C[2],F32_Hex_buf.C[3]);
			printf("%c",'#');
				
			if(Origin_Position_Get_Flag==false)
			{
				originBroadcastGPS_latitude = Flight_GlobalPosition.latitude;
				originBroadcastGPS_longitude = Flight_GlobalPosition.longitude;
				originBroadcastGPS_altitude = Flight_GlobalPosition.altitude;
				Origin_Position_Get_Flag=true;
				printf("Original Point has been reset");
			}
					
			if(Flight_Control_Flag==1)
			{
				currentBroadcastGPS = Flight_GlobalPosition;
				currentBroadcastQ = Flight_Quaternion;
					
				localOffset.x = 6356725.0*(currentBroadcastGPS.latitude-originBroadcastGPS_latitude);
				localOffset.y = C_EARTH*cosl(currentBroadcastGPS.latitude)*(currentBroadcastGPS.longitude-originBroadcastGPS_longitude);
				localOffset.z = currentBroadcastGPS.height;
					
				double currentYaw = toEulerAngle((static_cast<void*>(&currentBroadcastQ))).z;
		
				double x_Err = Flight_Command.x - localOffset.x;
				double y_Err = Flight_Command.y - localOffset.y;
				double z_Err = Flight_Command.z - localOffset.z;
				double yaw_Err = Flight_Command.yaw - currentYaw;
				
				double d_Err=__sqrtf(x_Err*x_Err+y_Err*y_Err);//
					
				double P_Vol_Control=0.25;
				
				double D_Vol=P_Vol_Control*d_Err;
				
				if (D_Vol > 0)
					D_Vol = (D_Vol < VolThresholdInMps) ? D_Vol : VolThresholdInMps;
				else if (D_Vol < 0)
					D_Vol =(D_Vol > -1 * VolThresholdInMps) ? D_Vol : -1 * VolThresholdInMps;
				else  
				 D_Vol = 0;
				
				double X_Vol=D_Vol*(x_Err/d_Err);
				double Y_Vol=D_Vol*(y_Err/d_Err);
				
				if(fabs(D_Vol)>Control_Accuracy_Limit||fabs(z_Err)>Control_Accuracy_Limit||fabs(yaw_Err)>Control_Accuracy_Limit*50)
					{
						Control_Command.flag=Flight_Command.flag;
						Control_Command.x=X_Vol;
						Control_Command.y=Y_Vol;
						Control_Command.z=Flight_Command.z;
						Control_Command.yaw=Flight_Command.yaw;
						v->control->flightCtrl(Control_Command);
					}
					else
					{
						Control_Command.flag=Flight_Command.flag;
						Control_Command.x=X_Vol;
						Control_Command.y=Y_Vol;
						Control_Command.z=Flight_Command.z;
						Control_Command.yaw=Flight_Command.yaw;
						v->control->flightCtrl(Control_Command);
						Flight_Control_Flag=false;
					}
			}
			
			if(GPS_Control_Flag==true)
			{
				double La_Err = GPS_Ctrl_Desire_LLA.Desire_Latitude - Flight_GlobalPosition.latitude;
				double Lo_Err = GPS_Ctrl_Desire_LLA.Desire_Longitude - Flight_GlobalPosition.longitude;
				double He_Err = GPS_Ctrl_Desire_LLA.Desire_Height - Flight_GlobalPosition.altitude;
				
				double d_Err=__sqrtf(La_Err*La_Err+Lo_Err*Lo_Err);//	
				
				double P_Vol_Control=750000;
				
				double D_Vol=P_Vol_Control*d_Err;
				
				if (D_Vol > 0)
					D_Vol = (D_Vol < VolThresholdInMps) ? D_Vol : VolThresholdInMps;
				else if (D_Vol < 0)
					D_Vol =(D_Vol > -1 * VolThresholdInMps) ? D_Vol : -1 * VolThresholdInMps;
				else  
				 D_Vol = 0;
				
				double X_Vol=D_Vol*(La_Err/d_Err);
				double Y_Vol=D_Vol*(Lo_Err/d_Err);
				
				if(fabs(D_Vol)>GPS_Control_Accuracy||fabs(He_Err)>Control_Accuracy_Limit)
					{
						Control_Command.flag=GPS_Ctrl_Desire_LLA.Flag;
						Control_Command.x=X_Vol;
						Control_Command.y=Y_Vol;
						Control_Command.z=GPS_Ctrl_Desire_LLA.Desire_Height;
						Control_Command.yaw=0.0;
						v->control->flightCtrl(Control_Command);
					}
					else
					{
						Control_Command.flag=GPS_Ctrl_Desire_LLA.Flag;
						Control_Command.x=X_Vol;
						Control_Command.y=Y_Vol;
						Control_Command.z=GPS_Ctrl_Desire_LLA.Desire_Height;
						Control_Command.yaw=0.0;
						v->control->flightCtrl(Control_Command);
						GPS_Control_Flag=false;
					}
			}
			
			if(Pre_Stop_Telemetry_Flag==true&&Flight_GlobalPosition.height<0.05)
			{
				v->releaseCtrlAuthority();
				Computer_Controling_Flag=false;
				printf("\nTelemetry Package Stop\n");
				Pre_Stop_Telemetry_Flag=false;
				TIM_Cmd(TIM2, DISABLE);
			}
		}
		Tele_Data_Cnt++;
		if(Tele_Data_Cnt>=200)Tele_Data_Cnt=0;
		
  }
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}
#ifdef __cplusplus
}
#endif //__cplusplus
