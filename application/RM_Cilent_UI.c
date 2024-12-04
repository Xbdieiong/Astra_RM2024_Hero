#include "RM_Cilent_UI.h"
#include "usart.h"
#include "math.h"
#include "bsp_usart.h"
#include "CAN_receive.h"
#include "autoaim.h"
#include "referee.h"
#include "servo_task.h"
#include "shoot.h"
#include "referee.h"

unsigned char UI_Seq;                      //包序号
Graph_Data G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11,G12,G13,G14,G15,G16,G17,G18,G19,G20,G21,G22,G23,G24,G25,G26,G27,G28,G29,G30,G31,G32,G33,G34,G35,G36,G37,G38,G39,G40;
String_Data CH_AUTOAIM,CH_ROTATE,CH_SUPERCAP,CH_SNIPER,CH_CAP,CH_HEAT,CH_AUTOAIM_E,CH_ROTATE_E,CH_SNIPER_E,CH_CAP_E,CH_HEAT_E;
Float_Data F1_HEAT,F2_cap,F4_aim;//热量

Int_Data I5_test;
char heat[4]="HEAT";
char cap[3]="CAP";
char sniper[6] = "Sniper";
char rotate[6] = "Rotate";
char autoaim[7] = "AutoAim";
char Defense_buff[7]="Defence";
char DMG_buff[3]="DMG";
char da_fu[7] = "BIGBUFF";
char xiao_fu[7] = "MINBUFF";
//(int)1.3*54,(int)2*54,(int)(0.9*54),(int)1.8*54,(int)1.6*54,(int)1.8*54,
//(int)2.5*54,(int)(0.45*54),(int)1.6*54,(int)(0.8*54),(int)(0.67*54),(int)(0.6*54),
//(int)1.45*54,(int)(0.25*54),(int)(0.85*54),(int)(0.45*54),(int)(0.35*54),(int)(0.3*54),
Graph_Data G_aim_small[16];
char G_aim_small_mark[16][3]={"011","012","013","014","015","016","017","018","019","020","021","022","023","024","025","026"};
int y_offset_screen[2]={(int)1.5*54,(int)2.4*54};
int hight_armor[2]={(int)1.1*54,(int)(0.4*54)};
int half_width_armor[2]={(int)(0.6*54),(int)(0.2*54)};
extern int shoot_speed_add_little;
extern int last_shoot_speed_add_little;
extern int windmill_mode;
extern int xtl_flag;
extern int sniper_flag;
extern int flying_flag;
uint16_t heat1_limit,heat1;
int HEAT_FORMAT(uint16_t HEAT)//热量格式化
{
	float x=0,y=0;
	x=floor(HEAT/100);//取整
	if((HEAT/100)-x>0)
	{
		y=1;
	}
	else
	{
		y=0;
	}
	x+=y;
	return x;
}

void UI_init(void)
{
	/*
	Char_Draw：111、112
	Graph:001-007、瞄准011-026
	Float:101、102
	*/
	//清除所有图层UI
	UI_Delete(UI_Data_Del_ALL,0);
	/*
	底层静态UI 图层1
	电容容量圆弧 图层2
	热量上限圆弧 图层3
	狙击模式图标 图层4
	陀螺模式图标 图层5
	自瞄模式图标 图层6
	电容模式图标 图层7
	*/
	get_shoot_heat1_limit_and_heat1( &heat1_limit,  &heat1);
	//获取热量
	
	/*↓↓↓静态UI（只发送一次）↓↓↓*/
	 //BUFF监视黄色为底色 触发状态变绿
	 /*射击准线&防撞提示*/
	 /*射击准线*/	 
	 memset(&G1,0,sizeof(G1));
	 memset(&G2,0,sizeof(G2));
	 memset(&G3,0,sizeof(G3));
	 memset(&G4,0,sizeof(G4));
	 memset(&G5,0,sizeof(G5));
	 memset(&G6,0,sizeof(G6));
	 memset(&G7,0,sizeof(G7));
	 Line_Draw(&G1,"001",UI_Graph_ADD,1,UI_Color_Green,3,960,540,960,140);//主轴线1
	 Line_Draw(&G2,"002",UI_Graph_ADD,1,UI_Color_Green,3,960,540,823,164);//主轴线2
	 Line_Draw(&G3,"003",UI_Graph_ADD,1,UI_Color_Green,3,960,540,1097,164);//主轴线3
	 Arc_Draw(&G4,"004",UI_Graph_ADD,1,UI_Color_Green,155,205,1,960,540,100,100);//辅助标尺1
	 Arc_Draw(&G5,"005",UI_Graph_ADD,1,UI_Color_Green,155,205,1,960,540,150,150);//辅助标尺2
	 Arc_Draw(&G6,"006",UI_Graph_ADD,1,UI_Color_Green,155,205,1,960,540,200,200);//辅助标尺3
	 Arc_Draw(&G7,"007",UI_Graph_ADD,1,UI_Color_Green,155,205,1,960,540,250,250);//辅助标尺4
	 memset(&G8,0,sizeof(G8));
	 memset(&G9,0,sizeof(G9));
	 memset(&G10,0,sizeof(G10));
	 memset(&G11,0,sizeof(G11));
	 memset(&G12,0,sizeof(G12));
	 memset(&G13,0,sizeof(G13));
	 memset(&G14,0,sizeof(G14));
	 Arc_Draw(&G8,"008",UI_Graph_ADD,1,UI_Color_Green,155,205,1,960,540,300,300);//辅助标尺5
	 Arc_Draw(&G9,"009",UI_Graph_ADD,1,UI_Color_Green,155,205,1,960,540,350,350);//辅助标尺6
	 Arc_Draw(&G10,"010",UI_Graph_ADD,1,UI_Color_Green,155,205,1,960,540,400,400);//辅助标尺7
	 /*防撞提示*/
	 Line_Draw(&G11,"011",UI_Graph_ADD,1,UI_Color_Green,5,289,0,476,405);//左斜线
	 Line_Draw(&G12,"012",UI_Graph_ADD,1,UI_Color_Green,5,474,405,637,405);//左横线
	 Line_Draw(&G13,"013",UI_Graph_ADD,1,UI_Color_Green,5,1444,405,1678,0);//右斜线
	 Line_Draw(&G14,"014",UI_Graph_ADD,1,UI_Color_Green,5,1283,405,1446,405);//右横线	 
	 UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7);
	 UI_ReFresh(7,G8,G9,G10,G11,G12,G13,G14);
	 /*陀螺提示*/
	 memset(&CH_ROTATE,0,sizeof(CH_ROTATE));
	 Char_Draw(&CH_ROTATE,"ROT",UI_Graph_ADD,1,UI_Color_Yellow,15,6,2,1230,700,rotate);//陀螺文字标识
	 /*狙击提示*/	 
	 memset(&CH_SNIPER,0,sizeof(CH_SNIPER));
	 Char_Draw(&CH_SNIPER,"SNI",UI_Graph_ADD,1,UI_Color_Yellow,15,6,2,605,700,sniper);//狙击文字标识	
	 /*电容热量提示*/
	 //图标
	 //电压
	 //圆弧
	 memset(&G25,0,sizeof(G25));
	 memset(&G26,0,sizeof(G26));
	 memset(&G27,0,sizeof(G27));
	 memset(&G28,0,sizeof(G28));
	 memset(&CH_CAP,0,sizeof(CH_CAP));
	 memset(&CH_HEAT,0,sizeof(CH_HEAT));
	 Arc_Draw(&G25,"025",UI_Graph_ADD,1,UI_Color_Yellow,70,110,25,960,540,325,325);//右圆弧槽背景
	 Arc_Draw(&G26,"026",UI_Graph_ADD,1,UI_Color_Yellow,250,290,25,960,540,325,325);//左圆弧槽背景
	 Char_Draw(&CH_CAP,"CAP",UI_Graph_ADD,1,UI_Color_Yellow,15,3,2,1310,540,cap);//电容文字
	 Char_Draw(&CH_HEAT,"HEA",UI_Graph_ADD,1,UI_Color_Yellow,13,4,2,610-42,540,heat);//热量文字
	 /*自瞄提示*/
	 memset(&CH_AUTOAIM,0,sizeof(CH_AUTOAIM));
	 Char_Draw(&CH_AUTOAIM,"AIM",UI_Graph_ADD,2,UI_Color_Yellow,15,7,2,920,815,autoaim);//自瞄文字标识
	 
	 /*经验提示*/
	 //Null
	 /*BUFF提示*/
	 //Null
	 /*整活*/
	 //Null
	 
	 /*动态UI存储空间预定义*/
	 memset(&F2_cap,0,sizeof(F2_cap));//电压示数
	 memset(&CH_ROTATE_E,0,sizeof(CH_ROTATE_E));//陀螺开关
	 memset(&CH_CAP_E,0,sizeof(CH_CAP_E));//电容开关
	 memset(&CH_HEAT_E,0,sizeof(CH_HEAT_E));//热量判断(暂无)
	 memset(&CH_SNIPER_E,0,sizeof(CH_SNIPER_E));//狙击开关
	 memset(&CH_AUTOAIM_E,0,sizeof(CH_AUTOAIM_E));//自瞄开关
	 /*动态刷新初始化*/
	 Arc_Draw(&G27,"027",UI_Graph_ADD,2,UI_Color_Green,110-round(PowerData.CapVot/0.6),110,20,960,540,325,325);//刷新容值
	 Float_Draw(&F2_cap,"102",UI_Graph_ADD,2,UI_Color_Green,10,3,2,1305,520,PowerData.CapVot);//刷新电压示数
	 //Arc_Draw(&G28,"028",UI_Graph_Change,3,UI_Color_Green,250,290,20,960,540,325,325);//刷新热量容值
	 Float_Draw(&F2_cap,"102",UI_Graph_ADD,2,UI_Color_Green,10,3,2,1305,520,PowerData.CapVot);//刷新热量示数
	//刷新热量示数
	//UI_ReFresh(1,G28);
	 UI_ReFresh(1,F2_cap);
	 UI_ReFresh(1,G27);
	 //整体刷新
	 UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7);
	 UI_ReFresh(7,G8,G9,G10,G11,G12,G13,G14);
	 UI_ReFresh(2,G25,G26);
	 Char_ReFresh(CH_CAP);
	 Char_ReFresh(CH_HEAT);
	 Char_ReFresh(CH_ROTATE);
	 Char_ReFresh(CH_SNIPER);
	 Char_ReFresh(CH_AUTOAIM);
	
	
	/*识别圆
	memset(&G5,0,sizeof(G5));//自瞄圆
	Circle_Draw(&G5,"005",UI_Graph_ADD,7,UI_Color_Yellow,1,960,540,50);
	
	UI_ReFresh(5,G1,G2,G3,G4,G5);*/
	
	/*自瞄状态
	memset(&G6,0,sizeof(G6));//状态圆
	Circle_Draw(&G6,"006",UI_Graph_ADD,7,UI_Color_Orange,1,960,540,60);
	
	memset(&CH_AIM,0,sizeof(CH_AIM));
	Char_Draw(&CH_AIM,"111",UI_Graph_ADD,7,UI_Color_Cyan,20,7,2,950,640,auto_aim);
	Char_ReFresh(CH_AIM);*/
}


void UI_ReFreshAll(void)
{	
	Arc_Draw(&G27,"027",UI_Graph_Change,2,UI_Color_Green,110-round(PowerData.CapVot/0.6),110,20,960,540,325,325);//刷新容值
	Float_Draw(&F2_cap,"102",UI_Graph_Change,2,UI_Color_Green,10,3,2,1305,520,PowerData.CapVot);//刷新电压示数
	UI_ReFresh(1,F2_cap);
	UI_ReFresh(1,G27);
	
	//Arc_Draw(&G28,"028",UI_Graph_Change,3,UI_Color_Green,250,290,20,960,540,325,325);//刷新热量
	//刷新热量示数
	//UI_ReFresh(1,G28);
	
	if(xtl_flag)//陀螺更新
	{
		Char_Draw(&CH_ROTATE_E,"ROE",UI_Graph_ADD,5,UI_Color_Main,15,6,2,1230,700,rotate);//陀螺文字标识
		Char_ReFresh(CH_ROTATE_E);
	}
	else
	{
		UI_Delete(UI_Data_Del_Layer,5);
	}
	if(flying_flag)//飞坡更新
	{
		Char_Draw(&CH_CAP_E,"CAE",UI_Graph_ADD,7,UI_Color_Main,15,3,2,1310,540,cap);//电容文字标识
		Char_ReFresh(CH_CAP_E);
	}
	else
	{
		UI_Delete(UI_Data_Del_Layer,7);
	}
	if(sniper_flag)//狙击更新
	{
		Char_Draw(&CH_SNIPER_E,"SNE",UI_Graph_ADD,4,UI_Color_Main,15,6,2,605,700,sniper);//狙击文字标识
		Char_ReFresh(CH_SNIPER_E);
	}
	else
	{
		UI_Delete(UI_Data_Del_Layer,4);
	}
	
}
/****************************************串口驱动映射************************************/
void UI_SendByte(unsigned char ch, uint16_t size)
{
	int a=0;
	while(HAL_UART_Transmit(&huart6, &ch, size, 100)!= HAL_OK)
	{
	 a++;
	}
}

/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/

void UI_Delete(uint8_t Del_Operate,uint8_t Del_Layer)
{

   unsigned char *framepoint;                      //读写指针
   uint16_t frametail=0xFFFF;                        //CRC16校验值
   int loop_control;                       //For函数循环控制
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   UI_Data_Delete del;
   
   framepoint=(unsigned char *)&framehead;
   
   framehead.SOF=UI_SOF;
   framehead.Data_Length=8;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   datahead.Data_ID=UI_Data_ID_Del;
   datahead.Sender_ID=Robot_ID;
	 datahead.Receiver_ID=Client_ID;                         //填充操作数据
   del.Delete_Operate=Del_Operate;
   del.Layer=Del_Layer;                                     //控制信息
   
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&del;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(del),frametail);  //CRC16校验值计算
   
   framepoint=(unsigned char *)&framehead;
   for(loop_control=0;loop_control<sizeof(framehead);loop_control++)
   {
      UI_SendByte(*framepoint,1);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(loop_control=0;loop_control<sizeof(datahead);loop_control++)
   {
      UI_SendByte(*framepoint,1);
      framepoint++;
   }
   framepoint=(unsigned char *)&del;
   for(loop_control=0;loop_control<sizeof(del);loop_control++)
   {
     UI_SendByte(*framepoint,1);
      framepoint++;
   }                                                                 //发送所有帧
   framepoint=(unsigned char *)&frametail;
   for(loop_control=0;loop_control<sizeof(frametail);loop_control++)
   {
      UI_SendByte(*framepoint,1);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   UI_Seq++;                                                         //包序号+1
}
/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_y    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
        
void Line_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=NULL;i++)
   image->graphic_name[2-i]=imagename[i];
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
        
void Rectangle_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=NULL;i++)
   image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Rectangle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
        
void Circle_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t Graph_Radius)
{
   int i;
   for(i=0;i<3&&imagename[i]!=NULL;i++)
   image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_x,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
        
void Arc_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_StartAngle,uint32_t Graph_EndAngle,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t x_Length,uint32_t y_Length)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=NULL;i++)
   image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}



/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_y    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
        
void Float_Draw(Float_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t  Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,float Graph_Float)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=NULL;i++)
   image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = Graph_Digit;
   image->graph_Float = Graph_Float*1000;
}

/************************************************绘制整型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Start_x、Start_y    开始坐标
        Graph_Int   要显示的变量
**********************************************************************************************************/
        
void Int_Draw(Int_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,float Graph_Int)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=NULL;i++)
   image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Int;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = 0;
   image->graph_Int = Graph_Int;
}

/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    字符个数
        Start_x、Start_y    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
        
void Char_Draw(String_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,char *Char_Data)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=NULL;i++)
   image->Graph_Control.graphic_name[2-i]=imagename[i];
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.start_angle = Graph_Size;
   image->Graph_Control.end_angle = Graph_Digit;
   
   for(i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
}

/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int UI_ReFresh(int cnt,...)
{
   int i,n;
   Graph_Data imageData;
   unsigned char *framepoint;                      //读写指针
   uint16_t frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   
   va_list ap;
   va_start(ap,cnt);
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+cnt*15;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据 
   switch(cnt)
   {
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
	 datahead.Sender_ID=Robot_ID;
	 datahead.Receiver_ID=Client_ID;                          //填充操作数据
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);          //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint,1);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint,1);
      framepoint++;
   }
   
   for(i=0;i<cnt;i++)
   {
      imageData=va_arg(ap,Graph_Data);
      
      framepoint=(unsigned char *)&imageData;
      frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验
      
      for(n=0;n<sizeof(imageData);n++)
      {
         UI_SendByte(*framepoint,1);
         framepoint++;             
      }                                               //发送图片帧
   }
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint,1);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   va_end(ap);
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}


/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形 变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
   int i;
   String_Data imageData;
   unsigned char *framepoint;                      //读写指针
   uint16_t frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   imageData=string_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
	 datahead.Data_ID= UI_Data_ID_DrawChar;
	 datahead.Sender_ID=Robot_ID;
	 datahead.Receiver_ID=Client_ID;                   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint,1);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint,1);
      framepoint++;
   }                                                   //发送操作数据  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
     UI_SendByte(*framepoint,1);
     framepoint++;             
   }                                               //发送图片帧
   
   
   
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint,1);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}


/*****************************************************CRC8校验值计算**********************************************/
const unsigned char CRC8_INIT_UI = 0xff; 
const unsigned char CRC8_TAB_UI[256] = 
{ 
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) 
{ 
unsigned char ucIndex; 
while (dwLength--) 
{ 
ucIndex = ucCRC8^(*pchMessage++); 
ucCRC8 = CRC8_TAB_UI[ucIndex]; 
} 
return(ucCRC8); 
}

uint16_t CRC_INIT_UI = 0xffff; 
const uint16_t wCRC_Table_UI[256] = 
{ 
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
Uint8_t chData; 
if (pchMessage == NULL) 
{ 
return 0xFFFF; 
} 
while(dwLength--) 
{ 
chData = *pchMessage++;
(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_UI[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 
0x00ff]; 
} 
return wCRC; 
}

