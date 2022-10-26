//---->>>>----文件描述：EC11旋转编码器底层驱动程序---<<<<----//
//---->>>>----文件版本：V1.0----<<<<----//
#include "EncoderEC11.h"
#include  "stc89c5xrc.h"
#include <STC89C5xRC.H>
  int G_PWM_NUM1;
  int G_PWM_NUM2;
  int G_PWM_NUM3;

//-------->>>>>>>>--------注意事项：EC11旋转编码器的扫描时间间隔控制在1~4ms之间，否则5ms及以上的扫描时间在快速旋转时可能会误判旋转方向--------<<<<<<<<--------//
 
//*******************************************************************/
//功能：初始化EC11旋转编码器相关参数
//形参：EC11旋转编码器的类型-->>  unsigned char Set_EC11_TYPE  <<--  ：0----一定位对应一脉冲；1（或非0）----两定位对应一脉冲。
//返回：无
//详解：对EC11旋转编码器的连接IO口做IO口模式设置。以及将相关的变量进行初始化
//*******************************************************************/
void Encoder_EC11_Init(unsigned char Set_EC11_TYPE)    //EC11类型选择：0-一定位一脉冲；1-两定位一脉冲
{
    //IO口模式初始化。初始化EC11的IO口为准双向模式

    EC11_A_Now = 1;
    EC11_B_Now = 1;
    EC11_Key = 1;
 


    if(Set_EC11_TYPE == 0)
    {
        EC11_Type = 0;
    }
    else
    {
        EC11_Type = 1;
    }
 
    //避免上电时EC11旋钮位置不确定导致一次动作误判
    EC11_A_Last = EC11_A_Now;   
    EC11_B_Last = EC11_B_Now;
 
    //--------清除按键计数器和标志位--------//
    EC11_KEY_COUNT = 0;                     //EC11按键动作计数器
    EC11_KEY_DoubleClick_Count = 0;         //EC11按键双击动作计数器
    FLAG_EC11_KEY_ShotClick = 0;            //EC11按键短按动作标志
    FLAG_EC11_KEY_LongClick = 0;            //EC11按键长按动作标志
    FLAG_EC11_KEY_DoubleClick = 0;          //EC11按键双击动作标志
}
 
 
//*******************************************************************/
//功能：扫描EC11旋转编码器的动作并将参数返回给动作分析函数使用
//形参：EC11旋转编码器的类型-->>  unsigned char Set_EC11_TYPE  <<--  ：0----一定位对应一脉冲；1（或非0）----两定位对应一脉冲
//返回：EC11旋转编码器的扫描结果-->>  char ScanResult  -->>  0：无动作；1：正转； -1：反转；2：只按下按键；3：按着按键正转；-3：按着按键反转
//详解：只扫描EC11旋转编码器有没有动作，不关心是第几次按下按键或长按或双击。返回值直接作为形参传给 [ void Encoder_EC11_Analyze(char EC11_Value); ] 函数使用
//*******************************************************************/
char Encoder_EC11_Scan()
{
//以下储存A、B上一次值的变量声明为静态全局变量，方便对EC11对应的IO口做初始化
//  static char EC11_A_Last = 0;
//  static char EC11_B_Last = 0;
    char ScanResult = 0;    //返回编码器扫描结果，用于分析编码器的动作
                            //返回值的取值：   0：无动作；      1：正转；           -1：反转；  
                            //                  2：只按下按键；    3：按着按键正转；   -3：按着按键反转
 
                            //======================================================//
    if(EC11_Type == 0)      //================一定位对应一脉冲的EC11================//
    {                       //======================================================//
        if(EC11_A_Now != EC11_A_Last)   //以A为时钟，B为数据。正转时AB反相，反转时AB同相
        {
            if(EC11_A_Now == 0)
            {
                if(EC11_B_Now ==1)      //只需要采集A的上升沿或下降沿的任意一个状态，若A下降沿时B为1，正转                    
                    ScanResult = 1;     //正转
 
                else                    //反转
                    ScanResult = -1;
            }
            EC11_A_Last = EC11_A_Now;   //更新编码器上一个状态暂存变量
            EC11_B_Last = EC11_B_Now;   //更新编码器上一个状态暂存变量
        }
    }   
                            //======================================================//
    else                    //================两定位对应一脉冲的EC11================//
    {                       //======================================================//
        if(EC11_A_Now !=EC11_A_Last)        //当A发生跳变时采集B当前的状态，并将B与上一次的状态进行对比。
        {                                   //若A 0->1 时，B 1->0 正转；若A 1->0 时，B 0->1 正转；
                                            //若A 0->1 时，B 0->1 反转；若A 1->0 时，B 1->0 反转
            if(EC11_A_Now == 1)     //EC11_A和上一次状态相比，为上升沿
            {
                if((EC11_B_Last == 1)&&(EC11_B_Now == 0))   //EC11_B和上一次状态相比，为下降沿
                    ScanResult = 1;                         //正转
 
                if((EC11_B_Last == 0)&&(EC11_B_Now == 1))   //EC11_B和上一次状态相比，为上升沿               
                    ScanResult = -1;                        //反转
 
                //>>>>>>>>>>>>>>>>下面为正转一次再反转或反转一次再正转处理<<<<<<<<<<<<<<<<//
                if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 0))  //A上升沿时，采集的B不变且为0
                    ScanResult = 1;                                 //正转
 
                if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 1))  //A上升沿时，采集的B不变且为1
                    ScanResult = -1;                                //反转
            }
 
            else                    //EC11_A和上一次状态相比，为下降沿
            {
                if((EC11_B_Last == 1)&&(EC11_B_Now == 0))   //EC11_B和上一次状态相比，为下降沿
                    ScanResult = -1;                        //反转
 
                if((EC11_B_Last == 0)&&(EC11_B_Now == 1))   //EC11_B和上一次状态相比，为上升沿
                    ScanResult = 1;                         //正转
 
                //>>>>>>>>>>>>>>>>下面为正转一次再反转或反转一次再正转处理<<<<<<<<<<<<<<<<//
                if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 0))  //A上升沿时，采集的B不变且为0
                    ScanResult = -1;                                //反转
 
                if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 1))  //A上升沿时，采集的B不变且为1   
                    ScanResult = 1;                                 //正转
 
            }               
            EC11_A_Last = EC11_A_Now;   //更新编码器上一个状态暂存变量
            EC11_B_Last = EC11_B_Now;   //更新编码器上一个状态暂存变量
        }
    }                                                                       
 
    if(EC11_Key == 0)   //如果EC11的按键按下，并且没有EC11没有转动，
    {
        if(ScanResult == 0)         //按下按键时未转动
            ScanResult = 2;         //返回值为2
        else
        {
            if(ScanResult == 1)     //按下按键时候正转
                ScanResult = 3;     //返回值为3
            if(ScanResult == -1)    //按下按键时候反转
                ScanResult = -3;    //返回值为-3
        }
    }
 
    return ScanResult;      //返回值的取值：   0：无动作；      1：正转；           -1：反转；
}                           //              2：只按下按键；    3：按着按键正转；   -3：按着按键反转
 
 
//*******************************************************************/
//功能：对EC11旋转编码器的动作进行分析，并作出相应的动作处理代码
//形参：无
//返回：char AnalyzeResult = 0;目前无用。若在该函数里做了动作处理，则函数的返回值无需理会
//详解：对EC11旋转编码器的动作进行模式分析，是单击还是双击还是长按松手还是一直按下。形参从 [ char Encoder_EC11_Scan(unsigned char Set_EC11_TYPE) ] 函数传入。在本函数内修改需要的动作处理代码
//*******************************************************************/
char Encoder_EC11_Analyze(char EC11_Value)
{
    char AnalyzeResult = 0;
    static unsigned int TMP_Value = 0;  //中间计数值，用于连续长按按键的动作延时间隔
    //>>>>>>>>>>>>>>>>编码器正转处理程序<<<<<<<<<<<<<<<<//
    if(EC11_Value == 1) //正转
    {
        //--------编码器正转动作代码--------//
        switch(EC11_NUM_SW)
        {
            case 1: G_PWM_NUM1+=10; if(G_PWM_NUM1>255)G_PWM_NUM1 = 0;   break;
            case 2: G_PWM_NUM2+=10; if(G_PWM_NUM1>255)G_PWM_NUM2 = 0;   break;
            case 3: G_PWM_NUM3+=10; if(G_PWM_NUM1>255)G_PWM_NUM3 = 0;   break;
            case 4: G_PWM_NUM1+=10;     if(G_PWM_NUM1>255)G_PWM_NUM1 = 0;   G_PWM_NUM3 = G_PWM_NUM2 = G_PWM_NUM1;   break;
            case 5: G_PWM_NUM1+=20; if(G_PWM_NUM1>=255){G_PWM_NUM1 = 0;G_PWM_NUM3+=20;if(G_PWM_NUM3>=255){G_PWM_NUM3 = 0;G_PWM_NUM2+=20;if(G_PWM_NUM2 >=255)G_PWM_NUM2 = 0;}}
            default :break;
        }

        Volt_OutPut+=100;

    }
 
    //>>>>>>>>>>>>>>>>编码器反转处理程序<<<<<<<<<<<<<<<<//
    if(EC11_Value == -1)    //反转
    {
        //--------编码器反转动作代码--------//
        switch(EC11_NUM_SW)
        {
            case 1: G_PWM_NUM1-=10; if(G_PWM_NUM1<0)G_PWM_NUM1 = 255;   break;
            case 2: G_PWM_NUM2-=10; if(G_PWM_NUM2<0)G_PWM_NUM1 = 255;   break;
            case 3: G_PWM_NUM3-=10; if(G_PWM_NUM3<0)G_PWM_NUM1 = 255;   break;
            case 4: G_PWM_NUM1-=10; if(G_PWM_NUM1<0)G_PWM_NUM1 = 255;   G_PWM_NUM3 = G_PWM_NUM2 = G_PWM_NUM1;   break;
            case 5: G_PWM_NUM1-=20; if(G_PWM_NUM1<0){G_PWM_NUM1 = 255;G_PWM_NUM3-=20;if(G_PWM_NUM3<0){G_PWM_NUM3 = 255;G_PWM_NUM2-=20;if(G_PWM_NUM2 <0)G_PWM_NUM2 = 255;}}
            default :break;
        }
        Volt_OutPut-=100;
    }
 
 
    //>>>>>>>>>>>>>>>>编码器按键按下并正转处理程序<<<<<<<<<<<<<<<<//
    if(EC11_Value == 3)
    {
        //--------编码器按键按下并正转动作代码--------//
        Volt_OutPut+=15;
        
    }
 
    //>>>>>>>>>>>>>>>>编码器按键按下并反转处理程序<<<<<<<<<<<<<<<<//
    if(EC11_Value == -3)
    {
        //--------编码器按键按下并反转动作代码--------//
        Volt_OutPut-=15;
        
    }
 
 
    //>>>>>>>>>>>>>>>>编码器按键按下处理程序<<<<<<<<<<<<<<<<//
    if(EC11_Value == 2)     //====检测到按键按下====//
    {
        if(EC11_KEY_COUNT<10000)    //打开按键按下时间定时器
            {
                EC11_KEY_COUNT++;
            }
        if(EC11_KEY_COUNT == KEY_COUNT_DESHAKING)   //按下按键时间到达消抖时间时
        {                                           //置位短按按键标志
            FLAG_EC11_KEY_ShotClick = 1;
        }
 
        if((EC11_KEY_DoubleClick_Count > 0)&&(EC11_KEY_DoubleClick_Count <= KEY_COUNT_DUALCLICKTIME))   //松开按键后，又在定时器在双击时间内按下按键
        {                                                                                               //置位双击按键标志
            FLAG_EC11_KEY_DoubleClick = 1;
        }
 
        if(EC11_KEY_COUNT == KEY_COUNT_LONGTIME)    //按下按键时间到达长按时间
        {                                           //置位长按按键标志并复位短按按键标志
            FLAG_EC11_KEY_LongClick = 1;
            FLAG_EC11_KEY_ShotClick = 0;
        }
 
    }
    else                    //====检测到按键松开====//     
    {
        if(EC11_KEY_COUNT < KEY_COUNT_DESHAKING)    //没到消抖时长就松开按键，复位所有定时器和按键标志
        {
            EC11_KEY_COUNT = 0;
            FLAG_EC11_KEY_ShotClick = 0;
            FLAG_EC11_KEY_LongClick = 0;
            FLAG_EC11_KEY_DoubleClick = 0;
            EC11_KEY_DoubleClick_Count = 0;
        }
        else
        {
            
            if(FLAG_EC11_KEY_ShotClick == 1)        //短按按键定时有效期间
            {
                if((FLAG_EC11_KEY_DoubleClick == 0)&&(EC11_KEY_DoubleClick_Count >= 0)) 
                        EC11_KEY_DoubleClick_Count++;
                if((FLAG_EC11_KEY_DoubleClick == 1)&&(EC11_KEY_DoubleClick_Count <= KEY_COUNT_DUALCLICKTIME))   //如果在规定双击时间内再次按下按键
                {                                                                                               //认为按键是双击动作
                    FLAG_EC11_KEY_DoubleClick = 2;
                    
                }   
 
                if((FLAG_EC11_KEY_DoubleClick == 0)&&(EC11_KEY_DoubleClick_Count > KEY_COUNT_DUALCLICKTIME))    //如果没有在规定双击时间内再次按下按键
                    FLAG_EC11_KEY_ShotClick = 0;                                                                //认为按键是单击动作
                    
            }
 
            if(FLAG_EC11_KEY_LongClick == 1)        //检测到长按按键松开
                FLAG_EC11_KEY_LongClick = 0;
                //Volt_OutPut=500;
        }
 
    }
 
 
    //>>>>>>>>>>>>>>>>编码器按键分析处理程序<<<<<<<<<<<<<<<<//
    if(EC11_KEY_COUNT > KEY_COUNT_DESHAKING)    //短按按键延时到了时间
    {
 
        //短按按键动作结束代码
        if((FLAG_EC11_KEY_ShotClick == 0)&&(EC11_KEY_DoubleClick_Count > KEY_COUNT_DUALCLICKTIME)&&(EC11_KEY_COUNT < KEY_COUNT_LONGTIME))   //短按按键动作结束代码
        {
            //--------短按按键动作结束代码--------//
            EC11_NUM_SW++;
            if(EC11_NUM_SW >= 4)
                EC11_NUM_SW = 1;
            AnalyzeResult = 1;
            //--------清除标志位--------//
            EC11_KEY_COUNT = 0;
            EC11_KEY_DoubleClick_Count = 0;
            FLAG_EC11_KEY_DoubleClick = 0;
        }
 
        //双击按键动作结束代码
        if((FLAG_EC11_KEY_DoubleClick == 2)&&(EC11_KEY_DoubleClick_Count > 0)&&(EC11_KEY_DoubleClick_Count <= KEY_COUNT_DUALCLICKTIME)) //双击按键动作结束代码
        {
            //--------双击按键动作结束代码--------//
            if(EC11_NUM_SW == 5)
                EC11_NUM_SW = 0;
            if(EC11_NUM_SW == 4)
                EC11_NUM_SW = 5;
 
            if(EC11_NUM_SW <4)
            {
                EC11_NUM_SW = 4;
            }
            AnalyzeResult = 2;
            //--------清除标志位--------//
            EC11_KEY_COUNT = 0;
            EC11_KEY_DoubleClick_Count = 0;
            FLAG_EC11_KEY_ShotClick = 0;
            FLAG_EC11_KEY_DoubleClick = 0;
            
        }
 
        //连续长按按键按下代码
        if((FLAG_EC11_KEY_LongClick == 1)&&(EC11_KEY_COUNT >= KEY_COUNT_LONGTIME))  //连续长按按键按下代码
        {
            TMP_Value ++;
            if(TMP_Value % KEY_LONG_REPEAT_TIME == 0)
            {
                TMP_Value = 0;
                //-------连续长按按键按下代码--------//
                AnalyzeResult = 4;
            }
        }
 
        //长按按键动作结束代码
        if((FLAG_EC11_KEY_LongClick == 0)&&(EC11_KEY_COUNT >= KEY_COUNT_LONGTIME))  //长按按键动作结束代码
        {                                                                           
            //--------长按按键按下动作结束代码--------//
            EC11_NUM_SW = 0;
            G_PWM_NUM1 = 0x20;
            G_PWM_NUM2 = 0x20;
            G_PWM_NUM3 = 0x20;
            AnalyzeResult = 3;
            //--------清除标志位--------//
            EC11_KEY_COUNT = 0;
        }
 
 
    }
    
    return AnalyzeResult;

    
}