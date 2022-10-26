//---->>>>----�ļ�������EC11��ת�������ײ���������---<<<<----//
//---->>>>----�ļ��汾��V1.0----<<<<----//
#include "EncoderEC11.h"
#include  "stc89c5xrc.h"
#include <STC89C5xRC.H>
  int G_PWM_NUM1;
  int G_PWM_NUM2;
  int G_PWM_NUM3;

//-------->>>>>>>>--------ע�����EC11��ת��������ɨ��ʱ����������1~4ms֮�䣬����5ms�����ϵ�ɨ��ʱ���ڿ�����תʱ���ܻ�������ת����--------<<<<<<<<--------//
 
//*******************************************************************/
//���ܣ���ʼ��EC11��ת��������ز���
//�βΣ�EC11��ת������������-->>  unsigned char Set_EC11_TYPE  <<--  ��0----һ��λ��Ӧһ���壻1�����0��----����λ��Ӧһ���塣
//���أ���
//��⣺��EC11��ת������������IO����IO��ģʽ���á��Լ�����صı������г�ʼ��
//*******************************************************************/
void Encoder_EC11_Init(unsigned char Set_EC11_TYPE)    //EC11����ѡ��0-һ��λһ���壻1-����λһ����
{
    //IO��ģʽ��ʼ������ʼ��EC11��IO��Ϊ׼˫��ģʽ

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
 
    //�����ϵ�ʱEC11��ťλ�ò�ȷ������һ�ζ�������
    EC11_A_Last = EC11_A_Now;   
    EC11_B_Last = EC11_B_Now;
 
    //--------��������������ͱ�־λ--------//
    EC11_KEY_COUNT = 0;                     //EC11��������������
    EC11_KEY_DoubleClick_Count = 0;         //EC11����˫������������
    FLAG_EC11_KEY_ShotClick = 0;            //EC11�����̰�������־
    FLAG_EC11_KEY_LongClick = 0;            //EC11��������������־
    FLAG_EC11_KEY_DoubleClick = 0;          //EC11����˫��������־
}
 
 
//*******************************************************************/
//���ܣ�ɨ��EC11��ת�������Ķ��������������ظ�������������ʹ��
//�βΣ�EC11��ת������������-->>  unsigned char Set_EC11_TYPE  <<--  ��0----һ��λ��Ӧһ���壻1�����0��----����λ��Ӧһ����
//���أ�EC11��ת��������ɨ����-->>  char ScanResult  -->>  0���޶�����1����ת�� -1����ת��2��ֻ���°�����3�����Ű�����ת��-3�����Ű�����ת
//��⣺ֻɨ��EC11��ת��������û�ж������������ǵڼ��ΰ��°����򳤰���˫��������ֱֵ����Ϊ�βδ��� [ void Encoder_EC11_Analyze(char EC11_Value); ] ����ʹ��
//*******************************************************************/
char Encoder_EC11_Scan()
{
//���´���A��B��һ��ֵ�ı�������Ϊ��̬ȫ�ֱ����������EC11��Ӧ��IO������ʼ��
//  static char EC11_A_Last = 0;
//  static char EC11_B_Last = 0;
    char ScanResult = 0;    //���ر�����ɨ���������ڷ����������Ķ���
                            //����ֵ��ȡֵ��   0���޶�����      1����ת��           -1����ת��  
                            //                  2��ֻ���°�����    3�����Ű�����ת��   -3�����Ű�����ת
 
                            //======================================================//
    if(EC11_Type == 0)      //================һ��λ��Ӧһ�����EC11================//
    {                       //======================================================//
        if(EC11_A_Now != EC11_A_Last)   //��AΪʱ�ӣ�BΪ���ݡ���תʱAB���࣬��תʱABͬ��
        {
            if(EC11_A_Now == 0)
            {
                if(EC11_B_Now ==1)      //ֻ��Ҫ�ɼ�A�������ػ��½��ص�����һ��״̬����A�½���ʱBΪ1����ת                    
                    ScanResult = 1;     //��ת
 
                else                    //��ת
                    ScanResult = -1;
            }
            EC11_A_Last = EC11_A_Now;   //���±�������һ��״̬�ݴ����
            EC11_B_Last = EC11_B_Now;   //���±�������һ��״̬�ݴ����
        }
    }   
                            //======================================================//
    else                    //================����λ��Ӧһ�����EC11================//
    {                       //======================================================//
        if(EC11_A_Now !=EC11_A_Last)        //��A��������ʱ�ɼ�B��ǰ��״̬������B����һ�ε�״̬���жԱȡ�
        {                                   //��A 0->1 ʱ��B 1->0 ��ת����A 1->0 ʱ��B 0->1 ��ת��
                                            //��A 0->1 ʱ��B 0->1 ��ת����A 1->0 ʱ��B 1->0 ��ת
            if(EC11_A_Now == 1)     //EC11_A����һ��״̬��ȣ�Ϊ������
            {
                if((EC11_B_Last == 1)&&(EC11_B_Now == 0))   //EC11_B����һ��״̬��ȣ�Ϊ�½���
                    ScanResult = 1;                         //��ת
 
                if((EC11_B_Last == 0)&&(EC11_B_Now == 1))   //EC11_B����һ��״̬��ȣ�Ϊ������               
                    ScanResult = -1;                        //��ת
 
                //>>>>>>>>>>>>>>>>����Ϊ��תһ���ٷ�ת��תһ������ת����<<<<<<<<<<<<<<<<//
                if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 0))  //A������ʱ���ɼ���B������Ϊ0
                    ScanResult = 1;                                 //��ת
 
                if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 1))  //A������ʱ���ɼ���B������Ϊ1
                    ScanResult = -1;                                //��ת
            }
 
            else                    //EC11_A����һ��״̬��ȣ�Ϊ�½���
            {
                if((EC11_B_Last == 1)&&(EC11_B_Now == 0))   //EC11_B����һ��״̬��ȣ�Ϊ�½���
                    ScanResult = -1;                        //��ת
 
                if((EC11_B_Last == 0)&&(EC11_B_Now == 1))   //EC11_B����һ��״̬��ȣ�Ϊ������
                    ScanResult = 1;                         //��ת
 
                //>>>>>>>>>>>>>>>>����Ϊ��תһ���ٷ�ת��תһ������ת����<<<<<<<<<<<<<<<<//
                if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 0))  //A������ʱ���ɼ���B������Ϊ0
                    ScanResult = -1;                                //��ת
 
                if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 1))  //A������ʱ���ɼ���B������Ϊ1   
                    ScanResult = 1;                                 //��ת
 
            }               
            EC11_A_Last = EC11_A_Now;   //���±�������һ��״̬�ݴ����
            EC11_B_Last = EC11_B_Now;   //���±�������һ��״̬�ݴ����
        }
    }                                                                       
 
    if(EC11_Key == 0)   //���EC11�İ������£�����û��EC11û��ת����
    {
        if(ScanResult == 0)         //���°���ʱδת��
            ScanResult = 2;         //����ֵΪ2
        else
        {
            if(ScanResult == 1)     //���°���ʱ����ת
                ScanResult = 3;     //����ֵΪ3
            if(ScanResult == -1)    //���°���ʱ��ת
                ScanResult = -3;    //����ֵΪ-3
        }
    }
 
    return ScanResult;      //����ֵ��ȡֵ��   0���޶�����      1����ת��           -1����ת��
}                           //              2��ֻ���°�����    3�����Ű�����ת��   -3�����Ű�����ת
 
 
//*******************************************************************/
//���ܣ���EC11��ת�������Ķ������з�������������Ӧ�Ķ����������
//�βΣ���
//���أ�char AnalyzeResult = 0;Ŀǰ���á����ڸú��������˶������������ķ���ֵ�������
//��⣺��EC11��ת�������Ķ�������ģʽ�������ǵ�������˫�����ǳ������ֻ���һֱ���¡��βδ� [ char Encoder_EC11_Scan(unsigned char Set_EC11_TYPE) ] �������롣�ڱ��������޸���Ҫ�Ķ����������
//*******************************************************************/
char Encoder_EC11_Analyze(char EC11_Value)
{
    char AnalyzeResult = 0;
    static unsigned int TMP_Value = 0;  //�м����ֵ�������������������Ķ�����ʱ���
    //>>>>>>>>>>>>>>>>��������ת�������<<<<<<<<<<<<<<<<//
    if(EC11_Value == 1) //��ת
    {
        //--------��������ת��������--------//
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
 
    //>>>>>>>>>>>>>>>>��������ת�������<<<<<<<<<<<<<<<<//
    if(EC11_Value == -1)    //��ת
    {
        //--------��������ת��������--------//
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
 
 
    //>>>>>>>>>>>>>>>>�������������²���ת�������<<<<<<<<<<<<<<<<//
    if(EC11_Value == 3)
    {
        //--------�������������²���ת��������--------//
        Volt_OutPut+=15;
        
    }
 
    //>>>>>>>>>>>>>>>>�������������²���ת�������<<<<<<<<<<<<<<<<//
    if(EC11_Value == -3)
    {
        //--------�������������²���ת��������--------//
        Volt_OutPut-=15;
        
    }
 
 
    //>>>>>>>>>>>>>>>>�������������´������<<<<<<<<<<<<<<<<//
    if(EC11_Value == 2)     //====��⵽��������====//
    {
        if(EC11_KEY_COUNT<10000)    //�򿪰�������ʱ�䶨ʱ��
            {
                EC11_KEY_COUNT++;
            }
        if(EC11_KEY_COUNT == KEY_COUNT_DESHAKING)   //���°���ʱ�䵽������ʱ��ʱ
        {                                           //��λ�̰�������־
            FLAG_EC11_KEY_ShotClick = 1;
        }
 
        if((EC11_KEY_DoubleClick_Count > 0)&&(EC11_KEY_DoubleClick_Count <= KEY_COUNT_DUALCLICKTIME))   //�ɿ����������ڶ�ʱ����˫��ʱ���ڰ��°���
        {                                                                                               //��λ˫��������־
            FLAG_EC11_KEY_DoubleClick = 1;
        }
 
        if(EC11_KEY_COUNT == KEY_COUNT_LONGTIME)    //���°���ʱ�䵽�ﳤ��ʱ��
        {                                           //��λ����������־����λ�̰�������־
            FLAG_EC11_KEY_LongClick = 1;
            FLAG_EC11_KEY_ShotClick = 0;
        }
 
    }
    else                    //====��⵽�����ɿ�====//     
    {
        if(EC11_KEY_COUNT < KEY_COUNT_DESHAKING)    //û������ʱ�����ɿ���������λ���ж�ʱ���Ͱ�����־
        {
            EC11_KEY_COUNT = 0;
            FLAG_EC11_KEY_ShotClick = 0;
            FLAG_EC11_KEY_LongClick = 0;
            FLAG_EC11_KEY_DoubleClick = 0;
            EC11_KEY_DoubleClick_Count = 0;
        }
        else
        {
            
            if(FLAG_EC11_KEY_ShotClick == 1)        //�̰�������ʱ��Ч�ڼ�
            {
                if((FLAG_EC11_KEY_DoubleClick == 0)&&(EC11_KEY_DoubleClick_Count >= 0)) 
                        EC11_KEY_DoubleClick_Count++;
                if((FLAG_EC11_KEY_DoubleClick == 1)&&(EC11_KEY_DoubleClick_Count <= KEY_COUNT_DUALCLICKTIME))   //����ڹ涨˫��ʱ�����ٴΰ��°���
                {                                                                                               //��Ϊ������˫������
                    FLAG_EC11_KEY_DoubleClick = 2;
                    
                }   
 
                if((FLAG_EC11_KEY_DoubleClick == 0)&&(EC11_KEY_DoubleClick_Count > KEY_COUNT_DUALCLICKTIME))    //���û���ڹ涨˫��ʱ�����ٴΰ��°���
                    FLAG_EC11_KEY_ShotClick = 0;                                                                //��Ϊ�����ǵ�������
                    
            }
 
            if(FLAG_EC11_KEY_LongClick == 1)        //��⵽���������ɿ�
                FLAG_EC11_KEY_LongClick = 0;
                //Volt_OutPut=500;
        }
 
    }
 
 
    //>>>>>>>>>>>>>>>>���������������������<<<<<<<<<<<<<<<<//
    if(EC11_KEY_COUNT > KEY_COUNT_DESHAKING)    //�̰�������ʱ����ʱ��
    {
 
        //�̰�����������������
        if((FLAG_EC11_KEY_ShotClick == 0)&&(EC11_KEY_DoubleClick_Count > KEY_COUNT_DUALCLICKTIME)&&(EC11_KEY_COUNT < KEY_COUNT_LONGTIME))   //�̰�����������������
        {
            //--------�̰�����������������--------//
            EC11_NUM_SW++;
            if(EC11_NUM_SW >= 4)
                EC11_NUM_SW = 1;
            AnalyzeResult = 1;
            //--------�����־λ--------//
            EC11_KEY_COUNT = 0;
            EC11_KEY_DoubleClick_Count = 0;
            FLAG_EC11_KEY_DoubleClick = 0;
        }
 
        //˫������������������
        if((FLAG_EC11_KEY_DoubleClick == 2)&&(EC11_KEY_DoubleClick_Count > 0)&&(EC11_KEY_DoubleClick_Count <= KEY_COUNT_DUALCLICKTIME)) //˫������������������
        {
            //--------˫������������������--------//
            if(EC11_NUM_SW == 5)
                EC11_NUM_SW = 0;
            if(EC11_NUM_SW == 4)
                EC11_NUM_SW = 5;
 
            if(EC11_NUM_SW <4)
            {
                EC11_NUM_SW = 4;
            }
            AnalyzeResult = 2;
            //--------�����־λ--------//
            EC11_KEY_COUNT = 0;
            EC11_KEY_DoubleClick_Count = 0;
            FLAG_EC11_KEY_ShotClick = 0;
            FLAG_EC11_KEY_DoubleClick = 0;
            
        }
 
        //���������������´���
        if((FLAG_EC11_KEY_LongClick == 1)&&(EC11_KEY_COUNT >= KEY_COUNT_LONGTIME))  //���������������´���
        {
            TMP_Value ++;
            if(TMP_Value % KEY_LONG_REPEAT_TIME == 0)
            {
                TMP_Value = 0;
                //-------���������������´���--------//
                AnalyzeResult = 4;
            }
        }
 
        //��������������������
        if((FLAG_EC11_KEY_LongClick == 0)&&(EC11_KEY_COUNT >= KEY_COUNT_LONGTIME))  //��������������������
        {                                                                           
            //--------�����������¶�����������--------//
            EC11_NUM_SW = 0;
            G_PWM_NUM1 = 0x20;
            G_PWM_NUM2 = 0x20;
            G_PWM_NUM3 = 0x20;
            AnalyzeResult = 3;
            //--------�����־λ--------//
            EC11_KEY_COUNT = 0;
        }
 
 
    }
    
    return AnalyzeResult;

    
}