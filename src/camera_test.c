#include "common.h"
#include "include.h"
#include "camera_test.h"
#include "misc.h"
#include "OLED.h"
#include "servo.h"
#include "motor.h"
#include "PID.h"
extern volatile IMG_STATUS_e      ov7725_eagle_img_flag;


#include "common.h"
#include "include.h"
#include "misc.h"

//#include "stdlib.h"
//#define ThreadHold 140    //�����ڰ׷ָ���ֵ��������������̬���ó������������������Զ�Ϊ��ɫ�򽫸�ֵ���󣬷����С��ֱ���������
#define rowStart 30       //ͼ������ʼ����
#define rowEnd   33     //ͼ������ֹ����
#define rowNum   rowEnd-rowStart  //��Ч����������ԭʼ�ɼ�����ΪROW(��Include.h�ж���)
#define proPix   5        //��Ե���������߽���Ч����
#define CENTURE  39      //����ֵ
#define servMotorCenture  1175    //san 1175  si 1310
#define servMotorLeft    1330    //san 1330  si 1430
#define servMotorRight    1010     //san 1020  si 1190  1090
//#define servPram          9       //���ת�Ǳ���ϵ��������������������Ƕȸ����������ֵ��������ǶȰڷ��ܴ��С��ֵ
uint8 servPram;
#if 0
#define MOTOR_HZ    (50)
#else
#define MOTOR_HZ    (20*1000)
#endif


  #define BLACK     0     
  #define WHITE    255


#define   EDGE  WHITE-BLACK  //�߽�ֵ��С
//#define ROW 60
//#define COL 80
//uint8 tempBuffer1[ROW][COL]={0};      //ͨ��DMA�ɼ�(��isr.c�����ж��д���DMA�ɼ�)��άͼ��洢��tempBuffer1������
uint32 left[rowNum] = {0};         //�����Ч��������˫�����Ե�������꣬��ʼ��Ϊ0
uint32 right[rowNum] = {CAMERA_W-1};     //�����Ч��������˫����Ե�������꣬��ʼ��ΪCOL-1
 volatile uint32 servPWMDuty;
int32 leftPos = 0;          //��߽�λ��[�з�����]
int32 rightPos = 0;          //�ұ߽�λ��[�з�����]
uint8 centure = 0;         //������ĵ�ǰ���������λ��[�޷�����]
int32 centureErr = 0;      //������ĵ�ǰ����λ��ƫ�� [�з�����]
uint32 turn = 0;            //�߽�����  ��Ϊ����ƫת  ��Ϊ����ƫת


int32 servPWMErr = 0;

extern volatile IMG_STATUS_e      ov7725_eagle_img_flag;
extern volatile PIDConst SevPID;
uint8 time;
extern int16 speed_out_set;//210
extern int16 speed_out_set_str;//300

volatile uint32 rowCnt = 0 ;//�м���

void ImageProc();
void DataReset();
uint8 imgbuff[CAMERA_SIZE];    //����洢����ͼ�������
//uint8 img[CAMERA_W*CAMERA_H]; //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��
uint8 img[CAMERA_H][CAMERA_W];
                               //������Ҫ��ά���飬ֻ��Ҫ�ĳ� uint8 img[CAMERA_H][CAMERA_W];
                               //imgbuff�ǲɼ��Ļ�������img�ǽ�ѹ��Ļ�������imgbuff���ڲɼ�ͼ��img����ͼ����.

/************����ͣ��**************/
#define Numgap  whiteNum-blackNum//�ڰ�������
#define ovtrowStart 36
uint32 i,j;
uint8 ovtflag=0;//=1����
uint8 leftEdgeflag=1;      //��߽�ȷ��kong��־���ҵ���߽���λ
uint8 rightEdgeflag=1;      //�ұ߽�ȷ��kong��־���ҵ���߽���λ
uint8 servflag=0;//ת�����1
//uint32 whiteNum=0;
//uint32 blackNum=0;
uint32 ovtleft[20]={0};
uint32 ovtright[20]={0}; 
int whiteNum;
int whitejudge,whiteNum_last;
int whitejudge1;
int ovtleftMax=0;
int ovtleftMin=80;
int ovtrightMax=0;
int ovtrightMin=80;
int GOAHEAD = 0;
extern int32 out;
int stopflag;
//uint32 PITcount=0; 
/************************************/



void Mycamera (void)         
{
    //camera_init(imgbuff);
    camera_get_img();//����ͷ��ȡͼ��
    img_extract(&img[0][0], &imgbuff[0],CAMERA_SIZE);
    //vcan_sendimg(img,sizeof(img));  //����ͼ����λ��
    //vcan_sendimg(imgbuff,CAMERA_SIZE);
    ImageProc(); 
    ovtStart();
}



 void ImageProc()
{
       DataReset();
       for(i=44,time=1;time>0;time--)// san41   si38 
       {     
                whiteNum++;
                for(j=40;j>3;j--)           //���Ե����
            {
                  if(img[i][j]-img[i][j-1] == EDGE )    //�Ӱ����䵽��  WHITE-BLACK
                  {
                
                    if(img[i][j-1]-img[i][j-3] ==0)
                    leftPos = (j-1);                    
                    leftEdgeflag=1;
                    break;
                  }
                  else
                  {
                     leftPos = (j-1);  //δ�������߽�
                     leftEdgeflag=0;
                     
                     continue;
                  }
                  
             }
                for(j=40;j<78;j++)           //�ұ�Ե����
            {     
                  whiteNum++;
                  if(img[i][j]-img[i][j+1] == EDGE )    //�Ӱ����䵽��  WHITE-BLACK
                  {
                    if(img[i][j+1]-img[i][j+3]==0)
                    rightPos = (j+1);                    
                    rightEdgeflag=1;
                    break;
                  }
                  else
                  {
                     rightPos = (j+1);  //δ�������߽�
                     rightEdgeflag=0;
                    
                     continue;
                  }
                  
             }

       }
         if (whiteNum-whiteNum_last>20)
    {
       whitejudge=1;
    }
       if(img[44][0]&&img[44][80]&&whitejudge)
       {
         
            for(j=3;j<80;j++)
            {
               if (img[i][j]==0)
               {
                ovtflag=1;
                break;
               }
            }
         
       }
       if (whiteNum_last-whiteNum>20)
    {
       whitejudge1=1;
    }
       
   
       centure = (leftPos + rightPos)/2;   //���嵱ǰλ��
       centureErr = (centure - CENTURE);  //����ƫ����

      
       SevPID.ActOutput = 0.9*centureErr + (1.0 - 0.9)*SevPID.ActOutput;
       servPWMErr = PID_LocPIDCalc(&SevPID);
       servPWMDuty=servMotorCenture+servPWMErr*1.35;
       if(servPWMDuty>1330){servPWMDuty=1330;}//1300
       if(servPWMDuty<1020){servPWMDuty=1020;}//1020
       
      
}
void DataReset()
{
  //uint32 i;
  leftPos = 0;
  rightPos = 0;
  centureErr = 0;
  centure = 0;
  turn = 0;
  GOAHEAD=0;
  int32 servPWMErr = 0;
 
 
  whitejudge=0;
  whiteNum_last=whiteNum;
  whiteNum=0;
  //ÿ�����
   ovtflag=0;//=1����
   leftEdgeflag = 1;      //��߽�ȷ�ϱ�־���ҵ���߽���λ
   rightEdgeflag = 1;

//  for(i=0;i<rowNum;i++)
//  {
//    left[i]=0;
//    right[i]=CAMERA_W-1;
//  }
}

void ovtStart()
{
  if(ovtflag==1)
  {
    for(i=ovtrowStart;i<ovtrowStart+10;i++)
    {
      for(j=0;j<80;j++)
      {
        if(img[i][j]==0)
          ovtright[i-ovtrowStart]=j;
       
      }
      for(j=80;j>0;j--)
      {
        if(img[i][j]==0)
          ovtleft[i-ovtrowStart]=j;
        
      }
      ovtCul();
      
    }
  //  DELAY_MS(300);
    
    
      if((ovtleftMax-ovtleftMin)<(ovtrightMax-ovtrightMin))
    {
      //if(leftEdgeflag&&rightEdgeflag)
      servPWMDuty=servMotorRight;
    //  ftm_pwm_duty(FTM2,FTM_CH0,servPWMDuty);
      servflag=1;
       DELAY_MS(280);
    }
    
    if((ovtleftMax-ovtleftMin)>(ovtrightMax-ovtrightMin))
    {
    //  if(leftEdgeflag&&rightEdgeflag)
      servPWMDuty=servMotorLeft;
    //  ftm_pwm_duty(FTM2,FTM_CH0,servPWMDuty);
      servflag=1;
       DELAY_MS(280);
    }
//    DELAY_MS(3000);
    if(servflag==1&&leftEdgeflag==1&&rightEdgeflag==1)
    {ovtflag=0;}
  }
  
}
void ovtCul()
{
//  ovtleftMax>ovtleft[i-ovtrowStart]? :ovtleftMax=ovtleft[i-ovtrowStart];
//  ovtleftMin<ovtleft[i-ovtrowStart]? :ovtleftMin=ovtleft[i-ovtrowStart];
//  ovtrightMax>ovtright[i-ovtrowStart]? :ovtrightMax=ovtleft[i-ovtrowStart];
//  ovtrightMin<ovtright[i-ovtrowStart]? :ovtrightMin=ovtleft[i-ovtrowStart];
  
  
  if(ovtleftMax<=ovtleft[i-ovtrowStart]){ovtleftMax=ovtleft[i-ovtrowStart];}  
  if(ovtleftMin>=ovtleft[i-ovtrowStart]){ovtleftMin=ovtleft[i-ovtrowStart];}
  if(ovtrightMax<=ovtright[i-ovtrowStart]){ovtrightMax=ovtright[i-ovtrowStart];}
  if(ovtrightMin>=ovtright[i-ovtrowStart]){ovtrightMin=ovtright[i-ovtrowStart];}
  
}


void StopMotor()
{    
//    if (whiteNum_last-whiteNum>20)
//    {
//       whitejudge1=1;
//    }
//    if(leftEdgeflag==1&&rightEdgeflag==1&&whitejudge1)
  //  PITcount++;
 //  if(PITcount>800)
   // {
      if((leftEdgeflag==1||rightEdgeflag==1)&&whitejudge1)
      {
        speed_out_set_str=0;
        speed_out_set=0;
        stopflag=1;
      }
 //  }
}



//          for(i=rowStart-1,time=3;time>0 ;time--)   //��������60�� ��ֹ1��δ��������Ե
//   {
//        for(j=40;j>1;j--)           //���Ե����
//        {
//              if(img[i][j]-img[i][j-1] == EDGE )    //�Ӱ����䵽��  WHITE-BLACK ����ԭ�������Ǻڼ��ס���tjx
//              {
//                leftPos = (j-1);
//                break;
//              }
//              else
//              {
//                 leftPos = (j-1);  //δ�������߽�
//                 continue;
//              }
//         }
//
//       //-------------���Ե����END------------------
//
//
//        //------------�ұ�Ե����------------------
//        for(j=40;j<CAMERA_W-1;j++)           //�ұ�Ե����
//        {
//              if(img[i][j-1]-img[i][j] == EDGE)   //�Ӱ����䵽��  BLACK-WHITE
//              {
//                rightPos = (j+1);
//                break;
//              }
//              else
//              {
//                rightPos = (j+1);  //δ�������߽�
//                continue;
//              }
//        }
//
//        //-------------�ұ�Ե����END-----------------
//    if( leftPos == 2 && rightPos == CAMERA_W-2)  //δ�ѵ��߽�  ͼ���쳣 ��������
//      continue;
//    else
//      break;       //�������˳�
//   }
//  //-------------��׼����������----------//
//
//  //----����δ��ʱ��ͼ������---------//
//    if(time > 0)
//    {
//      //-----��ʼ��Ե����--------
//      for(i=rowStart;i<rowEnd;i++)
//      {
//        //----���Ե����
//            for(j=leftPos;j>leftPos-proPix;j--)//���Ե�������
//            {
//                  if(j<=0)  break;   //����
//
//                  if(img[i][j+1]- img[i][j]== EDGE )
//                  {
//                    left[i-rowStart] = (j-1);
//                    turn--;
//                    leftEdgeFlag = 1;
//                    break;
//                  }
//                  else
//                  {left[i-rowStart] = 0;
//                    continue;
//                  }
//            }
//
//        if( leftEdgeFlag == 0 )
//        {
//              for(j=leftPos;j<leftPos+proPix;j++)//���Ե���Ҹ���
//              {
//                if(j>=CAMERA_W-1)  break;   //����
//
//                if(img[i][j+1]-img[i][j] == EDGE )
//                {
//                  left[i-rowStart] = (j+1);
//                  turn++;
//                  leftEdgeFlag = 1;
//                  break;
//                }
//                else
//                {
//                  continue;
//                }
//              }
//        }
//
//
//       //---�ұ�Ե����
//            for(j=rightPos;j>rightPos-proPix;j--)
//            {
//                  if(j<=0)  break;   //����
//
//                  if(img[i][j-1]-img[i][j] == EDGE )//�ұ�Ե��������
//                  {
//                    right[i-rowStart] = j;
//                    turn --;
//                    rightEdgeFlag = 1;
//                    break;
//                   }
//                  else
//                  {  right[i-rowStart] = 79;
//                    continue;
//                   }
//            }
//            if( rightEdgeFlag == 0)
//            {
//                  for(j=rightPos;j<rightPos+proPix;j++)   //�ұ�Ե��������
//                  {
//                        if(j>=CAMERA_W-1)  break;   //����
//
//                        if(img[i][j-1]-img[i][j] == EDGE )
//                        {
//                          right[i-rowStart] = (j+1);
//                          turn++;
//                          rightEdgeFlag = 1;
//                          break;
//                         }
//                        else
//                        {
//                          continue;
//                        }
//                  }
//            }
//
//            if( leftEdgeFlag == 0 )      //���δ�ҵ���Ե
//              //��������Ԥ�����Եֵ
//                {
//                  if(turn>0)        leftPos++;
//                  else if(turn<0)  leftPos--;
//                  else;
//                }
//
//            else
//              leftPos = left[i-rowStart];
//
//
//            if( rightEdgeFlag == 0 )    //�ұ�δ�ҵ���Ե
//              //��������Ԥ�����Եֵ
//                {
//                  if(turn>0)        rightPos++;
//                  else if(turn<0)   rightPos--;
//                  else;
//                }
//            else
//              rightPos = right[i-rowStart];
//
//
//            if(leftPos<0)
//              leftPos = 0;
//
//            if(rightPos>CAMERA_W-1)
//              rightPos = CAMERA_W-1;
//
//           leftEdgeFlag = 0;
//           rightEdgeFlag = 0;
//
//
//      }
//    }
//
//
//          //-----��ȡ��ǰ�����ƫ������
//          for(i=0;i<rowNum-1;i++)
//          {
//               leftPos += left[i];
//               rightPos += right[i];
//          }
//
////         leftPos=leftPos/rowNum;    //��Ȩ�����Եƽ��ֵ
////          rightPos=rightPos/rowNum;    //��Ȩ���ұ�Եƽ��ֵ
//          leftPos/=rowNum;    //��Ȩ����ASDF��Եƽ��ֵ
//          rightPos/=rowNum;   
 
//          for(i=rowStart-1,time=3;time>0 ;time--)   //��������60�� ��ֹ1��δ��������Ե
//   {
//        for(j=40;j>1;j--)           //���Ե����
//        {
//              if(img[i][j]-img[i][j-1] == EDGE )    //�Ӱ����䵽��  WHITE-BLACK ����ԭ�������Ǻڼ��ס���tjx
//              {
//                leftPos = (j-1);
//                break;
//              }
//              else
//              {
//                 leftPos = (j-1);  //δ�������߽�
//                 continue;
//              }
//         }
//
//       //-------------���Ե����END------------------
//
//
//        //------------�ұ�Ե����------------------
//        for(j=40;j<CAMERA_W-1;j++)           //�ұ�Ե����
//        {
//              if(img[i][j-1]-img[i][j] == EDGE)   //�Ӱ����䵽��  BLACK-WHITE
//              {
//                rightPos = (j+1);
//                break;
//              }
//              else
//              {
//                rightPos = (j+1);  //δ�������߽�
//                continue;
//              }
//        }
//
//        //-------------�ұ�Ե����END-----------------
//    if( leftPos == 2 && rightPos == CAMERA_W-2)  //δ�ѵ��߽�  ͼ���쳣 ��������
//      continue;
//    else
//      break;       //�������˳�
//   }
//  //-------------��׼����������----------//
//
//  //----����δ��ʱ��ͼ������---------//
//    if(time > 0)
//    {
//      //-----��ʼ��Ե����--------
//      for(i=rowStart;i<rowEnd;i++)
//      {
//        //----���Ե����
//            for(j=leftPos;j>leftPos-proPix;j--)//���Ե�������
//            {
//                  if(j<=0)  break;   //����
//
//                  if(img[i][j+1]- img[i][j]== EDGE )
//                  {
//                    left[i-rowStart] = (j-1);
//                    turn--;
//                    leftEdgeFlag = 1;
//                    break;
//                  }
//                  else
//                  {left[i-rowStart] = 0;
//                    continue;
//                  }
//            }
//
//        if( leftEdgeFlag == 0 )
//        {
//              for(j=leftPos;j<leftPos+proPix;j++)//���Ե���Ҹ���
//              {
//                if(j>=CAMERA_W-1)  break;   //����
//
//                if(img[i][j+1]-img[i][j] == EDGE )
//                {
//                  left[i-rowStart] = (j+1);
//                  turn++;
//                  leftEdgeFlag = 1;
//                  break;
//                }
//                else
//                {
//                  continue;
//                }
//              }
//        }
//
//
//       //---�ұ�Ե����
//            for(j=rightPos;j>rightPos-proPix;j--)
//            {
//                  if(j<=0)  break;   //����
//
//                  if(img[i][j-1]-img[i][j] == EDGE )//�ұ�Ե��������
//                  {
//                    right[i-rowStart] = j;
//                    turn --;
//                    rightEdgeFlag = 1;
//                    break;
//                   }
//                  else
//                  {  right[i-rowStart] = 79;
//                    continue;
//                   }
//            }
//            if( rightEdgeFlag == 0)
//            {
//                  for(j=rightPos;j<rightPos+proPix;j++)   //�ұ�Ե��������
//                  {
//                        if(j>=CAMERA_W-1)  break;   //����
//
//                        if(img[i][j-1]-img[i][j] == EDGE )
//                        {
//                          right[i-rowStart] = (j+1);
//                          turn++;
//                          rightEdgeFlag = 1;
//                          break;
//                         }
//                        else
//                        {
//                          continue;
//                        }
//                  }
//            }
//
//            if( leftEdgeFlag == 0 )      //���δ�ҵ���Ե
//              //��������Ԥ�����Եֵ
//                {
//                  if(turn>0)        leftPos++;
//                  else if(turn<0)  leftPos--;
//                  else;
//                }
//
//            else
//              leftPos = left[i-rowStart];
//
//
//            if( rightEdgeFlag == 0 )    //�ұ�δ�ҵ���Ե
//              //��������Ԥ�����Եֵ
//                {
//                  if(turn>0)        rightPos++;
//                  else if(turn<0)   rightPos--;
//                  else;
//                }
//            else
//              rightPos = right[i-rowStart];
//
//
//            if(leftPos<0)
//              leftPos = 0;
//
//            if(rightPos>CAMERA_W-1)
//              rightPos = CAMERA_W-1;
//
//           leftEdgeFlag = 0;
//           rightEdgeFlag = 0;
//
//
//      }
//    }
//
//
//          //-----��ȡ��ǰ�����ƫ������
//          for(i=0;i<rowNum-1;i++)
//          {
//               leftPos += left[i];
//               rightPos += right[i];
//          }
//
////         leftPos=leftPos/rowNum;    //��Ȩ�����Եƽ��ֵ
////          rightPos=rightPos/rowNum;    //��Ȩ���ұ�Եƽ��ֵ
//          leftPos/=rowNum;    //��Ȩ����ASDF��Եƽ��ֵ
//          rightPos/=rowNum;   
 

