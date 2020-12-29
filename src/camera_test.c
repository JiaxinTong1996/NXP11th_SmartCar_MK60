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
//#define ThreadHold 140    //赛道黑白分隔阈值，调整方法：静态放置车体若赛道轮廓不明显都为白色则将该值调大，否则调小，直至轮廓清楚
#define rowStart 30       //图像处理起始行数
#define rowEnd   33     //图像处理终止行数
#define rowNum   rowEnd-rowStart  //有效处理行数，原始采集行数为ROW(在Include.h中定义)
#define proPix   5        //边缘跟踪搜索边界有效像素
#define CENTURE  39      //中心值
#define servMotorCenture  1175    //san 1175  si 1310
#define servMotorLeft    1330    //san 1330  si 1430
#define servMotorRight    1010     //san 1020  si 1190  1090
//#define servPram          9       //舵机转角比例系数，调整方法：若舵机角度跟随慢增大该值，若舵机角度摆幅很大减小该值
uint8 servPram;
#if 0
#define MOTOR_HZ    (50)
#else
#define MOTOR_HZ    (20*1000)
#endif


  #define BLACK     0     
  #define WHITE    255


#define   EDGE  WHITE-BLACK  //边界值大小
//#define ROW 60
//#define COL 80
//uint8 tempBuffer1[ROW][COL]={0};      //通过DMA采集(在isr.c中行中断中触发DMA采集)二维图像存储于tempBuffer1数组中
uint32 left[rowNum] = {0};         //存放有效处理行中双线左边缘的列坐标，初始化为0
uint32 right[rowNum] = {CAMERA_W-1};     //存放有效处理行中双线右缘的列坐标，初始化为COL-1
 volatile uint32 servPWMDuty;
int32 leftPos = 0;          //左边界位置[有符号数]
int32 rightPos = 0;          //右边界位置[有符号数]
uint8 centure = 0;         //计算出的当前车体的中心位置[无符号数]
int32 centureErr = 0;      //计算出的当前车体位置偏差 [有符号数]
uint32 turn = 0;            //边界走势  正为向左偏转  负为向右偏转


int32 servPWMErr = 0;

extern volatile IMG_STATUS_e      ov7725_eagle_img_flag;
extern volatile PIDConst SevPID;
uint8 time;
extern int16 speed_out_set;//210
extern int16 speed_out_set_str;//300

volatile uint32 rowCnt = 0 ;//行计数

void ImageProc();
void DataReset();
uint8 imgbuff[CAMERA_SIZE];    //定义存储接收图像的数组
//uint8 img[CAMERA_W*CAMERA_H]; //由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
uint8 img[CAMERA_H][CAMERA_W];
                               //假如需要二维数组，只需要改成 uint8 img[CAMERA_H][CAMERA_W];
                               //imgbuff是采集的缓冲区，img是解压后的缓冲区。imgbuff用于采集图像，img用于图像处理.

/************超车停车**************/
#define Numgap  whiteNum-blackNum//黑白数量差
#define ovtrowStart 36
uint32 i,j;
uint8 ovtflag=0;//=1超车
uint8 leftEdgeflag=1;      //左边界确认kong标志，找到后边界置位
uint8 rightEdgeflag=1;      //右边界确认kong标志，找到后边界置位
uint8 servflag=0;//转弯后置1
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
    camera_get_img();//摄像头获取图像
    img_extract(&img[0][0], &imgbuff[0],CAMERA_SIZE);
    //vcan_sendimg(img,sizeof(img));  //发送图像到上位机
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
                for(j=40;j>3;j--)           //左边缘搜索
            {
                  if(img[i][j]-img[i][j-1] == EDGE )    //从白跳变到黑  WHITE-BLACK
                  {
                
                    if(img[i][j-1]-img[i][j-3] ==0)
                    leftPos = (j-1);                    
                    leftEdgeflag=1;
                    break;
                  }
                  else
                  {
                     leftPos = (j-1);  //未搜索到边界
                     leftEdgeflag=0;
                     
                     continue;
                  }
                  
             }
                for(j=40;j<78;j++)           //右边缘搜索
            {     
                  whiteNum++;
                  if(img[i][j]-img[i][j+1] == EDGE )    //从白跳变到黑  WHITE-BLACK
                  {
                    if(img[i][j+1]-img[i][j+3]==0)
                    rightPos = (j+1);                    
                    rightEdgeflag=1;
                    break;
                  }
                  else
                  {
                     rightPos = (j+1);  //未搜索到边界
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
       
   
       centure = (leftPos + rightPos)/2;   //求车体当前位置
       centureErr = (centure - CENTURE);  //求车体偏移量

      
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
  //每次清空
   ovtflag=0;//=1超车
   leftEdgeflag = 1;      //左边界确认标志，找到后边界置位
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



//          for(i=rowStart-1,time=3;time>0 ;time--)   //至多搜索60次 防止1次未搜索到边缘
//   {
//        for(j=40;j>1;j--)           //左边缘搜索
//        {
//              if(img[i][j]-img[i][j-1] == EDGE )    //从白跳变到黑  WHITE-BLACK 这里原本检测的是黑减白。。tjx
//              {
//                leftPos = (j-1);
//                break;
//              }
//              else
//              {
//                 leftPos = (j-1);  //未搜索到边界
//                 continue;
//              }
//         }
//
//       //-------------左边缘搜索END------------------
//
//
//        //------------右边缘搜索------------------
//        for(j=40;j<CAMERA_W-1;j++)           //右边缘搜索
//        {
//              if(img[i][j-1]-img[i][j] == EDGE)   //从白跳变到黑  BLACK-WHITE
//              {
//                rightPos = (j+1);
//                break;
//              }
//              else
//              {
//                rightPos = (j+1);  //未搜索到边界
//                continue;
//              }
//        }
//
//        //-------------右边缘搜索END-----------------
//    if( leftPos == 2 && rightPos == CAMERA_W-2)  //未搜到边界  图像异常 继续搜索
//      continue;
//    else
//      break;       //搜索到退出
//   }
//  //-------------基准点搜索结束----------//
//
//  //----搜索未超时，图像正常---------//
//    if(time > 0)
//    {
//      //-----开始边缘跟踪--------
//      for(i=rowStart;i<rowEnd;i++)
//      {
//        //----左边缘跟踪
//            for(j=leftPos;j>leftPos-proPix;j--)//左边缘向左跟踪
//            {
//                  if(j<=0)  break;   //断言
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
//              for(j=leftPos;j<leftPos+proPix;j++)//左边缘向右跟踪
//              {
//                if(j>=CAMERA_W-1)  break;   //断言
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
//       //---右边缘跟踪
//            for(j=rightPos;j>rightPos-proPix;j--)
//            {
//                  if(j<=0)  break;   //断言
//
//                  if(img[i][j-1]-img[i][j] == EDGE )//右边缘向左搜索
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
//                  for(j=rightPos;j<rightPos+proPix;j++)   //右边缘向右搜索
//                  {
//                        if(j>=CAMERA_W-1)  break;   //断言
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
//            if( leftEdgeFlag == 0 )      //左边未找到边缘
//              //根据趋势预测左边缘值
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
//            if( rightEdgeFlag == 0 )    //右边未找到边缘
//              //根据趋势预测左边缘值
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
//          //-----提取当前车体的偏移量吧
//          for(i=0;i<rowNum-1;i++)
//          {
//               leftPos += left[i];
//               rightPos += right[i];
//          }
//
////         leftPos=leftPos/rowNum;    //加权求左边缘平均值
////          rightPos=rightPos/rowNum;    //加权求右边缘平均值
//          leftPos/=rowNum;    //加权求左ASDF边缘平均值
//          rightPos/=rowNum;   
 
//          for(i=rowStart-1,time=3;time>0 ;time--)   //至多搜索60次 防止1次未搜索到边缘
//   {
//        for(j=40;j>1;j--)           //左边缘搜索
//        {
//              if(img[i][j]-img[i][j-1] == EDGE )    //从白跳变到黑  WHITE-BLACK 这里原本检测的是黑减白。。tjx
//              {
//                leftPos = (j-1);
//                break;
//              }
//              else
//              {
//                 leftPos = (j-1);  //未搜索到边界
//                 continue;
//              }
//         }
//
//       //-------------左边缘搜索END------------------
//
//
//        //------------右边缘搜索------------------
//        for(j=40;j<CAMERA_W-1;j++)           //右边缘搜索
//        {
//              if(img[i][j-1]-img[i][j] == EDGE)   //从白跳变到黑  BLACK-WHITE
//              {
//                rightPos = (j+1);
//                break;
//              }
//              else
//              {
//                rightPos = (j+1);  //未搜索到边界
//                continue;
//              }
//        }
//
//        //-------------右边缘搜索END-----------------
//    if( leftPos == 2 && rightPos == CAMERA_W-2)  //未搜到边界  图像异常 继续搜索
//      continue;
//    else
//      break;       //搜索到退出
//   }
//  //-------------基准点搜索结束----------//
//
//  //----搜索未超时，图像正常---------//
//    if(time > 0)
//    {
//      //-----开始边缘跟踪--------
//      for(i=rowStart;i<rowEnd;i++)
//      {
//        //----左边缘跟踪
//            for(j=leftPos;j>leftPos-proPix;j--)//左边缘向左跟踪
//            {
//                  if(j<=0)  break;   //断言
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
//              for(j=leftPos;j<leftPos+proPix;j++)//左边缘向右跟踪
//              {
//                if(j>=CAMERA_W-1)  break;   //断言
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
//       //---右边缘跟踪
//            for(j=rightPos;j>rightPos-proPix;j--)
//            {
//                  if(j<=0)  break;   //断言
//
//                  if(img[i][j-1]-img[i][j] == EDGE )//右边缘向左搜索
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
//                  for(j=rightPos;j<rightPos+proPix;j++)   //右边缘向右搜索
//                  {
//                        if(j>=CAMERA_W-1)  break;   //断言
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
//            if( leftEdgeFlag == 0 )      //左边未找到边缘
//              //根据趋势预测左边缘值
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
//            if( rightEdgeFlag == 0 )    //右边未找到边缘
//              //根据趋势预测左边缘值
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
//          //-----提取当前车体的偏移量吧
//          for(i=0;i<rowNum-1;i++)
//          {
//               leftPos += left[i];
//               rightPos += right[i];
//          }
//
////         leftPos=leftPos/rowNum;    //加权求左边缘平均值
////          rightPos=rightPos/rowNum;    //加权求右边缘平均值
//          leftPos/=rowNum;    //加权求左ASDF边缘平均值
//          rightPos/=rowNum;   
 

