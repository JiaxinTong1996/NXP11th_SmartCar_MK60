#ifndef __PID_H__
#define __PID_H__
#endif
typedef struct tagPIDConst
{
	float PropRto;
	float IntgRto;
	float DiffRto;
	float ErrPre;
	float ErrLast; //last error
	float ErrCur; //current error
	float ErrSum;  //error summation
	float ErrDiff;
	float ActOutput; //actual output
	float ExcOutput; //output excepted
	float u;
}PIDConst;

void InitPID(volatile PIDConst * PID, float propRto, float intgRto, float diffRto);
void SetPIDExcOutput(volatile PIDConst *PID, float excVal);
void SetPIDActOutput(volatile PIDConst *PID, float actVal);
float PID_LocPIDCalc(volatile PIDConst * PID);
float PID_LocPIDBBCal(volatile PIDConst * PID);
float PID_IncPIDCalcMotor(volatile PIDConst * PID);