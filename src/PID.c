#include "common.h"
#include "include.h"
#include "PID.h"
volatile PIDConst SevPID;  

void InitPID(volatile PIDConst * PID, float propRto, float intgRto, float diffRto)
{
	PID->ActOutput = 0;
	PID->ExcOutput = 0;
	PID->u = 0;
	PID->PropRto = propRto;
	PID->IntgRto = intgRto;
	PID->DiffRto = diffRto;
	PID->ErrPre = 0;
	PID->ErrCur = 0;
	PID->ErrLast = 0;
	PID->ErrSum = 0;
	PID->ErrDiff = 0;
}
void SetPIDExcOutput(volatile PIDConst *PID, float excVal)
{
	PID->ExcOutput = excVal;
}

void SetPIDActOutput(volatile PIDConst *PID, float actVal)
{
	PID->ActOutput = actVal;
}

float PID_LocPIDCalc(volatile PIDConst * PID)
{
	float u;
	PID->ErrCur = PID->ExcOutput - PID->ActOutput;
	PID->ErrDiff = PID->ErrCur - PID->ErrLast;
	PID->ErrSum += PID->ErrCur;
	u = PID->ErrCur * PID->PropRto + PID->ErrSum * PID->IntgRto + PID->ErrDiff * PID->DiffRto;
	PID->ErrLast = PID->ErrCur;
	return u;	
}

//float PID_LocPIDBBCal(volatile PIDConst * PID)
//{
//	float u;
//	PID->ErrCur = PID->ExcOutput - PID->ActOutput;
//	if(PID->ErrCur > 200)
//	{
//		PID->ErrSum = 0;
//		return 9800;
//	}
//	else if(PID->ErrCur < -200)
//	{
//		PID->ErrSum = 0;
//		return -8000;
//	}
//		
//	else
//	{
//		PID->ErrDiff = PID->ErrCur - PID->ErrLast;
//		PID->ErrSum += PID->ErrCur;
//		u = PID->ErrCur * PID->PropRto + PID->ErrSum * PID->IntgRto + PID->ErrDiff * PID->DiffRto;
//		PID->ErrLast = PID->ErrCur;
//		return u;
//	}	
//}
//float PID_IncPIDBBCal(volatile PIDConst * PID)
//{
//	float KP, KI, KD;
//	PID->ErrCur = PID->ExcOutput - PID->ActOutput;
//	if(PID->ErrCur > 200)
//		PID->u = 9800;
//	else if(PID->ErrCur < -200)		
//		PID->u = -8000;
//	else
//	{
//		KP = PID->PropRto * (PID->ErrCur - PID->ErrLast);
//		KI = PID->IntgRto * PID->ErrCur;
//		KD = PID->DiffRto * (PID->ErrCur - 2.0*PID->ErrLast + PID->ErrPre);
//		PID->u += KP + KI + KD;
//		if(PID->u > 9800)
//			PID->u = 9800;
//		else if(PID->u < -9800)
//			PID->u = -9800;
//	}
//	PID->ErrPre = PID->ErrLast;
//	PID->ErrLast = PID->ErrCur;
//	
//	return PID->u;
//}










