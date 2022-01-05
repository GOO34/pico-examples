#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define FLAG_VALUE 123
#define ENDSTDIN 255
#define CR 13

#define PathSplitBaseDis_mm 0.005
#define DegToRad (M_PI / 180.0)
#define RadToDeg (180.0 / M_PI)

enum SCARA_Axis//SCARA軸名稱
{
	J1,
	J2,
	J3,
	J4,
	AxisNumber
}eSCARA = AxisNumber;
enum CoordinatePointPosition//座標點位置
{
	PreviousPosition,
	CurrentPosition,
	TargetPosition,
	OffsetPostiton,
	TempPostiton,
	PositionNumber
}ePointPosition = PositionNumber;
enum PointName//點名稱
{
	_X,
	_Y,
	_Z,
	_Angle,
	PointNameNumber
}ePointName = PointNameNumber;
enum RangePoint//移動範圍
{
	Angle_Min,
	Angle_Max,
	Distance_Min,
	Distance_Max,
	RangePointNumber
}eRangePoint = RangePointNumber;
enum AxisPlusValue//移動資料
{
	PreviousPlus,
	CurrentPlus,
	TargetPlus,
	DifferencePlus,
	AxisPlusNumber
}eAxisPlusValue = AxisPlusNumber;
enum MoveNeedTimeAndSpeed//移動需求時間與速度
{
	Accelerate_sec,
	Decelerate_sec,
	AccMaxMoveDistance,
	DecMaxMoveDistance,
	TotalMoveTime_sec,
	MaxSpeed,// mm/min or rad/min or RPM
	MoveNeedTimeAndSpeedNumber
}eMoveNeedTimeAndSpeed = MoveNeedTimeAndSpeedNumber;
enum SpeedFormat//速度格式
{
	MaxOutFrequency,
	MaxOutSpeed,
	SpeedFormatNumber
}eSpeedFormat = SpeedFormatNumber;
enum P2PInfo//點到點資訊
{
	DeltaDistance_X,
	DeltaDistance_Y,
	DeltaDistance_Z,
	Distance_XY,
	Distance_Z,
	Distance_J3,
	Distance,
	P2PSpeed_XY,
	P2PSpeed_Z,
	P2PSpeed_J3,
	P2PSpeed,
	Theta,
	Phi,
	SplitSize,
	SplitBaseDistance,
	P2PInfoNumber
}eP2PInfo = P2PInfoNumber;
enum PathType//路徑型態
{
	Path_XYZ,
	Path_XY,
	Path_Z,
	Path_Theta,
	PathTypeNumber
}ePathType = PathTypeNumber;
enum PathPlus//路徑Plus
{
	Absolute_Position,
	Relative_Position,
	AxisSpeed,
	Direction,
	PathPlusNumber
}ePathPlus = PathPlusNumber;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

char Msg[256];
double CoordinatePoint[PositionNumber][PointNameNumber];//座標點
double BufCoordinatePoint[PositionNumber][PointNameNumber];//暫存座標點
double AxisPlusData[AxisNumber][AxisPlusNumber];//軸移動資料
double MaxRPS;//最大轉速
double pulse_per_round;//每轉Plus數
double AccelerateTime_sec;//加速度時間
double DecelerateTime_sec;//減速度時間
double RunSpeedPercentage;//運行速度百分比
double MovableRange[AxisNumber][RangePointNumber];//移動範圍
double ScrewPitch[AxisNumber];//各軸Pitch mm
double ReductionRation[AxisNumber];//各軸減速比
double AxisResolution[AxisNumber];//軸輸出解析(Angle/plus, mm/plus)
int SlowestSpeedAxisNum_XYZ = -1;//最慢軸編號
double SlowestSpeed_XYZ = 0;//最慢速度
double OutputSpeed[AxisNumber][SpeedFormatNumber];//各軸最大輸出頻率 plus/sec, rev/sec, mm/sec
double MaxOutputSpeed[AxisNumber];//經減速機後最大速度(rev/sec, mm/min)
double P2P[P2PInfoNumber];//點到點資訊
double BufP2P[P2PInfoNumber];//暫存點到點資訊
double MoveData[PathTypeNumber][MoveNeedTimeAndSpeedNumber];//移動需求時間(Sec)與速度(rev/sec)
double TempMoveData[MoveNeedTimeAndSpeedNumber];//動態移動需求時間(Sec)與速度(rev/sec)

// double MoveSpeed_J1;// J1 Plus/sec
// double MoveSpeed_J2;// J2 Plus/sec
// double MoveSpeed_J3;// J3 Plus/sec
// double MoveSpeed_J4;// J4 Plus/sec
// double MoveDirection_J1;// J1 Direction
// double MoveDirection_J2;// J2 Direction
// double MoveDirection_J3;// J3 Direction
// double MoveDirection_J4;// J4 Direction
// double MoveAbsolutePosition_J1[1];// J1 AbsolutePosition
// double MoveAbsolutePosition_J2[1];// J2 AbsolutePosition
// double MoveAbsolutePosition_J3[1];// J3 AbsolutePosition
// double MoveAbsolutePosition_J4[1];// J4 AbsolutePosition
// double MoveRelativePosition_J1[1];// J1 RelativePosition
// double MoveRelativePosition_J2[1];// J2 RelativePosition
// double MoveRelativePosition_J3[1];// J3 RelativePosition
// double MoveRelativePosition_J4[1];// J4 RelativePosition






double TempCoordinatePoint[PositionNumber][PointNameNumber];//動態座標點
double ProgressiveDistance[PositionNumber];//動態座標點
double AxisPosition[AxisNumber][PositionNumber];//軸位置
double Angle_Radian[PositionNumber][AxisNumber];//角度_徑度

int SlowestSpeedAxisNum = -1;//最慢軸編號
double SlowestSpeed = 0;//最慢速度
double MaxSpeed_1 = 0;//最大移動速度
double AccMaxDis = 0;//加速度最大移動距離
double DecMaxDis = 0;//減速度最大移動距離

int command;
int count;
double _mm_per_pulse = 0.01;

int ActionInt = 0;

#pragma region Custom//自定義
int ReceiveChar()//接收字符
{
	int buffer_length = 255;
	char ReceiveChar;
  	int i = 0;
	memset(Msg, 0, 256);

	ReceiveChar = getchar_timeout_us(0);
	while(ReceiveChar != buffer_length && ReceiveChar != PICO_ERROR_TIMEOUT)
	{
		Msg[i++] = ReceiveChar;
		ReceiveChar = getchar_timeout_us(0);
	}
	Msg[i++] = '\0';

	if(strlen(Msg) > 0) printf("[Receive Char] %s\n", Msg);
	
	return ReceiveChar;
}
#pragma region Print
void PrintAxisMovePlus(int AxisNum, double Dir, double Dis, double Speed, double Dis_Plus, double Speed_Plus)
{
	switch (AxisNum)
	{
		case 0: printf("\t[J1][Dir: %d][Pos: %.6f rad][Speed: %.6f rev/sec]\t[Pos_plus: %6d plus][Speed_plus: %6d plus/sec]\n", (int)Dir, Dis, Speed, (int)Dis_Plus, (int)Speed_Plus); break;
		case 1: printf("\t[J2][Dir: %d][Pos: %.6f mm][Speed: %.6f mm/sec]\t[Pos_plus: %6d plus][Speed_plus: %6d plus/sec]\n", (int)Dir, Dis, Speed, (int)Dis_Plus, (int)Speed_Plus); break;
		case 2: printf("\t[J3][Dir: %d][Pos: %.6f rad][Speed: %.6f rev/sec]\t[Pos_plus: %6d plus][Speed_plus: %6d plus/sec]\n", (int)Dir, Dis, Speed, (int)Dis_Plus, (int)Speed_Plus); break;
		case 3: printf("\t[J4][Dir: %d][Pos: %.6f mm][Speed: %.6f mm/sec]\t[Pos_plus: %6d plus][Speed_plus: %6d plus/sec]\n", (int)Dir, Dis, Speed, (int)Dis_Plus, (int)Speed_Plus); break;
		default: break;
	}
}
void PrintMoveData()
{
	for (size_t i = 0; i < PathTypeNumber; i++)
	{
		ePathType = i;

		switch (ePathType)
		{
			case Path_XYZ: 
				printf("\tPath_XYZ Move Data [速度: %.4f, 距離: %.6f mm] >>\n", P2P[P2PSpeed], P2P[Distance]); 
				break;
			case Path_XY: 
				printf("\tPath_XY Move Data [速度: %.4f, 距離: %.6f mm] >>\n", P2P[P2PSpeed_XY], P2P[Distance_XY]); 
				break;
			case Path_Z: 
				printf("\tPath_Z Move Data [速度: %.4f, 距離: %.6f mm] >>\n", P2P[P2PSpeed_Z], P2P[Distance_Z]); 
				break;
			case Path_Theta: 
				printf("\tPath_Theta Move Data [速度: %.4f, 距離: %.6f Radian] >>\n", P2P[P2PSpeed_J3], P2P[Distance_J3]); 
				break;
			default: 
				printf("\tMove Data [速度: %.4f, 距離: %.6f] >>\n", P2P[P2PSpeed], P2P[Distance]); 
				break;
		}

		printf("\t\t加速使用時間: %.6f sec, 移動距離: %.6f\n", MoveData[ePathType][Accelerate_sec], MoveData[ePathType][AccMaxMoveDistance]);
		printf("\t\t減速使用時間: %.6f sec, 移動距離: %.6f\n", MoveData[ePathType][Decelerate_sec], MoveData[ePathType][DecMaxMoveDistance]);
		printf("\t\t總移動時間: %.6f sec\n", MoveData[ePathType][TotalMoveTime_sec]);
		printf("\t\t最大移動速度: %.4f\n", MoveData[ePathType][MaxSpeed]);
	}
}
void PrintP2P()
{
	printf("\tP2P Info >>\n");
	for (size_t i = 0; i < P2PInfoNumber; i++) 
	{
		eP2PInfo = i;
		switch (eP2PInfo)
		{
			case Distance_XY: printf("\t\tDistance XY: %.4f mm\n", P2P[eP2PInfo]); break;
			case Distance_Z: printf("\t\tDistance Z: %.4f mm\n", P2P[eP2PInfo]); break;
			case Distance_J3: printf("\t\tDistance J3: %.6f Radian, %.6f Degree\n", P2P[eP2PInfo], P2P[eP2PInfo] * RadToDeg); break;
			case Distance: printf("\t\tDistance: %.4f mm\n", P2P[eP2PInfo]); break;
			case P2PSpeed_XY: printf("\t\tP2PSpeed XY: %.2f mm/sec\n", P2P[eP2PInfo]); break;
			case P2PSpeed_Z: printf("\t\tP2PSpeed Z: %.2f mm/sec\n", P2P[eP2PInfo]); break;
			case P2PSpeed_J3: printf("\t\tP2PSpeed J3: %.2f rev/sec\n", P2P[eP2PInfo]); break;
			case P2PSpeed: printf("\t\tP2PSpeed: %.2f mm/sec\n", P2P[eP2PInfo]); break;
			case Theta: printf("\t\t極角(Theta): %.6f Radian, %.6f Degree\n", P2P[eP2PInfo], P2P[eP2PInfo] * RadToDeg); break;
			case Phi: printf("\t\t方位角(Phi): %.6f Radian, %.6f Degree\n", P2P[eP2PInfo], P2P[eP2PInfo] * RadToDeg); break;
			case SplitSize: printf("\t\t路徑分割數: %d\n", (int)P2P[eP2PInfo]); break;
			case SplitBaseDistance: printf("\t\t分割步進距離: %.4f mm\n", P2P[eP2PInfo]); break;
			default: break;
		}
	}
}
void PrintCoordinatePoint()
{
	printf("\tCoordinate Point [X, Y, Z, Theta] >>\n");
	for (size_t i = 0; i < PositionNumber; i++) 
	{
		ePointPosition = i;
		printf("\t\t[%d] = [", ePointPosition);
		for (size_t j = 0; j < PointNameNumber; j++)
		{
			ePointName = j;
			if(j != 0) printf(", ");
			printf("%.4f", CoordinatePoint[ePointPosition][ePointName]);
		}
		printf("]\n");
	}
}
void PrintMaxOutputSpeed()
{
	printf("\t\tMax Output Speed [J1, J2, J3, J4] >> [");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		if (i != 0) printf(", ");
		printf("%.2f", OutputSpeed[i][MaxOutSpeed]);
		eSCARA = i;
		switch (eSCARA)
		{
			case J1: printf(" rev/sec"); break;
			case J2: printf(" mm/sec"); break;
			case J3: printf(" rev/sec"); break;
			case J4: printf(" mm/sec"); break;
			default: break;
		}
	}
	printf("]\n");
}
void PrintMaxOutputFrequency()
{
	printf("\t\tMax Output Frequency [J1, J2, J3, J4] >> [");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		if (i != 0) printf(", ");
		printf("%d plus/sec", (int)OutputSpeed[i][MaxOutFrequency]);
	}
	printf("]\n");
}
void PrintAxisResolution()
{
	printf("\t\tAxis Resolution [J1, J2, J3, J4] >> [");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		if (i != 0) printf(", ");
		eSCARA = i;
		if (eSCARA == J1 || eSCARA == J3)
			printf("%.6f Radain/plus", AxisResolution[i]);
		else
			printf("%.4f mm/plus", AxisResolution[i]);
	}
	printf("]\n");
}
void PrintReductionRation()
{
	printf("\t\tReduction Ration [J1, J2, J3, J4] >> [");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		if (i != 0) printf(", ");
		printf("%.6f", ReductionRation[i]);
	}
	printf("]\n");
}
void PrintScrewPitch()
{
	printf("\t\tScrew Pitch [J1, J2, J3, J4] >> [");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		if (i != 0) printf(", ");
		printf("%.6f mm", ScrewPitch[i]);
	}
	printf("]\n");
}
#pragma endregion
void InitMaxOutputSpeed()//初始化經減速機後最大速度
{
	printf("\tInit Max Output Speed...");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		eSCARA = i;
		MaxOutputSpeed[i] = (MaxRPS / ReductionRation[i]) * ScrewPitch[i];
		OutputSpeed[i][MaxOutSpeed] = MaxOutputSpeed[i];

		switch (eSCARA) //找出XYZ最慢移動速度
		{
			case J2:
				SlowestSpeed_XYZ = MaxOutputSpeed[eSCARA];
				SlowestSpeedAxisNum_XYZ = eSCARA;
				break;
			case J4:
				if (MaxOutputSpeed[eSCARA] < SlowestSpeed_XYZ)
				{
					SlowestSpeed_XYZ = MaxOutputSpeed[eSCARA];
					SlowestSpeedAxisNum_XYZ = eSCARA;
				}
				break;
			default: break;
		}
	}
	printf("Finish!!\n");
}
void InitMaxOutputFrequency()//初始化最大輸出頻率
{
	printf("\tInit Max Output Frequency...");
	for (size_t i = 0; i < AxisNumber; i++)
		OutputSpeed[i][MaxOutFrequency] = MaxRPS * pulse_per_round;
	printf("Finish!!\n");
}
void InitAxisResolution()//初始化軸解析度
{
	printf("\tInit Axis Resolution...");
	for (size_t i = J1; i < AxisNumber; i++)
	{
		eSCARA = i;
		double MotorOutResolution = pulse_per_round * ReductionRation[eSCARA];
		
		if (eSCARA == J1 || eSCARA == J3)
			AxisResolution[i] = (2 * M_PI) / MotorOutResolution;
		else
			AxisResolution[i] = ScrewPitch[i] / MotorOutResolution;
	}
	printf("Finish!!\n");
}
void InitReductionRation()//初始化減速比
{
	printf("\tInit Reduction Ration..");
	int Reducer_In[AxisNumber] = {100, 1, 70, 1};
	int Reducer_Out[AxisNumber] = {1, 1, 1, 1};
	int GearTeeth_In[AxisNumber] = {1, 36, 12, 14};
	int GearTeeth_Out[AxisNumber] = {1, 24, 22, 22};

	for (size_t i = J1; i < AxisNumber; i++)
		ReductionRation[i] = ((double)Reducer_In[i] / (double)Reducer_Out[i]) * ((double)GearTeeth_Out[i] / (double)GearTeeth_In[i]);
	
	printf("Finish!!\n");
}
void InitScrewPitch()//初始化螺桿Pitch(mm)
{
	printf("\tInit Screw Pitch...");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		eSCARA = i;
		switch (eSCARA)
		{
			case J2: ScrewPitch[J2] = 10.0; break;
			case J4: ScrewPitch[J4] = 25.0; break;
			default: ScrewPitch[eSCARA] = 1.0; break;//無螺桿值為1.0
		}
	}
	printf("Finish!!\n");
}
void InitRangePoint()//初始化移動範圍
{
	printf("\tInit Axis Range...");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		eSCARA = i;
		switch (eSCARA)
		{
			case J1: 
				MovableRange[eSCARA][Angle_Min] = -180.0 * DegToRad;
				MovableRange[eSCARA][Angle_Max] = 180.0 * DegToRad;
				MovableRange[eSCARA][Distance_Min] = 473.0;
				MovableRange[eSCARA][Distance_Max] = 473.0;
				break;
			case J2: 
				MovableRange[eSCARA][Angle_Min] = 0.0 * DegToRad;
				MovableRange[eSCARA][Angle_Max] = 0.0 * DegToRad;
				MovableRange[eSCARA][Distance_Min] = 0.0;
				MovableRange[eSCARA][Distance_Max] = 300.0;
				break;
			case J3: 
				MovableRange[eSCARA][Angle_Min] = -180.0 * DegToRad;
				MovableRange[eSCARA][Angle_Max] = 180.0 * DegToRad;
				MovableRange[eSCARA][Distance_Min] = 0.0;
				MovableRange[eSCARA][Distance_Max] = 0.0;
				break;
			case J4: 
				MovableRange[eSCARA][Angle_Min] = 0.0 * DegToRad;
				MovableRange[eSCARA][Angle_Max] = 0.0 * DegToRad;
				MovableRange[eSCARA][Distance_Min] = 0.0;
				MovableRange[eSCARA][Distance_Max] = 400.0;
				break;
			default: break;
		}
	}
	printf("Finish!!\n");
}
void InitMotorParam()//初始化馬達參數
{
	MaxRPS = 3000.0 / 60;// 3000 RPM 轉 rev/sec
	pulse_per_round = 3000.0;
	AccelerateTime_sec = 0.250;
	DecelerateTime_sec = 0.250;
	RunSpeedPercentage = 100.0;
}
void InitAxisPlusData()//初始化軸移動資料
{
	printf("\tInit Axis Plus Data...");
	for (size_t i = 0; i < AxisNumber; i++)
		for (size_t j = 0; j < AxisPlusNumber; j++)
			AxisPlusData[i][j] = 0.0;
	printf("Finish!!\n");
}
void InitP2P()//初始化P2P資訊
{
	printf("\tInit P2P Info...");
	for (size_t i = 0; i < P2PInfoNumber; i++)
		P2P[i] = 0.0;
	printf("Finish!!\n");
}
void InitCoordinatePoint()//初始化座標點
{
	printf("\tInit Coordinate Point...");
	for (size_t i = 0; i < PositionNumber; i++)
		for (size_t j = 0; j < PointNameNumber; j++)
			CoordinatePoint[i][j] = 0.000;
	printf("Finish!!\n");
}
void Init_AllParameter()//初始化全部參數
{
	printf("Init Parameter >>\n");
	InitCoordinatePoint();
	InitP2P();
	InitAxisPlusData();
	InitMotorParam();
	InitRangePoint();
	InitScrewPitch();
		PrintScrewPitch();
	InitReductionRation();
		PrintReductionRation();
	InitAxisResolution();
		PrintAxisResolution();
	InitMaxOutputFrequency();
		PrintMaxOutputFrequency();
	InitMaxOutputSpeed();
		PrintMaxOutputSpeed();
	printf("<< Init Parameter Finish\n\n");
}

void SetCoordinatePoint(int PointPos, double Point_X, double Point_Y, double Point_Z, double Point_Angle)//設定座標點
{
	CoordinatePoint[PointPos][_X] = Point_X;
	CoordinatePoint[PointPos][_Y] = Point_Y;
	CoordinatePoint[PointPos][_Z] = Point_Z;
	CoordinatePoint[PointPos][_Angle] = Point_Angle * DegToRad;
}
void SetCoordinatePoint_Buf(int PointPos, double Point_X, double Point_Y, double Point_Z, double Point_Angle)//設定暫存座標點
{
	BufCoordinatePoint[PointPos][_X] = Point_X;
	BufCoordinatePoint[PointPos][_Y] = Point_Y;
	BufCoordinatePoint[PointPos][_Z] = Point_Z;
	BufCoordinatePoint[PointPos][_Angle] = Point_Angle;
}
void CalculateP2P()//以球體計算兩點相關數值
{
	P2P[Distance] = 0; P2P[Theta] = 0; P2P[Phi] = 0;
	double dX = CoordinatePoint[TargetPosition][_X] - CoordinatePoint[CurrentPosition][_X];
	double dY = CoordinatePoint[TargetPosition][_Y] - CoordinatePoint[CurrentPosition][_Y];
	double dZ = CoordinatePoint[TargetPosition][_Z] - CoordinatePoint[CurrentPosition][_Z];
	/* 公式
	兩點距離：double P2P[Distance]
		P2P[Distance] = sqrt(pow((X1-X0),2) + pow((Y1-Y0),2) + pow((Z1-Z0),2));
	兩點Theta角(Rad)：double P2P[Theta]
		P2P[Theta] = acos((Z1-Z0) / P2P[Distance]);
		P2P[Theta] = asin(sqrt(pow((X1-X0),2) + pow((Y1-Y0),2)) / P2P[Distance]);
		P2P[Theta] = atan(sqrt(pow((X1-X0),2) + pow((Y1-Y0),2)) / (Z1-Z0));
	兩點Phi角(Rad)：double P2P[Phi]
		P2P[Phi] = acos((X1-X0) / (P2P[Distance] * sin(P2P[Theta])));
		P2P[Phi] = asin((Y1-Y0) / (P2P[Distance] * sin(P2P[Theta])));
		P2P[Phi] = atan((Y1-Y0) / (X1-X0));
	X點計算：_X
		_X = P2P[Distance] * sin(P2P[Theta]) * cos(P2P[Phi])
	Y點計算：_Y
		_Y = P2P[Distance] * sin(P2P[Theta]) * sin(P2P[Phi])
	Z點計算：_Z
		_Z = P2P[Distance] * cos(P2P[Theta])
	*/
	P2P[Distance] = sqrt(pow(dX,2) + pow(dY,2) + pow(dZ,2));
	P2P[Distance_XY] = sqrt(pow(dX,2) + pow(dY,2));
	P2P[Distance_Z] = dZ;
	P2P[Distance_J3] =
		(CoordinatePoint[TargetPosition][_Angle] - CoordinatePoint[CurrentPosition][_Angle])
		+ atan((CoordinatePoint[CurrentPosition][_Y] - CoordinatePoint[OffsetPostiton][_Y]) / (CoordinatePoint[CurrentPosition][_X] - CoordinatePoint[OffsetPostiton][_X]))
		- atan((CoordinatePoint[TargetPosition][_Y] - CoordinatePoint[OffsetPostiton][_Y]) / (CoordinatePoint[TargetPosition][_X] - CoordinatePoint[OffsetPostiton][_X]));
	
	P2P[Theta] = acos(dZ / P2P[Distance]);//Z<0 or Z<0 皆可

	P2P[Phi] = acos(dX / (P2P[Distance] * sin(P2P[Theta])));//第一象限、第二象限
	if (dY < 0)
		P2P[Phi] = 2 * M_PI - P2P[Phi];//第三象限、第四象限

	P2P[P2PSpeed_XY] = SlowestSpeed_XYZ * sin(P2P[Theta]);
	P2P[P2PSpeed_Z] = SlowestSpeed_XYZ * cos(P2P[Theta]);
	P2P[P2PSpeed_J3] = OutputSpeed[J3][MaxOutSpeed];
	P2P[P2PSpeed] = SlowestSpeed_XYZ;
	P2P[SplitSize] = ceil(P2P[Distance] / PathSplitBaseDis_mm);
	P2P[SplitBaseDistance] = P2P[Distance] / P2P[SplitSize];

	PrintP2P();
}
void CalculateMoveData(double TempMaxSpeed, double TempDistance, int Path_Type)//計算移動值
{
	MoveData[Path_Type][AccMaxMoveDistance] = (TempMaxSpeed * (RunSpeedPercentage / 100) * AccelerateTime_sec) / 2;
	MoveData[Path_Type][DecMaxMoveDistance] = (TempMaxSpeed * (RunSpeedPercentage / 100) * DecelerateTime_sec) / 2;

	if ((MoveData[Path_Type][AccMaxMoveDistance] + MoveData[Path_Type][DecMaxMoveDistance]) < TempDistance)
	{//梯形V-t
		MoveData[Path_Type][Accelerate_sec] = AccelerateTime_sec;
		MoveData[Path_Type][Decelerate_sec] = DecelerateTime_sec;
		MoveData[Path_Type][TotalMoveTime_sec] = AccelerateTime_sec + DecelerateTime_sec + ((TempDistance - MoveData[Path_Type][AccMaxMoveDistance] - MoveData[Path_Type][DecMaxMoveDistance]) / (TempMaxSpeed * (RunSpeedPercentage / 100)));
		MoveData[Path_Type][MaxSpeed] = TempMaxSpeed * (RunSpeedPercentage / 100);// mm/sec or rad/sec or RPS
	}
	else
	{//三角V-t
		MoveData[Path_Type][Accelerate_sec] = sqrt(((2 * TempDistance * AccelerateTime_sec * DecelerateTime_sec) / (TempMaxSpeed * (RunSpeedPercentage / 100))) * (AccelerateTime_sec / (pow(DecelerateTime_sec, 2) + (AccelerateTime_sec * DecelerateTime_sec))));
		MoveData[Path_Type][Decelerate_sec] = (DecelerateTime_sec * MoveData[Path_Type][Accelerate_sec]) / AccelerateTime_sec;
		MoveData[Path_Type][TotalMoveTime_sec] = MoveData[Path_Type][Accelerate_sec] + MoveData[Path_Type][Decelerate_sec];
		MoveData[Path_Type][MaxSpeed] = MoveData[Path_Type][Accelerate_sec] * ((TempMaxSpeed * (RunSpeedPercentage / 100)) / AccelerateTime_sec);
	}

	MoveData[Path_Type][AccMaxMoveDistance] = (MoveData[Path_Type][Accelerate_sec] * MoveData[Path_Type][MaxSpeed]) / 2;
	MoveData[Path_Type][DecMaxMoveDistance] = (MoveData[Path_Type][Decelerate_sec] * MoveData[Path_Type][MaxSpeed]) / 2;
}
void RenewMoveData(double MoveTime, double TempDistance, int Path_Type)//以時間更新MoveData
{
	double MaxSpeedMoveTime_ms = 0.0;

	if (MoveTime >  (AccelerateTime_sec + DecelerateTime_sec))
	{//梯形V-t
		MaxSpeedMoveTime_ms = MoveTime - AccelerateTime_sec - DecelerateTime_sec;

		MoveData[Path_Type][TotalMoveTime_sec] = MoveTime;
		MoveData[Path_Type][MaxSpeed] = (2 * TempDistance) / (MoveTime + MaxSpeedMoveTime_ms);
		MoveData[Path_Type][Accelerate_sec] = AccelerateTime_sec;
		MoveData[Path_Type][Decelerate_sec] = DecelerateTime_sec;
		MoveData[Path_Type][AccMaxMoveDistance] = (MoveData[Path_Type][Accelerate_sec] * MoveData[Path_Type][MaxSpeed]) / 2;
		MoveData[Path_Type][DecMaxMoveDistance] = (MoveData[Path_Type][Decelerate_sec] * MoveData[Path_Type][MaxSpeed]) / 2;
	}
	else
	{//三角V-t
		MoveData[Path_Type][TotalMoveTime_sec] = MoveTime;
		MoveData[Path_Type][MaxSpeed] = (2 * TempDistance) / MoveTime;
		MoveData[Path_Type][Accelerate_sec] = MoveTime / (1 + (DecelerateTime_sec / AccelerateTime_sec));
		MoveData[Path_Type][Decelerate_sec] = MoveData[Path_Type][TotalMoveTime_sec] - MoveData[Path_Type][Accelerate_sec];
		MoveData[Path_Type][AccMaxMoveDistance] = (MoveData[Path_Type][Accelerate_sec] * MoveData[Path_Type][MaxSpeed]) / 2;
		MoveData[Path_Type][DecMaxMoveDistance] = (MoveData[Path_Type][Decelerate_sec] * MoveData[Path_Type][MaxSpeed]) / 2;
	}
}
void CalculateFirstTimeMoveData()//計算各平面移動值
{
	CalculateMoveData(P2P[P2PSpeed], P2P[Distance], Path_XYZ);
	CalculateMoveData(P2P[P2PSpeed_XY], P2P[Distance_XY], Path_XY);
	CalculateMoveData(P2P[P2PSpeed_Z], P2P[Distance_Z], Path_Z);
	CalculateMoveData(P2P[P2PSpeed_J3], P2P[Distance_J3], Path_Theta);
}
void CalculateFinalMoveData()//計算最後移動值
{
	double BufTime = -1;
	CalculateFirstTimeMoveData();

	for (size_t i = 0; i < PathTypeNumber; i++)//找出移動最長時間
	{
		if (MoveData[i][TotalMoveTime_sec] > BufTime)
			BufTime = MoveData[i][TotalMoveTime_sec];
	}
	
	RenewMoveData(BufTime, P2P[Distance], Path_XYZ);
	RenewMoveData(BufTime, P2P[Distance_XY], Path_XY);
	RenewMoveData(BufTime, P2P[Distance_Z], Path_Z);
	RenewMoveData(BufTime, P2P[Distance_J3], Path_Theta);
	PrintMoveData();
}
void CalculateSplitPlus()//計算各軸分割的位置、速度
{
	double ProgressiveDistance = 0.0;
	double ProgressiveDistance_XY = 0.0;
	double ProgressiveDistance_J3 = 0.0;
	double ProgressiveDistance_Z = 0.0;
	double CurrentTimeBuf_XY = 0.0;
	double TargetTimeBuf_XY = 0.0;
	double SpeedBuf_J1 = 0.0;
	double SpeedBuf_J2 = 0.0;
	double SpeedBuf_J3 = 0.0;
	double SpeedBuf_Z = 0.0;
	double SplitSpeed_J1 = 0.0;//plus/sec
	double SplitSpeed_J2 = 0.0;//plus/sec
	double SplitSpeed_J3 = 0.0;//plus/sec
	double SplitSpeed_Z = 0.0;//plus/sec
	double MoveDirection_J1 = 0.0;
	double MoveDirection_J2 = 0.0;
	double MoveDirection_J3 = 0.0;
	double MoveDirection_Z = 0.0;
	double SplitDistance = P2P[Distance] / P2P[SplitSize];
	double SplitDistance_XY = P2P[Distance_XY] / P2P[SplitSize];
	double SplitDistance_J3 = P2P[Distance_J3] / P2P[SplitSize];
	double SplitDistance_Z = P2P[Distance_Z] / P2P[SplitSize];
	double StartPos[PointNameNumber];
	double CurrentPos[PointNameNumber];
	double TargetPos[PointNameNumber];

	/* 公式
		兩點距離：double P2P[Distance]
			P2P[Distance] = sqrt(pow((X1-X0),2) + pow((Y1-Y0),2) + pow((Z1-Z0),2));
		兩點Theta角(Rad)：double P2P[Theta]
			P2P[Theta] = acos((Z1-Z0) / P2P[Distance]);
			P2P[Theta] = asin(sqrt(pow((X1-X0),2) + pow((Y1-Y0),2)) / P2P[Distance]);
			P2P[Theta] = atan(sqrt(pow((X1-X0),2) + pow((Y1-Y0),2)) / (Z1-Z0));
		兩點Phi角(Rad)：double P2P[Phi]
			P2P[Phi] = acos((X1-X0) / (P2P[Distance] * sin(P2P[Theta])));
			P2P[Phi] = asin((Y1-Y0) / (P2P[Distance] * sin(P2P[Theta])));
			P2P[Phi] = atan((Y1-Y0) / (X1-X0));
		X點計算：_X
			_X = P2P[Distance] * sin(P2P[Theta]) * cos(P2P[Phi])
		Y點計算：_Y
			_Y = P2P[Distance] * sin(P2P[Theta]) * sin(P2P[Phi])
		Z點計算：_Z
			_Z = P2P[Distance] * cos(P2P[Theta])
	*/
	
	StartPos[_X] = CoordinatePoint[CurrentPosition][_X] - CoordinatePoint[OffsetPostiton][_X];
	StartPos[_Y] = CoordinatePoint[CurrentPosition][_Y] - CoordinatePoint[OffsetPostiton][_Y];
	StartPos[_Z] = CoordinatePoint[CurrentPosition][_Z] - CoordinatePoint[OffsetPostiton][_Z];
	StartPos[_Angle] = (CoordinatePoint[CurrentPosition][_Angle] + CoordinatePoint[OffsetPostiton][_Angle]) * DegToRad;
	CurrentPos[_X] = StartPos[_X];
	CurrentPos[_Y] = StartPos[_Y];
	CurrentPos[_Z] = StartPos[_Z];
	CurrentPos[_Angle] = StartPos[_Angle];

	printf("[Theta: %.6f, Phi: %.6f]\n", P2P[Theta], P2P[Phi]);
	for (size_t i = 0; i < (int)P2P[SplitSize]; i++)
	{
		ProgressiveDistance = ProgressiveDistance + SplitDistance;
		ProgressiveDistance_XY = ProgressiveDistance_XY + SplitDistance_XY;
		ProgressiveDistance_J3 = ProgressiveDistance_J3 + SplitDistance_J3;
		ProgressiveDistance_Z = ProgressiveDistance_Z + SplitDistance_Z;

		TargetPos[_X] = StartPos[_X] + ProgressiveDistance * sin(P2P[Theta]) * cos(P2P[Phi]);	
		TargetPos[_Y] = StartPos[_Y] + ProgressiveDistance * sin(P2P[Theta]) * sin(P2P[Phi]);
		TargetPos[_Z] = StartPos[_Z] + ProgressiveDistance * cos(P2P[Theta]);
		TargetPos[_Angle] = StartPos[_Angle] + ProgressiveDistance_J3;

		printf("(%6d)\n", i);

		#pragma region J1J2(X, Y)
		/* 公式
		兩點距離：double P2P[Distance]
			P2P[Distance] = sqrt(pow((X1-X0),2) + pow((Y1-Y0),2) + pow((Z1-Z0),2));
		兩點Theta角(Rad)：double P2P[Theta]
			P2P[Theta] = acos((Z1-Z0) / P2P[Distance]);
			P2P[Theta] = asin(sqrt(pow((X1-X0),2) + pow((Y1-Y0),2)) / P2P[Distance]);
			P2P[Theta] = atan(sqrt(pow((X1-X0),2) + pow((Y1-Y0),2)) / (Z1-Z0));
		兩點Phi角(Rad)：double P2P[Phi]
			P2P[Phi] = acos((X1-X0) / (P2P[Distance] * sin(P2P[Theta])));
			P2P[Phi] = asin((Y1-Y0) / (P2P[Distance] * sin(P2P[Theta])));
			P2P[Phi] = atan((Y1-Y0) / (X1-X0));
		X點計算：_X
			_X = P2P[Distance] * sin(P2P[Theta]) * cos(P2P[Phi])
		Y點計算：_Y
			_Y = P2P[Distance] * sin(P2P[Theta]) * sin(P2P[Phi])
		Z點計算：_Z
			_Z = P2P[Distance] * cos(P2P[Theta])
		*/
		double MoveAngle = (acos(TargetPos[_X] / sqrt(pow(TargetPos[_X], 2) + pow(TargetPos[_Y], 2)))) - (acos(CurrentPos[_X] / sqrt(pow(CurrentPos[_X], 2) + pow(CurrentPos[_Y], 2)))); 
		
		if (MoveAngle < 0) 
			MoveDirection_J1 = -1.0;
		else
			MoveDirection_J1 = 1.0;
		
		if (ProgressiveDistance_XY < MoveData[Path_XY][AccMaxMoveDistance])
			TargetTimeBuf_XY = sqrt((2 * ProgressiveDistance_XY * MoveData[Path_XY][Accelerate_sec]) / MoveData[Path_XY][MaxSpeed]);
		else if (ProgressiveDistance_XY > P2P[Distance_XY] - MoveData[Path_XY][DecMaxMoveDistance])
			TargetTimeBuf_XY = MoveData[Path_XY][TotalMoveTime_sec] - sqrt((2 * sqrt(pow(MoveData[Path_XY][Decelerate_sec], 2)) * (P2P[Distance_XY] - ProgressiveDistance_XY)) / sqrt(pow(MoveData[Path_XY][MaxSpeed], 2)));
		else
			TargetTimeBuf_XY = (ProgressiveDistance_XY - ((MoveData[Path_XY][MaxSpeed] * MoveData[Path_XY][Accelerate_sec]) / 2)) / MoveData[Path_XY][MaxSpeed];

		SpeedBuf_J1 = MoveAngle / (TargetTimeBuf_XY - CurrentTimeBuf_XY);// rad/sec
		SplitSpeed_J1 = (OutputSpeed[J1][MaxOutFrequency] * (SpeedBuf_J1 / (2 * M_PI))) / OutputSpeed[J1][MaxOutSpeed];
		PrintAxisMovePlus((int)J1, MoveDirection_J1, MoveAngle, SpeedBuf_J1, 0.0, SplitSpeed_J1);

		double MoveDis = sqrt(pow(TargetPos[_X], 2) + pow(TargetPos[_Y], 2)) - sqrt(pow(CurrentPos[_X], 2) + pow(CurrentPos[_Y], 2));
		
		if (MoveDis < 0) 
			MoveDirection_J2 = -1.0;
		else
			MoveDirection_J2 = 1.0;
		
		SpeedBuf_J2 = sqrt(pow(MoveDis, 2)) / (TargetTimeBuf_XY - CurrentTimeBuf_XY);// mm/sec
		SplitSpeed_J2 = (OutputSpeed[J2][MaxOutFrequency] * SpeedBuf_J2) / OutputSpeed[J2][MaxOutSpeed];
		PrintAxisMovePlus((int)J2, MoveDirection_J2, MoveDis, SpeedBuf_J2, 0.0, SplitSpeed_J2);

		CurrentPos[_X] = TargetPos[_X];
		CurrentPos[_Y] = TargetPos[_Y];
		CurrentPos[_Z] = TargetPos[_Z];
		CurrentPos[_Angle] = TargetPos[_Angle];
		CurrentTimeBuf_XY = TargetTimeBuf_XY;
		#pragma endregion

		#pragma region J3(Angle)
		if (MoveData[Path_Theta][MaxSpeed] < 0)
			MoveDirection_J3 = -1.0;
		else
			MoveDirection_J3 = 1.0;
		
		if (sqrt(pow(ProgressiveDistance_J3, 2)) < sqrt(pow(MoveData[Path_Theta][AccMaxMoveDistance], 2)))
			SpeedBuf_J3 = sqrt(2 * ProgressiveDistance_J3 * (MoveData[Path_Theta][MaxSpeed] / MoveData[Path_Theta][Accelerate_sec]));
		else if (sqrt(pow(ProgressiveDistance_J3, 2)) > sqrt(pow(P2P[Distance_J3], 2)) - sqrt(pow(MoveData[Path_Theta][DecMaxMoveDistance], 2)))
			SpeedBuf_J3 = sqrt(((2 * sqrt(pow(MoveData[Path_Theta][MaxSpeed], 2))) * (sqrt(pow(P2P[Distance_J3], 2)) - sqrt(pow(ProgressiveDistance_J3 - SplitDistance_J3, 2)))) / sqrt(pow(MoveData[Path_Theta][Decelerate_sec], 2)));
		else
			SpeedBuf_J3 = sqrt(pow(MoveData[Path_Theta][MaxSpeed], 2));

		if (SpeedBuf_J3 < 0) 
			SpeedBuf_J3 = 0.0;
		if (SpeedBuf_J3 > sqrt(pow(MoveData[Path_Theta][MaxSpeed], 2))) 
			SpeedBuf_J3 = sqrt(pow(MoveData[Path_Theta][MaxSpeed], 2));

		SplitSpeed_J3 = (OutputSpeed[J3][MaxOutFrequency] * (SpeedBuf_J3 / (2 * M_PI))) / OutputSpeed[J3][MaxOutSpeed];
		PrintAxisMovePlus((int)J3, MoveDirection_J3, ProgressiveDistance_J3, SpeedBuf_J3, 0.0, SplitSpeed_J3);
		#pragma endregion

		#pragma region J4(Z)
		if (MoveData[Path_Z][MaxSpeed] < 0)
			MoveDirection_Z = -1.0;
		else
			MoveDirection_Z = 1.0;

		if (sqrt(pow(ProgressiveDistance_Z, 2)) < sqrt(pow(MoveData[Path_Z][AccMaxMoveDistance], 2)))
			SpeedBuf_Z = sqrt(2 * ProgressiveDistance_Z * (MoveData[Path_Z][MaxSpeed] / MoveData[Path_Z][Accelerate_sec]));
		else if (sqrt(pow(ProgressiveDistance_Z, 2)) > sqrt(pow(P2P[Distance_Z], 2)) - sqrt(pow(MoveData[Path_Z][DecMaxMoveDistance], 2)))
			SpeedBuf_Z = sqrt(((2 * sqrt(pow(MoveData[Path_Z][MaxSpeed], 2))) * (sqrt(pow(P2P[Distance_Z], 2)) - sqrt(pow(ProgressiveDistance_Z - SplitDistance_Z, 2)))) / sqrt(pow(MoveData[Path_Z][Decelerate_sec], 2)));
		else
			SpeedBuf_Z = sqrt(pow(MoveData[Path_Z][MaxSpeed], 2));

		if (SpeedBuf_Z < 0) 
			SpeedBuf_Z = 0.0;
		if (SpeedBuf_Z > sqrt(pow(MoveData[Path_Z][MaxSpeed], 2))) 
			SpeedBuf_Z = sqrt(pow(MoveData[Path_Z][MaxSpeed], 2));

		SplitSpeed_Z = (OutputSpeed[J4][MaxOutFrequency] * SpeedBuf_Z) / OutputSpeed[J4][MaxOutSpeed];
		PrintAxisMovePlus((int)J4, MoveDirection_Z, ProgressiveDistance_Z, SpeedBuf_Z, 0.0, SplitSpeed_Z);
		#pragma endregion
	}
}
#pragma endregion

void Core1_Entry()
{
	while(true)
	{
		ReceiveChar();
		if(strlen(Msg) > 0) ActionInt = Msg[0];
	}
}

int main() //Main Function
{
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
	
	sleep_us(10000000);
	Init_AllParameter();

	printf("<====Set Coordinate Point====>\n");
	SetCoordinatePoint(OffsetPostiton, -250.0, -250.0, 0.0, 0.0);
	PrintCoordinatePoint();
	printf("<====Raspberry Pi Pico Ready====>\n");
	multicore_launch_core1(Core1_Entry);
	
    while (true)
    {
		switch (ActionInt)
		{
		case '0':
			gpio_put(LED_PIN, 0);
			break;
		case '1':
			gpio_put(LED_PIN, 1);
			sleep_us(125000);
			gpio_put(LED_PIN, 0);
			sleep_us(125000);
			break;
		case 'T':
			RunSpeedPercentage = 100;
			SetCoordinatePoint(CurrentPosition, -245.0, 250.0, 3.0, 0.0);
			SetCoordinatePoint(TargetPosition, -255.0, 249.99, -3.0, 90.0);
			CalculateP2P();
			CalculateFinalMoveData();
			CalculateSplitPlus();

			printf("Done!!\n");

			ActionInt = 0;
			break;
		default:
			break;
		}

    }
    return 0;
}


