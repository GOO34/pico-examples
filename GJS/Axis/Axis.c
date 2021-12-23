#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define FLAG_VALUE 123
#define ENDSTDIN 255
#define CR 13

#define MaxRPM 3000
#define _pulse_per_round 3000
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
	AxisPlusNumber
}eAxisPlusValue = AxisPlusNumber;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

char Msg[256];
double ReductionRation[AxisNumber];//各軸減速比
int MaxOutputFrequency[AxisNumber];//各軸最大輸出頻率 plus/min
double ScrewPitch[AxisNumber];//各軸Pitch mm
double MaxOutputSpeed[AxisNumber];//經減速機後最大速度(RPM, mm/min)
double AxisResolution[AxisNumber];//軸輸出解析(Angle/plus, mm/plus)
double CoordinatePoint[PositionNumber][PointNameNumber];//座標點
double AxisPosition[AxisNumber][PositionNumber];//軸位置
double MovableRange[AxisNumber][RangePointNumber];//移動範圍
int AxisPlusData[AxisNumber][AxisPlusNumber];//軸移動資料
int RunSpeedPercentage = 100;//運行速度百分比
double SlowestSpeed = 0;//最慢速度

int command;
int count;
double _mm_per_pulse = 0.01;

int ActionInt = 0;

#pragma region Custom//自定義
void absolute_move(double current_position, double target_position, double accelerate_time_ms, double decelerate_time_ms, double max_speed, double initial_speed, double final_speed)
{
	int pulse_per_round = _pulse_per_round;
	double mm_per_pulse = _mm_per_pulse;
	/*
	* S = (Vi +Vmax) * Tacc / 2 + Vmax * (Tt - Tacc - Tde) + (Vmax + Vf) * Tde / 2;
	* Tmax = Tt - Tacc - Tde = (S - (1/2) * ((Vi +Vmax) * Tacc + (Vmax + Vf) * Tde))/ Vmax ;
	* Tt = Tmax + Tacc +Tde;
	*/

	int max_time_ms = (int)round(((target_position - current_position) - (1 / 2) * ((initial_speed + max_speed) * accelerate_time_ms + (max_speed + final_speed) * decelerate_time_ms)) / max_speed);
	//int total_time_ms = (int)round(max_time_ms - accelerate_time_ms - decelerate_time_ms);

	//最高速度-pulse週期(ms)
	int max_ms_per_pulse = mm_per_pulse / max_speed;
	if (max_ms_per_pulse < 1)
		max_ms_per_pulse = 1;
	//初始速度-pulse週期(ms)
	int inital_ms_per_pulse = mm_per_pulse / initial_speed;
	if (inital_ms_per_pulse < 1)
		inital_ms_per_pulse = 1;
	//最終速度-pulse週期(ms)
	int final_ms_per_pulse = mm_per_pulse / final_speed;
	if (final_ms_per_pulse < 1)
		final_ms_per_pulse = 1;

	//最高速度段-總共需要的pulse
	int max_pulse = max_speed * max_time_ms / mm_per_pulse;
	//加速度段-總共需要的pulse
	int accelerate_pulse = max_speed * max_time_ms / mm_per_pulse;
	//減速度段-總共需要的pulse
	int decelerate_pulse = max_speed * max_time_ms / mm_per_pulse;

	int accelerate_decrease_interval = (max_ms_per_pulse - inital_ms_per_pulse) / accelerate_pulse;
	int decelerate_increase_interval = (final_ms_per_pulse - max_ms_per_pulse) / accelerate_pulse;

	//加速段
	for (size_t i = 0; i < accelerate_pulse; i++)
	{
		int interval = inital_ms_per_pulse + i * accelerate_decrease_interval;
		rotate_pulse(interval);
	}
	//最高速段
	for (size_t i = 0; i < max_pulse; i++)
	{
		int interval = max_ms_per_pulse;
		rotate_pulse(interval);
	}
	//減速段
	for (size_t i = 0; i < decelerate_pulse; i++)
	{
		int interval = max_ms_per_pulse + i * decelerate_increase_interval;
		rotate_pulse(interval);
	}
}

double ReductionRatioCalculation(int Reducer_In, int Reducer_Out, int GearTeeth_In, int GeraTeeth_Out)
{
	double ReducerReductionRatio = (double)Reducer_In/(double)Reducer_Out;
	double GearReductionRatio = (double)GeraTeeth_Out/(double)GearTeeth_In;
	return ReducerReductionRatio * GearReductionRatio;
}
void Screw_Pitch(double J1_Picth, double J2_Picth, double J3_Picth, double J4_Picth)//各軸螺桿Pitch mm(無螺桿值為1.0)
{
	printf("Screw Pitch = [");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		eSCARA = i;
		switch (eSCARA)
		{
			case J1: ScrewPitch[J1] = J1_Picth; break;
			case J2: ScrewPitch[J2] = J2_Picth; break;
			case J3: ScrewPitch[J3] = J3_Picth; break;
			case J4: ScrewPitch[J4] = J4_Picth; break;
			default: break;
		}

		if (i != 0)
			printf(", %.2f mm", ScrewPitch[i]);
		else
			printf("%.2f mm", ScrewPitch[i]);
	}
	printf("]\n");
}
void Reduction_Ration(int Reducer_In[], int Reducer_Out[], int GearTeeth_In[], int GeraTeeth_Out[])//減速比計算
{
	printf("Reduction Ration = [");
	for (size_t i = J1; i < AxisNumber; i++)
	{
		ReductionRation[i] = (double)ReductionRatioCalculation(Reducer_In[i], Reducer_Out[i], GearTeeth_In[i], GeraTeeth_Out[i]);

		if (i != 0)
			printf(", %.6f", ReductionRation[i]);
		else
			printf("%.6f", ReductionRation[i]);
	}
	printf("]\n");
}
void Axis_Resolution()//各軸輸出解析計算
{
	printf("Axis Resolution = [");
	for (size_t i = J1; i < AxisNumber; i++)
	{
		if (i != 0) printf(", ");
		
		eSCARA = i;
		double MotorOutResolution = (double)_pulse_per_round * ReductionRation[eSCARA];
		
		if (eSCARA == J1 || eSCARA == J3)
		{
			AxisResolution[i] = 360.0 / MotorOutResolution;
			printf("%.3f Angle/plus", AxisResolution[i]);
		}
		else
		{
			AxisResolution[i] = ScrewPitch[i] / MotorOutResolution;
			printf("%.3f mm/plus", AxisResolution[i]);
		}
	}
	printf("]\n");
}
void Max_Output_Frequency()//最大輸出頻率計算
{
	printf("Max Output Frequency = [");
	
	for (size_t i = 0; i < AxisNumber; i++)
	{
		MaxOutputFrequency[i] = MaxRPM * _pulse_per_round;

		if (i != 0)
			printf(", %d plus/rev", MaxOutputFrequency[i]);
		else
			printf("%d plus/rev", MaxOutputFrequency[i]);
	}
	printf("]\n");
}
void Max_Output_Speed()//經減速機後最大速度計算
{
	printf("Max Output Speed = [");
	for (size_t i = J1; i < AxisNumber; i++)
	{
		eSCARA = i;
		MaxOutputSpeed[eSCARA] = ((double)MaxRPM / ReductionRation[eSCARA]) * ScrewPitch[eSCARA];
		switch (eSCARA)
		{
			case J1: printf("%.2f RPM", MaxOutputSpeed[eSCARA]); break;
			case J2: printf(", %.2f mm/min", MaxOutputSpeed[eSCARA]); break;
			case J3: printf(", %.2f RPM", MaxOutputSpeed[eSCARA]); break;
			case J4: printf(", %.2f mm/min", MaxOutputSpeed[eSCARA]); break;
			default: break;
		}
	}
	printf("]\n");
}
void Slowest_Speed()//最慢速
{
	for (size_t i = J1; i < AxisNumber; i++)
	{
		eSCARA = i;
		MaxOutputSpeed[eSCARA] = (double)((MaxRPM / ReductionRation[eSCARA]) * ScrewPitch[eSCARA]);
		switch (eSCARA)
		{
			case J2: SlowestSpeed = MaxOutputSpeed[J2]; break;
			case J4: if (MaxOutputSpeed[J4] < SlowestSpeed) SlowestSpeed = MaxOutputSpeed[J2]; break;
			default: break;
		}
	}
	printf("Slowest Speed = %6.f\n", SlowestSpeed);
}
void PrintCoordinatePoint()//Print全部設定座標點
{
	printf("New Coordinate Point >>\n");
	for (size_t i = 0; i < PositionNumber; i++) 
	{
		ePointPosition = i;
		printf("\t[%d] = [", ePointPosition);
		for (size_t j = 0; j < PointNameNumber; j++)
		{
			ePointName = j;
			if(j != 0) printf(", ");
			printf("%.6f", CoordinatePoint[ePointPosition][ePointName]);
		}
		printf("]\n");
	}
}
void PrintAxisPosition()//Print全部軸位置
{
	printf("Axis Position>>\n");
	for (size_t i = 0; i < AxisNumber; i++) 
	{
		eSCARA = i;
		printf("\t[");
		for (size_t j = 0; j < PositionNumber; j++)
		{
			ePointPosition = j;
			if(j != 0) printf(", ");
			printf("%.6f", AxisPosition[eSCARA][ePointPosition]);
		}
		printf("]\n");
	}
}
void InitCoordinatePoint()//座標點初始化
{
	printf("Coordinate Point Initial......\n");
	for (size_t i = 0; i < PositionNumber; i++)
	{
		ePointPosition = i;
		for (size_t j = 0; j < PointNameNumber; j++)
			CoordinatePoint[i][j] = 0.000;
	}
}
void InitRangePoint()//移動範圍初始化
{
	printf("Range Point Initial......\n");
	for (size_t i = 0; i < AxisNumber; i++)
	{
		eSCARA = i;
		switch (eSCARA)
		{
			case J1: 
				MovableRange[eSCARA][Angle_Min] = -180.0;
				MovableRange[eSCARA][Angle_Max] = 180.0;
				MovableRange[eSCARA][Distance_Min] = 473.0;
				MovableRange[eSCARA][Distance_Max] = 473.0;
				printf("J1 Movable Range\n\tAngle = %.4f Deg ~ %.4f Deg\n\tDistance = %.3f mm ~ %.3f mm\n", MovableRange[eSCARA][Angle_Min], MovableRange[eSCARA][Angle_Max], MovableRange[eSCARA][Distance_Min], MovableRange[eSCARA][Distance_Max]);
				break;
			case J2: 
				MovableRange[eSCARA][Angle_Min] = 0.0;
				MovableRange[eSCARA][Angle_Max] = 0.0;
				MovableRange[eSCARA][Distance_Min] = 0.0;
				MovableRange[eSCARA][Distance_Max] = 300.0;
				printf("J2 Movable Range\n\tAngle = %.4f Deg ~ %.4f Deg\n\tDistance = %.3f mm ~ %.3f mm\n", MovableRange[eSCARA][Angle_Min], MovableRange[eSCARA][Angle_Max], MovableRange[eSCARA][Distance_Min], MovableRange[eSCARA][Distance_Max]);
				break;
			case J3: 
				MovableRange[eSCARA][Angle_Min] = -180.0;
				MovableRange[eSCARA][Angle_Max] = 180.0;
				MovableRange[eSCARA][Distance_Min] = 0.0;
				MovableRange[eSCARA][Distance_Max] = 0.0;
				printf("J3 Movable Range\n\tAngle = %.4f Deg ~ %.4f Deg\n\tDistance = %.3f mm ~ %.3f mm\n", MovableRange[eSCARA][Angle_Min], MovableRange[eSCARA][Angle_Max], MovableRange[eSCARA][Distance_Min], MovableRange[eSCARA][Distance_Max]);
				break;
			case J4: 
				MovableRange[eSCARA][Angle_Min] = 0.0;
				MovableRange[eSCARA][Angle_Max] = 0.0;
				MovableRange[eSCARA][Distance_Min] = 0.0;
				MovableRange[eSCARA][Distance_Max] = 400.0;
				printf("J4 Movable Range\n\tAngle = %.4f Deg ~ %.4f Deg\n\tDistance = %.3f mm ~ %.3f mm\n", MovableRange[eSCARA][Angle_Min], MovableRange[eSCARA][Angle_Max], MovableRange[eSCARA][Distance_Min], MovableRange[eSCARA][Distance_Max]);
				break;
			default: break;
		}
	}
}
void InitAxisPlusData()//軸移動資料初始化
{
	printf("Axis Plus Data Initial......\n");
	for (size_t i = 0; i < AxisNumber; i++)
		for (size_t j = 0; j < AxisPlusNumber; j++)
			AxisPlusData[i][j] = 0;
}
void WriteCoordinatePoint(int PointPos, double Point_X, double Point_Y, double Point_Z, double Point_Angle)//寫入座標點
{
	CoordinatePoint[PointPos][_X] = Point_X;
	CoordinatePoint[PointPos][_Y] = Point_Y;
	CoordinatePoint[PointPos][_Z] = Point_Z;
	CoordinatePoint[PointPos][_Angle] = Point_Angle;
}
void SetToTempPosition(int PointPos)//設定到動態座標點
{
	for (size_t i = 0; i < PointNameNumber; i++)
		CoordinatePoint[TempPostiton][i] = CoordinatePoint[PointPos][i] - CoordinatePoint[OffsetPostiton][i];
}
void CalculateJ1J2J3(double AngleRad, int tPos)
{
	double J1J2_DistanceMin = MovableRange[J1][Distance_Min] + MovableRange[J2][Distance_Min];
	double J1J2_DistanceMax = MovableRange[J1][Distance_Max] + MovableRange[J2][Distance_Max];
	double PointToOriginDistance = CoordinatePoint[TempPostiton][_X] / cos(AngleRad);

	printf("Point to Origin Distance: %.6f mm\n", PointToOriginDistance);
	// Check Distance
	if ((PointToOriginDistance >= J1J2_DistanceMin) & (PointToOriginDistance <= J1J2_DistanceMax)) 
	{
		AxisPosition[J1][tPos] = AngleRad;
		AxisPosition[J2][tPos] = PointToOriginDistance - MovableRange[J1][Distance_Max];
		AxisPosition[J3][tPos] = (CoordinatePoint[TempPostiton][_Angle] * DegToRad) - AngleRad;
		AxisPosition[J4][tPos] = CoordinatePoint[TempPostiton][_Z];
	}
	else
	{
		printf("Point out of range!\n");
	}
	PrintAxisPosition();
}

void CalculateAxisPosition_SCARA(int Pos)
{
	SetToTempPosition(Pos);

	double Angle_Radian_J1 = atan(CoordinatePoint[TempPostiton][_Y] / CoordinatePoint[TempPostiton][_X]);
	CalculateJ1J2J3(Angle_Radian_J1, Pos);
}
//TempPostiton


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

	if(strlen(Msg) > 0) printf("Receive Char: %s\n", Msg);
	
	return ReceiveChar;
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
	InitCoordinatePoint();
	InitAxisPlusData();
	InitRangePoint();
	printf("Degree to Radian Parameter: %.10f\nRadian to Degree Parameter: %.10f\n", DegToRad, RadToDeg);
	printf("<====SCARA Parameter Initialization====>\n");
	Screw_Pitch(1.0, 10.0, 1.0, 25.0);
	int Reducer_In[AxisNumber] = {100, 1, 70, 1};
	int Reducer_Out[AxisNumber] = {1, 1, 1, 1};
	int GearTeeth_In[AxisNumber] = {1, 36, 12, 14};
	int GearTeeth_Out[AxisNumber] = {1, 24, 22, 22};
	Reduction_Ration(Reducer_In, Reducer_Out, GearTeeth_In, GearTeeth_Out);
	Axis_Resolution();
	Max_Output_Frequency();
	Max_Output_Speed();
	Slowest_Speed();
	printf("<====Set Coordinate Point====>\n");
	WriteCoordinatePoint(OffsetPostiton, -250.0, -250.0, 0.0, 0.0);
	WriteCoordinatePoint(CurrentPosition, 100.1, 100.0, 0.0, 0.0);
	WriteCoordinatePoint(TargetPosition, 111.1, 111.1, 11.0, 0.0);
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
			CalculateAxisPosition_SCARA(CurrentPosition);
			CalculateAxisPosition_SCARA(TargetPosition);
			ActionInt = 0;
			break;
		
		default:
			break;
		}

    }
    return 0;
}


