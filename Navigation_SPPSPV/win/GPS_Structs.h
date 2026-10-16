#ifndef _GNSS_RTK_H_
#define _GNSS_RTK_H_

#include <stdio.h>
#include <Windows.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

using namespace Eigen;

#define FILEMODE 0  //1-file 0-ip
         
#define PI                3.1415926535898
#define CLIGHT            299792458.0
#define SQR(x)            ((x)*(x))
#define MAXIteration      15
#define Rad2Deg           (180.0/PI)
#define Deg2Rad           (PI/180.0)

#define R_WGS84           6378137.0          /* Radius Earth [m]; WGS-84  */
#define F_WGS84           1.0/298.257223563  /* Flattening; WGS-84   */
#define Omega_WGS         7.2921151467e-5   /*[rad/s], the earth rotation rate */
//#define GM_Earth         398600.5e+9      /* [m^3/s^2]; WGS-84 */
#define R_CGS2K           6378137.0          /* Radius Earth [m]; CGCS2000  */
#define F_CGS2K           1.0/298.257222101  /* Flattening; CGCS2000   */
#define Omega_BDS         7.2921150e-5      /*[rad/s], the earth rotation rate */
//#define GM_BDS           398600.4418e+9     /* [m^3/s^2]; CGCS2000  */
#define mu_GPS            3.986005e14
#define OMEGA_e_dot_GPS   7.2921151467e-5
#define mu_BDS            3.986004418e14
#define OMEGA_e_dot_BDS   7.2921150e-5

#define POLYCRC32         0xEDB88320u /* CRC32 polynomial */
#define MAXCHANNUM        148
#define MAXGPSNUM         32
#define MAXBDSNUM         63
#define MAXRAWLEN         40000               // 原始数据缓冲区最大长度

#define  FG1_GPS          1575.42E6                /* L1信号频率 */
#define  FG2_GPS          1227.60E6                /* L2信号频率*/
#define  WL1_GPS          (CLIGHT/FG1_GPS)
#define  WL2_GPS          (CLIGHT/FG2_GPS)

#define  FG1_BDS          1561.098E6               /* B1信号频率 */
#define  FG2_BDS          1207.140E6               /* B2信号频率 */
#define  FG3_BDS          1268.520E6               /* B3信号频率 */
#define  WL1_BDS          (CLIGHT/FG1_BDS)
#define  WL2_BDS          (CLIGHT/FG2_BDS)
#define  WL3_BDS          (CLIGHT/FG3_BDS)

/************************粗差探测MWGF阈值设置********************/
#define  GF_Threshold     0.05                  //GF阈值
#define  MW_Threshold     3.00                  //MW阈值

/************************对流层改正所需参数********************/
#define  H0               0                     //海平面高度
#define  T0               15 + 273.16           //标准温度
#define  P0               1013.25               //气压（单位mbar）
#define  RH0              0.50                  //相对湿度

//GNSS Systems
enum GNSSSYS { GPS = 0, BDS = 1, GLONASS = 2, GALILEO = 3, QZSS = 4, UNKNOWN = 5 };

struct COMMONTIME {
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;

	COMMONTIME()
	{
		Year = 0;
		Month = 0;
		Day = 0;
		Hour = 0;
		Minute = 0;
		Second = 0.0;
	}
};

struct MJDTIME {
	int Days;
	double FracDay;

	MJDTIME() {
		Days = 0;
		FracDay = 0.0;
	}
};

struct GPSTIME {
	short Week;
	double SecOfWeek;

	GPSTIME() {
		Week = 0;
		SecOfWeek = 0.0;
	}

	GPSTIME(short w,double sow):Week(w),SecOfWeek(sow){}
};

union XYZ {
	double xyz[3];
	struct {
		double x;
		double y;
		double z;
	};
};

union BLH {
	double blh[3];
	struct {
		double b;
		double l;
		double h;
	};
};

union ENU {
	double enu[3];
	struct {
		double e;
		double n;
		double u;
	};
};

//GPS广播星历结构体
struct GPSEPH {
	short PRN;
	GNSSSYS System;
	GPSTIME TOC, TOE;    //钟差和星历拟合的参考时间
	double a0, a1, a2;   //钟差，钟漂，频漂
	double IODE, IODC;
	double sqrt_A, M0, e, OMEGA, i0, omega;
	double Crc, Crs, Cuc, Cus, Cic, Cis;
	double deltaN, OMEGA_dot, IDOT;
	int SVhealth;
	double tgd1, tgd2;
};

// 每颗卫星的观测值
struct SATOBSDATA {
	short Prn;
	GNSSSYS System;
	double P[2], L[2], D[2];   //双频
	double   cn0[2], LockTime[2];
	unsigned char half[2];
	bool valid;

	SATOBSDATA()
	{
		Prn = 0;
		System = UNKNOWN;
		for (int i = 0; i < 2; i++)
			P[i] = L[i] = D[i] = 0.0;
		valid = false;
	}
};

/* 每颗卫星位置、速度和钟差等的中间计算结果 */
struct SATMIDRES
{
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;
	double Elevation, Azimuth;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid;  //false=没有星历或星历过期,true-计算成功

	SATMIDRES()
	{
		SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
		SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
		Elevation = PI / 2.0;
		SatClkOft = SatClkSft = 0.0;
		Tgd1 = Tgd2 = TropCorr = 0.0;
		Valid = false;
	}
};

struct MWGF {
	short Prn;
	GNSSSYS System;
	double MW;
	double GF;
	double PIF;
	int n;    //平滑计数

	MWGF() {
		Prn = n = 0;
		System = UNKNOWN;
		MW = GF = PIF = 0.0;
	}
};

//每个历元的观测值
struct EPOCHOBSDATA
{
	GPSTIME      Time;     // 没有经过钟差改正的时间（接收机钟的表面时）
	short        SatNum;
	SATOBSDATA   SatObs[MAXCHANNUM];     //原始观测
	SATMIDRES    SatPVT[MAXCHANNUM];     //卫星状态
	MWGF         CombObs[MAXCHANNUM];    //组合观测

	EPOCHOBSDATA()
	{
		SatNum = 0;
	}
};

// 用户位置速度信息
struct SPPPOS {
	GPSTIME Time;
	double Pos[3], Vel[3];
	double RcvClk[2];
	double PDOP, SigmaPos, SigmaVel;
	MatrixXd Dxx;
	int SatNum;
};

/* AngleChange.cpp */
double Degree2Rad(double theta);
double Rad2Degree(double theta);

/* MatrixModule.cpp */
Matrix3d BLH2ENUMat(double blh[3]);
Matrix3d Rx(double theta);
Matrix3d Rz(double theta);
Matrix3d Rz_dot(double theta);

/* TimeChange.cpp */
void CommonTimeToMJDTime(const COMMONTIME* ct, MJDTIME* mjd);
void MJDTimeToCommonTime(const MJDTIME* mjd, COMMONTIME* ct);
void MJDTimeToGPSTime(const MJDTIME* mjd, GPSTIME* gt);
void GPSTimeToMJDTime(const GPSTIME* gt, MJDTIME* mjd);
void CommonTimeToGPSTime(const COMMONTIME* ct, GPSTIME* gt);
void GPSTimeToCommonTime(const GPSTIME* gt, COMMONTIME* ct);


/* CoordinateSystem.cpp */
double Distance(double X1[3], double X2[3]);
void BLHToXYZ(const double blh[3], double xyz[3], const double R, const double F);
void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F);
//void BLHToENUMat(const double blh[], double Mat[]);
void CompSatElAz(const double Xr[], const double Xs[], double R, double F, double* Elev, double* Azim); void CompSatElAz(const double Xr[], const double Xs[], double* Elev, double* Azim);
void CompEnudPos(const double X0[], const double Xr[], double dENU[], const double R, const double F);


/*Decode.cpp*/
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBSDATA*obs, GPSEPH geph[], GPSEPH beph[]);
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBSDATA* obs);
int decode_gpsephem(unsigned char* buff, GPSEPH* geph);
int decode_bdsephem(unsigned char* buff, GPSEPH* beph);
//int decode_psrpos(unsigned char* buff, POSRES* pos);

/*decode*/
double D8(unsigned char* p);
float F4(unsigned char* p);
int I4(unsigned char* p);
unsigned int UI4(unsigned char* p);
short S2(unsigned char* p);
unsigned short US2(unsigned char* p);

/* SatPVTCalculation.cpp*/
bool CompSatClkOff(const int Prn, const GNSSSYS Sys, const GPSTIME* t,GPSEPH* GPSEph, GPSEPH* BDSEph, SATMIDRES* Mid);
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPH* Eph, SATMIDRES* Mid);
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPH* Eph, SATMIDRES* Mid);
void EarthRotationCorrection(SATMIDRES* Mid, double UserPos[3], double Omegae);
//void SatPVTCalculation(GPSEPH* geph, GPSEPH* beph, EPOCHOBSDATA* obs, double UserPos[3]);
void SatPVTCalculation(GPSEPH* geph, GPSEPH* beph, EPOCHOBSDATA* obs, SPPPOS* UserPos/*double UserPos[3]*/);
//void TestSatPvt(EPOCHOBSDATA* obs, GPSEPH* geph, GPSEPH* beph);

/* ErrorCorrect.cpp*/
double HopefieldTropCorr(double H, double E);
void DetectOutlier(EPOCHOBSDATA* obs);


/* Socket.cpp */
void CloseSocket(SOCKET& sock);
bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);


/* SPP.cpp */
//void SatPVTatSignal(GPSEPH* geph, GPSEPH* beph, double UserPos[3], EPOCHOBSDATA* obs);
bool SPP(GPSEPH* geph, GPSEPH* beph, SPPPOS* UserPos, EPOCHOBSDATA* obs);
void SPV(EPOCHOBSDATA* obs, SPPPOS* UserPos);
#endif