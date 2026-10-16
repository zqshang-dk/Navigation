#include "GPS_Structs.h"
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;


double  GetDiffTime(GPSTIME*t1, GPSTIME* t2) {
	return (t1->Week - t2->Week) * 604800.0 + (t1->SecOfWeek - t2->SecOfWeek);
}

/********************************************************************************
	函数名称：CompSatClkoff

	函数目的：计算GPS和BDS卫星钟差和钟速

	注意事项：
		1.星历过期判断，星历健康标记SVHealth是否为0，如果过期或者不健康，返回false，否则返回true；
		2.注意BDS星历的参考时间为BDT，GPST-BDT=14s
		3.计算成功，Mid->valid=true;

	输入参数：
		prn    卫星的prn号
		sys    卫星系统（GPS/BDS）
		t      卫星信号发射时间
		geph   GPS卫星星历
		beph   BDS卫星星历
		Mid    中间变量

	输出参数：
		不加相对论改正的卫星钟差    
*********************************************************************************/
// 用于判断卫星是否过期，是否健康
// 计算卫星“概略”钟差
bool CompSatClkOff(const int prn, const GNSSSYS sys, const GPSTIME* t, GPSEPH* geph, GPSEPH* beph, SATMIDRES* Mid) {
	
	double dt, Limt=7500;
	GPSTIME GT;
	GPSEPH* eph= geph + prn - 1;

	if (sys == GPS) {
		eph = geph + prn - 1;
		Limt = 7500.0;
		GT = *t;
	}

	else if (sys == BDS) {
		eph = beph + prn - 1;
		Limt = 3900.0;
		GT.Week = t->Week - 1356;
		GT.SecOfWeek = t->SecOfWeek - 14.0;
		if (GT.SecOfWeek < 0.0) {
			GT.SecOfWeek += 604800.0;
			GT.Week -= 1;
		}
	}

	dt = GetDiffTime(&GT, &eph->TOC);
	if (fabs(dt) > Limt || eph->SVhealth != 0) {
		return false;
	}
	Mid->SatClkOft = eph->a0 + eph->a1 * dt + eph->a2 * dt * dt;
	Mid->SatClkSft = eph->a1 + 2 * eph->a2 * dt;
	Mid->Valid = true;

	return true;
}


/******************************************************************************************
	函数名称：CompGPSSatPVT

	函数目的：计算GPS卫星的位置、速度、钟差、钟速（含相对论改正）

	注意事项：
		1.星历过期判断，星历健康标记SVHealth是否为0，如果过期或者不健康，返回false，否则返回true；
		2.注意BDS星历的参考时间为BDT，GPST-BDT=14s
		3.计算成功，Mid->valid=true;

	输入参数：
		prn    卫星的prn号
		sys    卫星系统（GPS/BDS）
		t      卫星信号发射时间
		geph   GPS卫星星历
		Mid    中间变量

	输出参数：
		GPS卫星的位置、速度、钟差、钟速（含相对论改正）
*********************************************************************************************/
int CompGPSSatPVT(const int prn, const GPSTIME* t, const GPSEPH* geph, SATMIDRES* Mid) {
	const GPSEPH* eph = geph + prn - 1;
	GPSTIME GT = *t;
	GPSTIME GTOE = eph->TOE;

	// 计算轨道长半轴
	double A = eph->sqrt_A * eph->sqrt_A;

	// 计算平均运算角速度
	double n0 = sqrt(mu_GPS / (A * A * A));

	// 计算相对于星历参考历元的时间
	double tk = GetDiffTime(&GT, &GTOE);
	//double tk = (t->Week - eph->TOE.Week) * 604800.0 + (t->SecOfWeek - eph->TOE.SecOfWeek);
	if (fabs(tk) > 7500.0 || eph->SVhealth != 0 || eph->PRN == 0) {
		Mid->Valid = false;
		return 0;
	}

	// 对平均运动角速度进行改正
	double n = n0 + eph->deltaN;
	
	// 计算平近点角
	double Mk = eph->M0 + n * tk;
	
	// 计算偏近点角（注意：加上循环次数）
	double Ek = Mk;
	double Ek_ref = Mk;
	for (int i = 0; i < 10; i++) {
		Ek_ref = Ek;
		Ek = Mk + eph->e * sin(Ek_ref);

		if (fabs(Ek - Ek_ref) < 1E-10) {
			break;
		}
	}

	// 计算真近点角
	//double vk = atan2((sqrt(1 - eph->e * eph->e) / (1 - eph->e * sin(Ek))), ((cos(Ek) - eph->e) / (1 - eph->e * cos(Ek))));
	double vk = atan2((sqrt(1 - eph->e * eph->e)*sin(Ek) ), ((cos(Ek) - eph->e)));

	// 计算升交点角距
	double fi_k = vk + eph->omega;

	// 计算二阶调和改正数
	// 计算升交点角距改正数
	double delta_uk = eph->Cus * sin(2 * fi_k) + eph->Cuc * cos(2 * fi_k);

	// 计算向径改正数
	double delta_rk = eph->Crs * sin(2 * fi_k) + eph->Crc * cos(2 * fi_k);

	// 计算轨道倾角改正数
	double delta_ik = eph->Cis * sin(2 * fi_k) + eph->Cic * cos(2 * fi_k);

	// 计算经过改正的升交点角距
	double uk = fi_k + delta_uk;

	// 计算经过改正的向径
	double rk = A * (1 - eph->e * cos(Ek)) + delta_rk;

	// 计算经过改正的轨道倾角
	double ik = eph->i0 + delta_ik + eph->IDOT * tk;

	// 计算卫星在轨道平面上的位置
	double xk1 = rk * cos(uk);
	double yk1 = rk * sin(uk);

	// 计算改正后的升交点赤经
	double OMEGA_k = eph->OMEGA + (eph->OMEGA_dot - OMEGA_e_dot_GPS) * tk - OMEGA_e_dot_GPS * eph->TOE.SecOfWeek;

	// 计算在地固坐标系下的位置
	Mid->SatPos[0] = xk1 * cos(OMEGA_k) - yk1 * cos(ik) * sin(OMEGA_k);
	Mid->SatPos[1] = xk1 * sin(OMEGA_k) + yk1 * cos(ik) * cos(OMEGA_k);
	Mid->SatPos[2] = yk1 * sin(ik);


	//GPS钟差改正
	double F = -2 * sqrt(mu_GPS) / (CLIGHT*CLIGHT);
	double delta_t = F * eph->e * eph->sqrt_A * sin(Ek);
	Mid->SatClkOft += delta_t;

	//GPS钟速计算
	double delta_tr_dot = F * eph->e * eph->sqrt_A * cos(Ek) * (n / (1 - eph->e * cos(Ek)));
	Mid->SatClkSft += delta_tr_dot;

	/******************************************GPS运动速度解算*******************************************************************/
	double Ek_dot = n / (1 - eph->e * cos(Ek));
	double fi_k_dot = sqrt(1 - eph->e * eph->e) * Ek_dot / (1 - eph->e * cos(Ek));
	double uk_dot = 2 * (eph->Cus * cos(2 * fi_k) - eph->Cuc * sin(2 * fi_k)) * fi_k_dot + fi_k_dot;
	double rk_dot = eph->sqrt_A * eph->sqrt_A * eph->e * sin(Ek) * Ek_dot + 2 * (eph->Crs * cos(2 * fi_k) - eph->Crc * sin(2 * fi_k)) * fi_k_dot;
	double ik_dot = eph->IDOT + 2 * (eph->Cis * cos(2 * fi_k) - eph->Cic * sin(2 * fi_k)) * fi_k_dot;
	double OMEGA_k_dot = eph->OMEGA_dot - OMEGA_e_dot_GPS;

	Matrix<double, 3, 4> R;
	R <<
		cos(OMEGA_k), -sin(OMEGA_k) * cos(ik), -(xk1 * sin(OMEGA_k) + yk1 * cos(OMEGA_k) * cos(ik)), yk1* sin(OMEGA_k)* sin(ik),
		sin(OMEGA_k), cos(OMEGA_k)* cos(ik), (xk1 * cos(OMEGA_k) - yk1 * sin(OMEGA_k) * cos(ik)), -yk1 * cos(OMEGA_k) * sin(ik),
		0, sin(ik), 0, yk1* cos(ik);

	double xk1_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
	double yk1_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);
	double xk_dot, yk_dot, zk_dot;
	Vector3d V/*(xk_dot, yk_dot, zk_dot)*/;
	Vector4d DOT(xk1_dot, yk1_dot, OMEGA_k_dot, ik_dot);
	V = R * DOT;

	Mid->SatVel[0] = V(0);
	Mid->SatVel[1] = V(1);
	Mid->SatVel[2] = V(2);

	Mid->Valid = true;
	
	Mid->Tgd1 = eph->tgd1 * CLIGHT;
	return 1;
}

/******************************************************************************************
	函数名称：CompBDSSatPVT

	函数目的：计算GPS卫星的位置、速度、钟差、钟速（含相对论改正）

	注意事项：
		1.星历过期判断，星历健康标记SVHealth是否为0，如果过期或者不健康，返回false，否则返回true；
		2.注意BDS星历的参考时间为BDT，GPST-BDT=14s
		3.计算成功，Mid->valid=true;

	输入参数：
		prn    卫星的prn号
		sys    卫星系统（GPS/BDS）
		t      卫星信号发射时间
		beph   BDS卫星星历
		Mid    中间变量

	输出参数：
		GPS卫星的位置、速度、钟差、钟速（含相对论改正）
*********************************************************************************************/
int CompBDSSatPVT(const int prn, const GPSTIME* t, const GPSEPH* beph, SATMIDRES* Mid) {
	const GPSEPH* eph = beph + prn - 1;
	GPSTIME BT = *t;
	BT.Week -= 1356;
	BT.SecOfWeek -= 14;

	if (BT.SecOfWeek < 0.0) {
		BT.SecOfWeek += 604800.0;
		BT.Week -= 1;
	}

	GPSTIME BTOE = eph->TOE;

	// 计算轨道长半轴
	double A = eph->sqrt_A * eph->sqrt_A;

	// 计算平均运算角速度
	double n0 = sqrt(mu_BDS / (A * A * A));
	// 计算相对于星历参考历元的时间
	double tk = GetDiffTime(&BT, &BTOE);
	if (fabs(tk) > 3900.0 || eph->SVhealth != 0 || eph->PRN == 0) {
		Mid->Valid = false;
		return 0;
	}

	// 对平均运动角速度进行改正
	double n = n0 + eph->deltaN;

	// 计算平近点角
	double Mk = eph->M0 + n * tk;

	// 计算偏近点角（注意：加上循环次数）
	double Ek = Mk;
	double Ek_ref = Mk;
	for (int i = 0; i < 10; i++) {
		Ek_ref = Ek;
		Ek = Mk + eph->e * sin(Ek_ref);

		if (fabs(Ek - Ek_ref) < 1E-10) {
			break;
		}
	}

	// 计算真近点角
	//double vk = atan2((sqrt(1 - eph->e * eph->e) / (1 - eph->e * sin(Ek))), ((cos(Ek) - eph->e) / (1 - eph->e * cos(Ek))));
	double vk = atan2((sqrt(1 - eph->e * eph->e)*sin(Ek)), ((cos(Ek) - eph->e)));

	// 计算升交点角距
	double fi_k = vk + eph->omega;

	// 计算二阶调和改正数
	// 计算升交点角距改正数
	double delta_uk = eph->Cus * sin(2 * fi_k) + eph->Cuc * cos(2 * fi_k);

	// 计算向径改正数
	double delta_rk = eph->Crs * sin(2 * fi_k) + eph->Crc * cos(2 * fi_k);

	// 计算轨道倾角改正数
	double delta_ik = eph->Cis * sin(2 * fi_k) + eph->Cic * cos(2 * fi_k);

	// 计算经过改正的升交点角距
	double uk = fi_k + delta_uk;

	// 计算经过改正的向径
	double rk = A * (1 - eph->e * cos(Ek)) + delta_rk;

	// 计算经过改正的轨道倾角
	double ik = eph->i0 + delta_ik + eph->IDOT * tk;

	// 计算卫星在轨道平面上的位置
	double xk = rk * cos(uk);
	double yk = rk * sin(uk);

	double Ek_dot = n / (1 - eph->e * cos(Ek));
	double fi_k_dot = sqrt(1 - eph->e * eph->e) * Ek_dot / (1 - eph->e * cos(Ek));
	double uk_dot = 2 * (eph->Cus * cos(2 * fi_k) - eph->Cuc * sin(2 * fi_k)) * fi_k_dot + fi_k_dot;
	double rk_dot = eph->sqrt_A * eph->sqrt_A * eph->e * sin(Ek) * Ek_dot + 2 * (eph->Crs * cos(2 * fi_k) - eph->Crc * sin(2 * fi_k)) * fi_k_dot;
	double ik_dot = eph->IDOT + 2 * (eph->Cis * cos(2 * fi_k) - eph->Cic * sin(2 * fi_k)) * fi_k_dot;
	double OMEGA_k_dot = eph->OMEGA_dot - OMEGA_e_dot_BDS;

	double xk1_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
	double yk1_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);

	// 升交点赤经
	if ((prn >= 1 && prn <= 5) || (prn >= 59 && prn <= 63)) {
		double OMEGA_k = eph->OMEGA + eph->OMEGA_dot * tk - OMEGA_e_dot_BDS * eph->TOE.SecOfWeek;
		
		double xgk = xk * cos(OMEGA_k) - yk * cos(ik) * sin(OMEGA_k);
		double ygk = xk * sin(OMEGA_k) + yk * cos(ik) * cos(OMEGA_k);
		double zgk = yk * sin(ik);
		Vector3d POS_GK(xgk, ygk, zgk);
		Vector3d POS;
		POS = Rz(OMEGA_e_dot_BDS * tk) * Rx(Degree2Rad(-5.0)) * POS_GK;
		Mid->SatPos[0] = POS(0);
		Mid->SatPos[1] = POS(1);
		Mid->SatPos[2] = POS(2);

		/******************************************GEO速度解算******************************************/
		Matrix<double, 3, 4> R;
		R <<
			cos(OMEGA_k), -sin(OMEGA_k) * cos(ik), -(xk * sin(OMEGA_k) + yk * cos(OMEGA_k) * cos(ik)), yk* sin(OMEGA_k)* sin(ik),
			sin(OMEGA_k), cos(OMEGA_k)* cos(ik), (xk * cos(OMEGA_k) - yk * sin(OMEGA_k) * cos(ik)), -yk * cos(OMEGA_k) * sin(ik),
			0, sin(ik), 0, yk* cos(ik);
		Vector3d V_GK;
		double OMEGA_k_dot_geo = eph->OMEGA_dot;
		Vector4d DOT(xk1_dot, yk1_dot, OMEGA_k_dot_geo, ik_dot);
		V_GK = R * DOT;

		Vector3d V;
		V = Rz(OMEGA_e_dot_BDS * tk) * Rx(Degree2Rad(-5.0)) * V_GK + OMEGA_e_dot_BDS * Rz_dot(OMEGA_e_dot_BDS * tk) * Rx(Degree2Rad(-5.0)) * POS_GK;
		Mid->SatVel[0] = V(0);
		Mid->SatVel[1] = V(1);
		Mid->SatVel[2] = V(2);
	}
	
	else {
		// 计算改正后的升交点赤经
		double OMEGA_k = eph->OMEGA + (eph->OMEGA_dot - OMEGA_e_dot_BDS) * tk - OMEGA_e_dot_BDS * eph->TOE.SecOfWeek;

		// 计算在地固坐标系下的位置
		Mid->SatPos[0] = xk * cos(OMEGA_k) - yk * cos(ik) * sin(OMEGA_k);
		Mid->SatPos[1] = xk * sin(OMEGA_k) + yk * cos(ik) * cos(OMEGA_k);
		Mid->SatPos[2] = yk * sin(ik);

		Matrix<double, 3, 4> R;
		R <<
			cos(OMEGA_k), -sin(OMEGA_k) * cos(ik), -(xk * sin(OMEGA_k) + yk * cos(OMEGA_k) * cos(ik)), yk* sin(OMEGA_k)* sin(ik),
			sin(OMEGA_k), cos(OMEGA_k)* cos(ik), (xk * cos(OMEGA_k) - yk * sin(OMEGA_k) * cos(ik)), -yk * cos(OMEGA_k) * sin(ik),
			0, sin(ik), 0, yk* cos(ik);

		/*double xk1_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
		double yk1_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);*/
		double xk_dot, yk_dot, zk_dot;
		Vector3d V/*(xk_dot, yk_dot, zk_dot)*/;
		Vector4d DOT(xk1_dot, yk1_dot, OMEGA_k_dot, ik_dot);
		V = R * DOT;

		Mid->SatVel[0] = V(0);
		Mid->SatVel[1] = V(1);
		Mid->SatVel[2] = V(2);
	}

	//BDS钟差改正
	double F = -2 * sqrt(mu_BDS) / (CLIGHT * CLIGHT);
	double delta_t = F * eph->e * eph->sqrt_A * sin(Ek);
	double k1_3 = FG1_BDS * FG1_BDS / (FG3_BDS * FG3_BDS);
	double delta_tsv = k1_3 * eph->tgd1 / (1 - k1_3);
	Mid->SatClkOft = Mid->SatClkOft + delta_t + delta_tsv;

	//BDS钟速计算
	double delta_tr_dot = F * eph->e * eph->sqrt_A * cos(Ek) * (n / (1 - eph->e * cos(Ek)));
	Mid->SatClkSft += delta_tr_dot;

	Mid->Valid = true;

	Mid->Tgd1 = eph->tgd1 * CLIGHT;
	Mid->Tgd2 = eph->tgd2 * CLIGHT;

	return 1;
}

/******************************************************************************************
	函数名称：EarthRotationCorrection

	函数目的：地球自转改正：卫星信号从发射到接受需要传播一段时间，这段时间地球存在自转。
			如果不做改正，发射时刻的卫星坐标和接收时刻的卫星坐标不一致，存在误差

	注意事项：
		1.星历过期判断，星历健康标记SVHealth是否为0，如果过期或者不健康，返回false，否则返回true；
		2.注意BDS星历的参考时间为BDT，GPST-BDT=14s
		3.计算成功，Mid->valid=true;

	输入参数：
		卫星的位置和速度

	输出参数：
		经过地球自传改正后卫星的位置和速度
*********************************************************************************************/
void EarthRotationCorrection(SATMIDRES* Mid, double UserPos[3], double Omegae) {
	double rou, dt;
	
	rou = sqrt(SQR(Mid->SatPos[0] - UserPos[0]) + SQR(Mid->SatPos[1] - UserPos[1]) + SQR(Mid->SatPos[2] - UserPos[2]));
	dt = rou / CLIGHT;

	Matrix<double, 3, 1> SatPOS;
	SatPOS <<
		Mid->SatPos[0],
		Mid->SatPos[1],
		Mid->SatPos[2];
	Matrix<double, 3, 1> SatVEL;
	SatVEL <<
		Mid->SatVel[0],
		Mid->SatVel[1],
		Mid->SatVel[2];

	SatPOS = Rz(Omegae * dt) * SatPOS;
	SatVEL = Rz(Omegae * dt) * SatVEL;
	Mid->SatPos[0] = SatPOS[0];
	Mid->SatPos[1] = SatPOS[1];
	Mid->SatPos[2] = SatPOS[2];

	Mid->SatVel[0] = SatVEL[0];
	Mid->SatVel[1] = SatVEL[1];
	Mid->SatVel[2] = SatVEL[2];
}

/******************************************************************************************
	函数名称：SatPVTCalculation

	函数目的：计算某历元观测中所有观测到的卫星的位置

	注意事项：1.卫星信号发射时刻=接收机接收时刻-P/C-卫星钟差

	输入参数：
		某历元的观测值和卫星的星历

	输出参数：
		卫星的位置、速度、钟差、钟速
*********************************************************************************************/
void SatPVTCalculation(GPSEPH* geph, GPSEPH* beph, EPOCHOBSDATA* obs, SPPPOS* UserPos/*double UserPos[3]*/) {	
	GPSTIME tr = obs->Time;  // 接收机钟的表面时，接收机接收到卫星信号的时刻
	GPSTIME ts;              // 卫星信号发射时刻
	double blh[3];
	double R, F;

	for (int i = 0; i < obs->SatNum; i++) {
		SATMIDRES* Mid = obs->SatPVT + i;   //Mid指向当前卫星的PVT计算结果结构体

		ts.Week = tr.Week;
		ts.SecOfWeek = tr.SecOfWeek - obs->SatObs[i].P[0] / CLIGHT; //第一次没有卫星钟差

		if (!CompSatClkOff(obs->SatObs[i].Prn,obs->SatObs[i].System,&ts,geph,beph,Mid)) {
			Mid->Valid = false;
			continue;
		}
		ts.SecOfWeek = tr.SecOfWeek - obs->SatObs[i].P[0] / CLIGHT - obs->SatPVT[i].SatClkOft; //相当于是数组的变量，用点.
		if (!CompSatClkOff(obs->SatObs[i].Prn, obs->SatObs[i].System, &ts, geph, beph, Mid)) {   // 迭代计算一次进行修正
			Mid->Valid = false;
			continue;
		}
		ts.SecOfWeek = tr.SecOfWeek - obs->SatObs[i].P[0] / CLIGHT - obs->SatPVT[i].SatClkOft; //相当于是数组的变量，用点.

		if (obs->SatObs[i].System == GPS) {
			if (!CompGPSSatPVT(obs->SatObs[i].Prn,&ts,geph,Mid)) {
				Mid->Valid = false;
				continue;
			}
			EarthRotationCorrection(Mid, UserPos->Pos, Omega_WGS);

		}
		else if (obs->SatObs[i].System == BDS) {

			if (!CompBDSSatPVT(obs->SatObs[i].Prn,&ts,beph,Mid)) {
				Mid->Valid = false;
				continue;
			}
			EarthRotationCorrection(Mid, UserPos->Pos, Omega_BDS);
		}
		else {
			Mid->Valid = false;
			continue;
		}

		Mid->Valid = true;

		// 计算卫星的高度角和对流层改正
		if (obs->SatObs[i].System == GPS) {
			R = R_WGS84;
			F = F_WGS84;
		}
		else if (obs->SatObs[i].System == BDS) {
			R = R_CGS2K;
			F = F_CGS2K;
		}
		else continue;
		CompSatElAz(UserPos->Pos, obs->SatPVT[i].SatPos, R, F, &obs->SatPVT[i].Elevation, &obs->SatPVT[i].Azimuth);
		XYZToBLH(UserPos->Pos, blh, R, F);
		obs->SatPVT[i].TropCorr = HopefieldTropCorr(blh[2], obs->SatPVT[i].Elevation);
		//cout << "User BLH: B=" << blh[0]
		//	<< " L=" << blh[1]
		//	<< " H=" << blh[2]
		//	<< " Elev=" << obs->SatPVT[i].Elevation
		//	<< endl;
	}
}

