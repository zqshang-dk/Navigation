#include "GPS_Structs.h"

/********************************************************************************
	函数名称：HopefieldTropCorr

	函数目的：对流层延迟改正

	注意事项：
		1. 注意E的单位是角度还是rad

	输入参数：
		H       测站高度
		E       卫星相对于测站的高度角
		
	输出参数：
		dTrop   对流层延迟改正数
********************************************************************************/

double HopefieldTropCorr(double H, double E) {
	double RH, p, T, e, hw, hd, Kw, Kd, dTrop;
	if (H < 0) {
		return 0;
	}
	if (H < -1000.0 || H > 50000.0) {
		return 0.0;
	}
	if (E <= 0.0) {
		return 0.0;
	}
	RH = RH0 * exp(-0.0006396 * (H - H0));
	p = P0 * pow((1 - 0.0000226 * (H - H0)), 5.225);
	T = T0 - 0.0065 * (H - H0);
	e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	hw = 11000;
	hd = 40136 + 148.72 * (T0 - 273.16);
	Kw = 155.2e-7 * 4810 * e * (hw - H) / SQR(T);
	Kd = 155.2e-7 * p * (hd - H) / T;
	double Edeg = E * Rad2Deg;
	double md = 1.0 / sin(Deg2Rad * sqrt(SQR(Edeg) + 6.25));
	double mw = 1.0 / sin(Deg2Rad * sqrt(SQR(Edeg) + 2.25));

	return Kd * md + Kw * mw;
}
/********************************************************************************
	函数名称：DetectOutlier

	函数目的：粗差探测

	设计要求：
		一个历元中有多颗卫星，通过循环对其中一颗卫星进行粗差探测。
		函数内部可先定义MWGF CurComObs[MAXCHANNUM];
		用于存放当前历元的计算结果，EPOCHOBSDATA结构体中ComObs保存上个历元的平滑结果。

	输入参数：
		obs

	输出参数：
		
********************************************************************************/

void DetectOutlier(EPOCHOBSDATA* obs) {
	// 用于存储当前历元的数据
	MWGF CurComObs[MAXCHANNUM];
	double Lmw, Lgf, Pif;
	double f1, f2;
	double dMW, dGF;

	memset(CurComObs, 0, sizeof(CurComObs));

	for (int i = 0; i < obs->SatNum; i++) {
		obs->SatObs[i].valid = false;
		// 1.检查该卫星的双频伪距和相位数据是否有效和完整，
		// 若不全或为0，将Valid标记为false，continue
		if (fabs(obs->SatObs[i].P[0]) < 1e-5 || fabs(obs->SatObs[i].P[1]) < 1e-5
			|| fabs(obs->SatObs[i].L[0]) < 1e-5 || fabs(obs->SatObs[i].L[1]) < 1e-5) {
			continue;
		}

		// 2.计算当前历元该卫星的GF和MW组合值
		if (obs->SatObs[i].System == GPS) {
			f1 = FG1_GPS;
			f2 = FG2_GPS;
		}
		else if (obs->SatObs[i].System == BDS) {
			f1 = FG1_BDS;
			f2 = FG3_BDS;
		}
		else {
			continue;
		}

		Lmw = (f1 * obs->SatObs[i].L[0] - f2 * obs->SatObs[i].L[1]) / (f1 - f2) -
			(f1 * obs->SatObs[i].P[0] + f2 * obs->SatObs[i].P[1]) / (f1 + f2);
		Lgf = obs->SatObs[i].L[0] - obs->SatObs[i].L[1];
		//对于可用的观测数据，计算伪距的IF组合观测值，用于SPP
		Pif = (SQR(f1) * obs->SatObs[i].P[0] - SQR(f2) * obs->SatObs[i].P[1]) / (SQR(f1) - SQR(f2));

		// 记录当前历元的MWGF
		CurComObs[i].GF = Lgf;
		CurComObs[i].MW = Lmw;
		CurComObs[i].PIF = Pif;
		CurComObs[i].Prn = obs->SatObs[i].Prn;
		CurComObs[i].System = obs->SatObs[i].System;
		CurComObs[i].n = 1;

		bool found = false;

		for (int j = 0; j < MAXCHANNUM; j++) {
			// 3.从上个历元的MWGF数据中查找该卫星的GF和MW组合值
			if (obs->SatObs[i].System == obs->CombObs[j].System &&
				obs->SatObs[i].Prn == obs->CombObs[j].Prn) {
				found = true;
				//计算当前历元该卫星GF与上一历元对应GF的差值dGF
				//计算当前历元该卫星MW与上一历元对应MW平滑值的差值dMW
				dGF = fabs(CurComObs[i].GF - obs->CombObs[j].GF);
				dMW = fabs(CurComObs[i].MW - obs->CombObs[j].MW);

				//检查dGF和dMW是否超限，限差阈值建议为5cm和3m。若超限，标记为粗差，将Valid标记为
				//false ，若不超限，标记为可用将Valid标记为true，并计算该卫星的MW平滑值
				if (dGF < GF_Threshold && dMW < MW_Threshold) {
					obs->SatObs[i].valid = true;
					// 计算平滑后的MW
					CurComObs[i].MW = ((CurComObs[i].n - 1) * obs->CombObs[j].MW + CurComObs[i].MW) / CurComObs[i].n;
					CurComObs[i].n += 1;
				}
				else {
					obs->SatObs[i].valid = false;
					CurComObs[i].n = 0;
					continue;
					break;
				}
			}
			//如果没有上一个历元的MW/GF历史，无法做差分比较，先认为是可用卫星
			if (!found) {
				obs->SatObs[i].valid = true;        
			}
		}
		// 所有卫星循环计算完成之后，将CurComObs内存拷贝到ComObs，
		// 即函数运行结束后，ComObs保存了当前历元的GF和MW平滑值
		memcpy(obs->CombObs, CurComObs, sizeof(CurComObs));
	}
}