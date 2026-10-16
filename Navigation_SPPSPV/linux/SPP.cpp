#include "GPS_Structs.h"
#include <cmath>
#include <limits>
using namespace std;
/******************************************************************************************
	函数名称：SPP

	函数目的：单点定位

	注意事项：1.矩阵的维数；2.只有GPS或者只有BDS的时候平移列；3.n<dim的时候不能进行解算
		

	输入参数：eph,obs
		

	输出参数：userpos
		
*********************************************************************************************/

bool SPP(GPSEPH* geph, GPSEPH* beph, SPPPOS *UserPos, EPOCHOBSDATA* obs) {
	int i = 0;
	// 设定初始位置XR，设初始的接收机钟差RcvClk
	double blh[3], dPos[3],Rou;
	double sigma2;
	int GPSNum = 0, BDSNum = 0, n;
	
	// 构造观测矩阵
	MatrixXd B(MAXCHANNUM, 5), BTB(5, 5), BTW(5, 1);
	VectorXd w(MAXCHANNUM), X(5),X0(5),V(MAXCHANNUM);

	if (fabs(UserPos->Pos[0]) < 1e-6 && fabs(UserPos->Pos[1]) < 1e-6 && fabs(UserPos->Pos[2]) < 1e-6) {
		//X0 << 1000.0, 1000.0, 1000.0, 0.0, 0.0;
		X0 << -2267335.1440, 5008648.2444, 3222378.1781, 0.0, 0.0;
	}
	else {
		X0 << UserPos->Pos[0], UserPos->Pos[1], UserPos->Pos[2],
			UserPos->RcvClk[0] * CLIGHT, UserPos->RcvClk[1] * CLIGHT;
	}

	//计算发射时刻的卫星位置和卫星钟差（包含地球自转改正）
	for(i = 0 ; i < 10 ; i++ ){   // 最多循环迭代15次
		X.setZero();
		B.setZero();
		BTB.setZero();
		BTW.setZero();
		w.setZero();
		V.setZero();

		UserPos->Pos[0] = X0(0);
		UserPos->Pos[1] = X0(1);
		UserPos->Pos[2] = X0(2);
		UserPos->RcvClk[0] = X0(3);
		UserPos->RcvClk[1] = X0(4);

		SatPVTCalculation(geph, beph, obs, UserPos);
		n = GPSNum = BDSNum = 0;
		sigma2 = 0;

		// 卫星位置计算失败、观测数据不完整或有粗差，不参与定位计算
		for (int j = 0; j < obs->SatNum; j++) {
			if (obs->SatPVT[j].Valid == false || obs->SatObs[j].valid == false) {
				continue;
			}

			// 保留初始位置，作为后面的计算的参考值
			for (int k = 0; k < 3; k++) {
				dPos[k] = X0(k) - obs->SatPVT[j].SatPos[k];
			}

			Rou = sqrt(SQR(dPos[0]) + SQR(dPos[1]) + SQR(dPos[2]));

			B(n, 0) = dPos[0] / Rou;
			B(n, 1) = dPos[1] / Rou;
			B(n, 2) = dPos[2] / Rou;

			w(n) = obs->CombObs[j].PIF - (Rou - obs->SatPVT[j].SatClkOft* CLIGHT + obs->SatPVT[j].TropCorr);
			
			if (obs->SatObs[j].System == GPS) {
				B(n, 3) = 1.0;
				B(n, 4) = 0.0;
				w(n) -= X0(3);
				GPSNum++;
			}
			else if (obs->SatObs[j].System == BDS) {
				B(n, 3) = 0.0;
				B(n, 4) = 1.0;
				w(n) = w(n) - X0(4) /* - (beph + obs->SatObs[j].Prn - 1)->tgd1 * SQR(FG1_BDS) / (SQR(FG1_BDS) - SQR(FG3_BDS))*/;
				BDSNum++;
			}
			else continue;
			n++;
		}
		int dim = 5;
		//if (GPSNum == 0 || BDSNum == 0) {
		//	dim = 4;
		//}
		if (n < 4) {
			return false;
		}
		BTB = B.block(0, 0, n, 5).transpose() * B.block(0, 0, n, 5);
		BTW = B.block(0, 0, n, 5).transpose() * w.head(n);

		if (GPSNum == 0) {
			for (int k = 0; k < 5; k++) {
				BTB(k, 3) = BTB(k, 4);
			}
			for (int k = 0; k < 5; k++) {
				BTB(3, k) = BTB(4, k);
			}
			BTW(3, 0) = BTW(4, 0);
			dim--;
			for (int k = 0; k < n; k++) {
				B(k, 3) = B(k, 4);
			}
		}
		else if (BDSNum == 0) {
			dim--;
		}
		else;

		X = BTB.block(0, 0, dim, dim).inverse() * BTW.block(0, 0, dim, 1);

		if (X.norm() < 1e-4) {
			break;
		}

		//更新用户位置
		for (int k = 0; k < 3; k++) {
			X0(k) += X(k);
		}

		UserPos->SigmaPos  = UserPos->PDOP = 0;
		V = B.block(0, 0, n, dim) * X.head(dim) - w.head(n);

		for (int k = 0; k < n; k++) {
			sigma2 += V(k) * V(k);
		}	

		if (n <= dim) {
			UserPos->SigmaPos = 0.0;
		}
		else {
			UserPos->SigmaPos = sqrt(sigma2 / (n - dim));
		}

		MatrixXd Q = BTB.block(0, 0, dim, dim).inverse();

		UserPos->PDOP = sqrt(BTB.block(0, 0, dim, dim).inverse()(0, 0) +
			BTB.block(0, 0, dim, dim).inverse()(1, 1) +
			BTB.block(0, 0, dim, dim).inverse()(2, 2));

		//更新接收机钟差
		if (dim == 5) {
			X0(3) += X(3);
			X0(4) += X(4);
		}
		else if (dim == 4) {
			if (GPSNum == 0) {
				X0(4) += X(3);
			}
			else if (BDSNum == 0) {
				X0(3) += X(3);
			}
		}
		
		BTB.setZero();
		BTW.setZero();
		w.setZero();
		V.setZero();
	}
	UserPos->Pos[0] = X0(0);
	UserPos->Pos[1] = X0(1);
	UserPos->Pos[2] = X0(2);
	UserPos->RcvClk[0] = X0(3) / CLIGHT;
	UserPos->RcvClk[1] = X0(4) / CLIGHT;
	UserPos->SatNum = n;

	return true;
}

/******************************************************************************************
	函数名称：SPV

	函数目的：单点测速

	注意事项：


	输入参数：


	输出参数：

*********************************************************************************************/
void SPV(EPOCHOBSDATA* obs, SPPPOS* UserPos) {
	int n;
	double dPos[3],Rou,Rou_dot;
	
	UserPos->Time = obs->Time;
	n = obs->SatNum;

	// 建立矩阵B
	MatrixXd B(n, 4), BTPB(4, 4), BTPW(4, 1), Q(4, 4);
	VectorXd w(n), X(4),v(n);

	B.setZero();
	w.setZero();
	int m = 0;
	for (int i = 0; i < obs->SatNum; i++) {
		if (obs->SatPVT[i].Valid == false || obs->SatObs[i].valid == false) {
			continue;
		}

		// 计算接收机到卫星的位置
		dPos[0] = UserPos->Pos[0] - obs->SatPVT[i].SatPos[0];
		dPos[1] = UserPos->Pos[1] - obs->SatPVT[i].SatPos[1];
		dPos[2] = UserPos->Pos[2] - obs->SatPVT[i].SatPos[2];
		Rou = sqrt(SQR(dPos[0]) + SQR(dPos[1]) + SQR(dPos[2]));

		// 计算卫星的相对速度Rou_dot
		Rou_dot = -(dPos[0] * obs->SatPVT[i].SatVel[0] +
			dPos[1] * obs->SatPVT[i].SatVel[1] +
			dPos[2] * obs->SatPVT[i].SatVel[2]) / Rou;

		//构造观测矩阵
		B(m, 0) = dPos[0] / Rou;
		B(m, 1) = dPos[1] / Rou;
		B(m, 2) = dPos[2] / Rou;
		B(m, 3) = 1.0;

		// 计算残差
		w(m) = obs->SatObs[i].D[0] - (Rou_dot - CLIGHT * obs->SatPVT[i].SatClkSft);
		m++;
	}
	if (m < 4) {
		return;
	}
	BTPB = B.block(0,0,m,4).transpose() * B.block(0, 0, m, 4);
	BTPW = B.block(0, 0, m, 4).transpose() * w.head(m);
	X = BTPB.inverse() * BTPW;
	
	// 计算残差
	v = B.block(0, 0, m, 4) * X - w.head(m);
	UserPos->SigmaVel = sqrt((v.transpose() * v)(0, 0) / (m - 4));
	Q = BTPB.inverse();
	UserPos->Dxx = UserPos->SigmaVel * Q;

	UserPos->Vel[0] = X(0);
	UserPos->Vel[1] = X(1);
	UserPos->Vel[2] = X(2);
}