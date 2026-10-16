#include "GPS_Structs.h"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

double Distance(double X1[3], double X2[3]) {
	return sqrt(SQR(X1[0] - X2[0]) + SQR(X1[1] - X2[1]) + SQR(X1[2] - X2[2]));
}

void BLHToXYZ(const double blh[3], double  xyz[3], const double R, const double F) {
	double e2 = 2 * F - F * F;
	double n = R / sqrt(1 - e2 * SQR(sin(blh[0])));

	xyz[0] = (n + blh[2]) * cos(blh[0]) * cos(blh[1]);
	xyz[1] = (n + blh[2]) * cos(blh[0]) * sin(blh[1]);
	xyz[2] = (n * (1 - e2) + blh[2]) * sin(blh[0]);
}

void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F) {
	double e2 = 2.0 * F - F * F;
	double p = sqrt(SQR(xyz[0]) + SQR(xyz[1]));

	blh[1] = atan2(xyz[1], xyz[0]) /** Rad2Deg*/;

	if (p < 1E-12) {
		blh[0] = (xyz[2] >= 0.0 ? PI / 2.0 : -PI / 2.0);
		blh[2] = fabs(xyz[2]) - R * sqrt(1.0 - e2);
		return;
	}

	double delta_z = e2 * xyz[2];
	double dz = 0.0;
	double sinB = 0.0;
	double n = 0.0;

	blh[0] = atan((xyz[2] + delta_z)/ p);

	while (true) {
		dz = delta_z;
		blh[0] = atan((xyz[2] + delta_z) / p);
		sinB = sin(blh[0]);
		n = R / sqrt(1.0 - e2 * SQR(sinB));
		delta_z = n * e2 * sinB;
		if (fabs(delta_z - dz) < 1E-12) {
			break;
		}
	}
	
	blh[0] = atan2(xyz[2] + delta_z, p) /** Rad2Deg*/;
	n = R / sqrt(1.0 - e2 * SQR(sin(blh[0])));
	blh[2] = sqrt(SQR(xyz[0]) + SQR(xyz[1]) + SQR(xyz[2] + delta_z)) - n;
}


void CompEnudPos(const double X0[], const double Xr[], double dENU[], const double R, const double F) {
	double blh[3];
	XYZToBLH(X0, blh, R, F);

	Vector3d dxyz;
	dxyz <<
		Xr[0] - X0[0],
		Xr[1] - X0[1],
		Xr[2] - X0[2];

	Vector3d denu = BLH2ENUMat(blh) * dxyz;

	dENU[0] = denu[0];
	dENU[1] = denu[1];
	dENU[2] = denu[2];
}

void CompSatElAz(const double Xr[], const double Xs[], double R,double F,double* Elev, double* Azim) {
	// ¼ÆËã²âÕ¾µ½ÎÀÐÇµÄ¾àÀë
	Vector3d dx;
	dx <<
		Xr[0] - Xs[0],
		Xr[1] - Xs[1],
		Xr[2] - Xs[2];
	
	double blh[3], enu[3];
	XYZToBLH(Xr, blh, R, F);
	Vector3d de = BLH2ENUMat(blh) * ( - dx);
	enu[0] = de[0];
	enu[1] = de[1];
	enu[2] = de[2];

	*Elev = atan(enu[2]/ sqrt(SQR(enu[1])+SQR(enu[0])));
	*Azim = atan2(enu[0], enu[1]);
	if (*Azim < 0.0) {
		*Azim += 2.0 * PI;
	}
}


