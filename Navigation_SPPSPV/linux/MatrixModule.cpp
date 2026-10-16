#include "GPS_Structs.h"
using namespace Eigen;
Matrix3d BLH2ENUMat(double blh[3]) {
	Matrix3d R;
	R <<
		-sin(blh[1]), cos(blh[1]), 0,
		-sin(blh[0]) * cos(blh[1]), -sin(blh[0]) * sin(blh[1]), cos(blh[0]),
		cos(blh[0])* cos(blh[1]), cos(blh[0])* sin(blh[1]), sin(blh[0]);
	return R;
}
Matrix3d Rx(double theta) {
	Matrix3d R;
	R <<
		1, 0, 0,
		0, cos(theta), sin(theta),
		0, -sin(theta), cos(theta);
	return R;
}

Matrix3d Rz(double theta) {
	Matrix3d R;
	R <<
		cos(theta), sin(theta), 0,
		-sin(theta), cos(theta), 0,
		0, 0, 1;
	return R;
}

Matrix3d Rz_dot(double theta) {
	Matrix3d R;
	R <<
		-sin(theta), cos(theta), 0,
		-cos(theta), -sin(theta), 0,
		0, 0, 0;
	return R;
}