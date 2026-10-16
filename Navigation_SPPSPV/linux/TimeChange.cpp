#include<iostream>
#include <cmath>

#include "GPS_Structs.h"

void CommonTimeToMJDTime(const COMMONTIME* ct, MJDTIME* mjd) {
	double y, m;
	if (ct->Month <= 2) {
		y = ct->Year - 1;
		m = ct->Month + 12;
	}
	else {
		y = ct->Year;
		m = ct->Month;
	}
	mjd->Days = int(365.25 * y) + int(30.6001 * (m + 1)) + ct->Day
		+ 1720981.5 - 2400000.5;
	
	//double ut = ct->Hour / 24.0 + ct->Minute / 1440.0 + ct->Second / 86400.0;
	// 设计两个变量是为了保证精度，不能直接用两个变量再分开，这样的话double不能完全保证精度
	double ut = ct->Hour + ct->Minute / 60.0 + ct->Second / 3600.0;
	mjd->FracDay = ut / 24;
}

void MJDTimeToCommonTime(const MJDTIME* mjd, COMMONTIME* ct) {
	int a, b, c, d, e;
	a = int(mjd->Days + 2400000.5 + 0.5);
	b = a + 1537;
	c = int((b - 122.1) / 365.25);
	d = int(365.25 * c);
	e = int((b - d) / 30.6001);

	ct->Day = b - d - int(30.6001 * e);
	ct->Month = e - 1 - 12 * int(e / 14);
	ct->Year = c - 4715 - int((7 + ct->Month) / 10);

	ct->Hour = int(mjd->FracDay * 24.0);
	ct->Minute = int((mjd->FracDay * 24.0 - ct->Hour) * 60);
	ct->Second = ((mjd->FracDay * 24.0 - ct->Hour) * 60 - ct->Minute) * 60;
}

void MJDTimeToGPSTime(const MJDTIME* mjd, GPSTIME* gt) {
	gt->Week = int((mjd->Days - 44244) / 7.0);
	gt->SecOfWeek = (mjd->Days + mjd->FracDay - 44244 - gt->Week * 7) * 86400;
}

void GPSTimeToMJDTime(const GPSTIME* gt, MJDTIME* mjd) {
	double day = gt->SecOfWeek / 86400.0;
	mjd->Days = 44244 + gt->Week * 7 + int(day);
	mjd->FracDay = day - int(day);
}

void CommonTimeToGPSTime(const COMMONTIME* ct, GPSTIME* gt) {
	MJDTIME mjd;
	CommonTimeToMJDTime(ct, &mjd);
	MJDTimeToGPSTime(&mjd, gt);
}

void GPSTimeToCommonTime(const GPSTIME* gt, COMMONTIME* ct) {
	MJDTIME mjd;
	GPSTimeToMJDTime(gt, &mjd);
	MJDTimeToCommonTime(&mjd, ct);
}