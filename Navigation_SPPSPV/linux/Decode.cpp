#include "GPS_Structs.h"

double D8(unsigned char* p)
{
	double r;
	memcpy(&r, p, 8);
	return r;
}

float F4(unsigned char* p)
{
	float r;
	memcpy(&r, p, 4);
	return r;
}

int I4(unsigned char* p)
{
	int r;
	memcpy(&r, p, 4);
	return r;
}

unsigned int UI4(unsigned char* p)
{
	unsigned int r;
	memcpy(&r, p, 4);
	return r;
}

short S2(unsigned char* p)
{
	short r;
	memcpy(&r, p, 2);
	return r;
}

unsigned short US2(unsigned char* p)
{
	unsigned short r;
	memcpy(&r, p, 2);
	return r;
}

/********************************************************************************
	函数名称：crc32

	函数目的：CRC校验，用于判断当前消息在传输或者存储过程中是否有损坏，只有CRC通过，
			才说明这条消息可靠

	输入参数
		buff  缓冲区的地址
		len   缓冲区的数据的长度

	输出参数
		CRC校验码
********************************************************************************/
unsigned int crc32(const unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;
	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
			else crc >>= 1;
		}
	}
	return crc;
}

/********************************************************************************
	函数名称：DecodeNovOem7Dat

	函数目的：板卡解码主函数

	设计要求：
		1. 输入buff缓冲区，总字节数len，buff中可能有多组消息，循环解码每一条消息，
		在文件模式下，解码得到观测值后，函数返回；
		在实时模式下，解码所有消息，直到剩余的字节不能组成一条消息。
		2. 剩余的字节要和下一组要读取的字节组成一个buff，避免数据丢失

	输入参数：
		buff    缓冲区地址
		len     缓冲区数据的长度

	输出参数：
		obs     观测值
		geph    GPS广播星历
		beph    BDS广播星历
		Type    解码的种类 
********************************************************************************/
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBSDATA* obs, GPSEPH geph[], GPSEPH beph[]/*POSRES* pos*/) {
		uint16_t MsgID = 0;
		uint16_t MsgLen = 0;
		unsigned char Msg[10240];
		int Type = 0;

		int i = 0;
	while (true) {
		// 1.设置循环变量i=0，开始查找AA 44 12同步字符
		for (; i < len; i++) {
			if (buff[i] == 0xAA && buff[i + 1] == 0x44 && buff[i + 2] == 0x12) 
			{
				break;
			}
		}
		
		// 2.找到同步字符后，获取消息头长度的字符28字节，若字节数量不足即 i + 28 > len，跳出循环（break）至第6步
		if ((i + 28) > len) 
		{
			break;
		}
		memcpy(Msg, buff + i, 28);
		// 3.从消息头中解码消息长度MsgLen和消息类型MsgID，获得整条消息buff[i, i + 28 + MsgLen + 4]，
		// 若字节数量不足即i + 28 + MsgLen + 4 > len，跳出循环（break）至第6步
		MsgID = US2(Msg+ 4);// 解码MsgID
		MsgLen = US2(Msg + 8); //解码MsgLen
		if ((i + 28 + MsgLen + 4) > len)
		{
			break;
		}
		memcpy(Msg + 28, buff + i + 28, MsgLen + 4);

		// 4.CRC检验，若不通过，跳过同步字符3个字节，即i=i+3，返回到第1步
		if (crc32(Msg, 28 + MsgLen) != UI4(Msg + 28 + MsgLen)) {
			i = i + 3;
			continue;
		}

		// 5.根据消息ID，调用对应的解码函数，若解码得到观测值，跳出循环至第6步；否则，跳过整条消息，即i = i + 28 + MsgLen + 4，返回第1步
		switch (MsgID)
		{
		case 43:
			Type = decode_rangeb_oem7(buff + i, obs);
			break;
		case 7:
			Type = decode_gpsephem(buff + i, geph);
			break;
		case 1696:
			Type = decode_bdsephem(buff + i, beph);
			break;
		default:
			break;
		}
		i = i + 28 + MsgLen + 4;
		if (MsgID == 43 && Type == 1) {
			break;
		}

		if (FILEMODE == 1 && Type == 1) {
			break;
		}
	}
	// 6.循环结束后，将不足一条消息的剩余字节，拷贝至buff缓冲区的开始处，将len设置为剩余字节数量，并返回给主函数。
	// 因为一次读取的数据可能刚好是截断在半条消息中间。如果把这部分丢掉，下一次就拼不成完整的消息了
	memcpy(buff, buff + i, len - i);
	len = len - i;
	return Type;
}

/********************************************************************************
	函数名称：decode_rangeb_oem7

	函数目的：观测数据解码函数

	设计要求：
		输入buff缓冲区，buff是一条完整的Rangeb消息，解码得到一个历元所有卫星所有信号的观测值后，函数返回。

	输入参数：
		buff    缓冲区地址
		len     缓冲区数据的长度

	输出参数：
		1       解码成功
		0       解码失败
********************************************************************************/
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBSDATA* obs) {
	//把所有的int类型的写到一行里面
	int ObsNum, i = 0, Freq, prn, m = 0, count = 0;
	double wl;
	unsigned char* p = buff + 32;
	unsigned int ch_tr_status;    //都写int
	int PhaseLockFlag, CodeLockedFlag, ParityFlag, SatSystem, SignalType;
	GNSSSYS sys;
	bool FindFlag;


	// 1. 从消息头中解码得到观测时刻，该时刻为接收机钟表面时，用GPSTIME结构体表示
	obs->Time.Week = US2(buff + 14);
	obs->Time.SecOfWeek = UI4(buff + 16) / 1000.0;

	// 2.解码得到观测值数量，为所有卫星所有信号观测值的总数
	ObsNum = UI4(buff + 28);
	obs->SatNum = 0;
	memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));    //将观测值初始化为0,清空之前的数据

	// 3. 对所有信号观测值进行循环解码，字节累加44个
	// ① 解码得到跟踪状态标记，从中取出Phase lock flag/Code locked flag /
	//  Parity known flag / Satellite system / signal type等数据
	for (i = 0/*, p += 4*/; i < ObsNum; i++,p+=44) {
		ch_tr_status = UI4(p + 40);
		PhaseLockFlag = (ch_tr_status >> 10) & 0x01;   //相位跟踪锁定
		ParityFlag = (ch_tr_status >> 11) & 0x01;      //半周状态
		CodeLockedFlag = (ch_tr_status >> 12) & 0x01;  //伪距相位观测值的跟踪状态
		SatSystem = (ch_tr_status >> 16) & 0x07;       //卫星类型  
		SignalType = (ch_tr_status >> 21) & 0x1F;      //信号类型
		prn = US2(p);

		// ② 如果卫星系统不是GPS或BDS，continue至①
		if (SatSystem != 4 && SatSystem != 0) {    //0=GPS;4=BDS
			continue;
		}
		// ③ 如果GPS卫星的信号类型不是L1 C/A或者L2P（Y），BDS卫星不是B1I或B3I，continue至①，
		// 否则记录信号频率类型，第一频率s = 0，第二频率s = 1；
		if (SatSystem == 0) {
			sys = GPS;
			if (SignalType == 0) {
				Freq = 0;
				wl = WL1_GPS;
			}
			else if (SignalType == 9) {
				Freq = 1;
				wl = WL2_GPS;
			}
			else {
				continue;
			}
		}

		else if (SatSystem == 4) {
			sys = BDS;
			if (SignalType == 0 || SignalType == 4) {
				Freq = 0;
				wl = WL1_BDS;
			}
			else if (SignalType == 2 || SignalType == 6) {
				Freq = 1;
				wl = WL3_BDS;
			}
			else {
				continue;
			}
		}

		else continue;

		// ④ 解码得到卫星号Prn以及卫星系统号，在当前观测值结构体中进行搜索，
		// 如果找到相同的卫星，将解码的观测值填充到该卫星对应的数组中；
		// 如果在当前已解码的卫星数据中没有发现，则填充到现有数据的末尾
		// 同一颗卫星可能有多个频点的观测，应该放到同一个SATOBSDATA里面，
		// 如果找到已有记录，直接把第二个频点补到一个结构体中，如果没有再重新放
		FindFlag = false;
		for (int j = 0; j < MAXCHANNUM; j++) {
			if (obs->SatObs[j].System == sys && obs->SatObs[j].Prn == prn) {
				m = j;
				FindFlag = true;
				break;
			}
			// 如果什么都没有，直接放
			if (obs->SatObs[j].Prn == 0) {
				m = j;                  
				FindFlag = true;
				break;
			}
		}
		if (FindFlag == false) {
			continue;
		}
		obs->SatObs[m].Prn = prn;
		obs->SatObs[m].System = sys;
		obs->SatObs[m].P[Freq] = (CodeLockedFlag == 1 ? D8(p + 4) : 0.0);
		obs->SatObs[m].L[Freq] = -wl * (PhaseLockFlag == 1 ? D8(p + 16) : 0.0);
		obs->SatObs[m].D[Freq] = -wl * F4(p + 28);
		obs->SatObs[m].LockTime[Freq] = F4(p + 36);
		obs->SatObs[m].cn0[Freq] = F4(p + 32);
		obs->SatObs[m].half[Freq] = ParityFlag;
	}
	// 4. 统计有效卫星数量
	for (i = 0; i < MAXCHANNUM; i++) {
		if (obs->SatObs[i].System != UNKNOWN && obs->SatObs[i].Prn != 0) {
			obs->SatNum++;
		}
	}
	
	return 1;
}


/********************************************************************************
	函数名称：decode_gpsephem

	函数目的：GPS星历数据解码函数

	设计要求：
		输入buff缓冲区，buff是一条完整的Rangeb消息，解码得到一个历元所有卫星所有信号的观测值后，函数返回。
		输入buff缓冲区，buff是一条完整的GPS或BDS消息，解码得到一颗卫星的广播星历，函数返回。
		Eph为星历数组，存储所有卫星GPS或BDS卫星的广播星历，
		GPS最大卫星数为32，BDS最大卫星数为63。不同卫星号的星历存储位置为数组元素i=prn-1


	输入参数：
		buff    缓冲区地址
		len     缓冲区数据的长度

	输出参数：
		2       解码成功
		0       解码失败
********************************************************************************/
int decode_gpsephem(unsigned char* buff, GPSEPH* geph) {
	int prn;
	unsigned char* p = buff + 28;

	//1. 解码得到卫星号prn，判断prn-1是否超出Eph数组范围。如果越界函数返回。
	prn = UI4(p);
	if (prn<1 || prn>MAXGPSNUM) {
		return 0;
	}

	//2. 根据OEM星历格式，解码星历数据，注意各数据的单位。
	GPSEPH* eph = geph + prn - 1;   //确定不同卫星号的星历存储位置

	eph->PRN = prn;
	eph->System = GPS;
	eph->TOC.Week = eph->TOE.Week = UI4(p + 24);
	eph->TOE.SecOfWeek = D8(p + 32);
	eph->TOC.SecOfWeek = D8(p + 164);

	eph->a0 = D8(p + 180);
	eph->a1 = D8(p + 188);
	eph->a2 = D8(p + 196);

	eph->IODE = UI4(p + 16);
	eph->IODC = UI4(p + 160);

	eph->sqrt_A = sqrt(D8(p + 40));
	eph->M0 = D8(p + 56);
	eph->e = D8(p + 64);
	eph->OMEGA = D8(p + 144);
	eph->i0 = D8(p + 128);
	eph->omega = D8(p + 72);

	eph->Cic = D8(p + 112);
	eph->Cis = D8(p + 120);
	eph->Crc = D8(p + 96);
	eph->Crs = D8(p + 104);
	eph->Cuc = D8(p + 80);
	eph->Cus = D8(p + 88);

	eph->deltaN = D8(p + 48);
	eph->OMEGA_dot = D8(p + 152);

	eph->SVhealth = UI4(p + 12);
	eph->IDOT = D8(p + 136);
	eph->tgd1 = D8(p + 172);
	
	return 2;
}


/********************************************************************************
	函数名称：decode_bdsephem

	函数目的：BDS星历数据解码函数

	设计要求：
		输入buff缓冲区，buff是一条完整的Rangeb消息，解码得到一个历元所有卫星所有信号的观测值后，函数返回。
		输入buff缓冲区，buff是一条完整的GPS或BDS消息，解码得到一颗卫星的广播星历，函数返回。
		Eph为星历数组，存储所有卫星GPS或BDS卫星的广播星历，
		GPS最大卫星数为32，BDS最大卫星数为63。不同卫星号的星历存储位置为数组元素i=prn-1


	输入参数：
		buff    缓冲区地址
		len     缓冲区数据的长度

	输出参数：
		3       解码成功
		0       解码失败
********************************************************************************/
int decode_bdsephem(unsigned char* buff, GPSEPH* beph) {
	int prn;
	unsigned char* p = buff + 28;

	//1. 解码得到卫星号prn，判断prn-1是否超出Eph数组范围。如果越界函数返回。
	prn = UI4(p);
	if (prn<1 || prn>MAXBDSNUM) {
		return 0;
	}

	//2. 根据OEM星历格式，解码星历数据，注意各数据的单位。
	GPSEPH* eph = beph + prn - 1;   //确定不同卫星号的星历存储位置

	eph->PRN = prn;
	eph->System = BDS;
	eph->TOC.Week = eph->TOE.Week = UI4(p + 4);

	//BDS星历的参考时间TOC和TOE均定义为GPSTIME结构体，注意它们的时间系统。
	eph->TOE.SecOfWeek = UI4(p + 72);
	eph->TOC.SecOfWeek = UI4(p + 40);

	eph->a0 = D8(p + 44);
	eph->a1 = D8(p + 52);
	eph->a2 = D8(p + 60);

	eph->IODE = UI4(p + 68);
	eph->IODC = UI4(p + 36);

	eph->sqrt_A = D8(p + 76);
	eph->M0 = D8(p + 108);
	eph->e = D8(p + 84);
	eph->OMEGA = D8(p + 116);
	eph->i0 = D8(p + 132);
	eph->omega = D8(p + 92);

	eph->Cic = D8(p + 180);
	eph->Cis = D8(p + 188);
	eph->Crc = D8(p + 164);
	eph->Crs = D8(p + 172);
	eph->Cuc = D8(p + 148);
	eph->Cus = D8(p + 156);

	eph->deltaN = D8(p + 100);
	eph->OMEGA_dot = D8(p + 124);

	eph->SVhealth = UI4(p + 16);
	eph->IDOT = D8(p + 140);
	eph->tgd1 = D8(p + 20);
	eph->tgd2 = D8(p + 28);

	return 3;

}

