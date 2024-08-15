#include "pch.h"
#include "SpaIntSect.h"
#include <locale.h>

CSpaIntSect::CSpaIntSect(void)
{
}

CSpaIntSect::~CSpaIntSect(void)
{
	//释放动态数组内存
	if (GCP != NULL)
	{
		delete[] GCP;
		GCP = NULL;
	}
	if (P != NULL)
	{
		delete[] P;
		P = NULL;
	}

}

void CSpaIntSect::SetPoiNum(int m,int n)
{
	GCP= new Point[m];
	P= new Point[n];
}

CString* CSpaIntSect::SplitString(CString str, char split, int& iSubStrs)//分割字符串
{
	int iPos = 0; //分割符位置
	int iNums = 0; //分割符的总数
	CString strTemp = str;
	CString strRight;
	//先计算子字符串的数量
	while (iPos != -1)
	{
		iPos = strTemp.Find(split);
		if (iPos == -1)
		{
			break;
		}
		strRight = strTemp.Mid(iPos + 1, str.GetLength());
		strTemp = strRight;
		iNums++;
	}
	if (iNums == 0) //没有找到分割符
	{
		//子字符串数就是字符串本身
		iSubStrs = 1;
		return NULL;
	}
	//子字符串数组
	iSubStrs = iNums + 1; //子串的数量 = 分割符数量 + 1
	CString* pStrSplit;
	pStrSplit = new CString[iSubStrs];
	strTemp = str;
	CString strLeft;
	for (int i = 0; i < iNums; i++)
	{
		iPos = strTemp.Find(split);
		//左子串
		strLeft = strTemp.Left(iPos);
		//右子串
		strRight = strTemp.Mid(iPos + 1, strTemp.GetLength());
		strTemp = strRight;
		pStrSplit[i] = strLeft;
	}
	pStrSplit[iNums] = strTemp;
	return pStrSplit;
}

bool CSpaIntSect::LoadObsData(const CString strFileName, CString& strObsData)//读取数据
{
	//读取数据
	CStdioFile sf;
	if (!sf.Open(strFileName, CFile::modeRead)) return 0;
	CString strLine;
	strLine.Empty();
	//控制点及待定点个数
	BOOL bEOF = sf.ReadString(strLine);
	int tmp,m,n,o;
	CString* pstrData = SplitString(strLine, ',', tmp);
	for (int j = 0; j < tmp; j++)
		pstrData[j].Trim();
	iCountGCP = _ttoi(pstrData[0]);
	iCountP = _ttoi(pstrData[1]);
	SetPoiNum(iCountGCP, iCountP);
	//读取控制点
	for (int i = 0; i < iCountGCP; i++)
	{
		sf.ReadString(strLine);
		pstrData = SplitString(strLine, ',', m);
		for (int j = 0; j < m; j++)
			pstrData[j].Trim();
		GCP[i].name = pstrData[0];
		GCP[i].ImgLeft.x = _tstof(pstrData[1])/1000 ;
		GCP[i].ImgLeft.y = _tstof(pstrData[2])/1000 ;
		GCP[i].ImgRight.x = _tstof(pstrData[3])/1000 ;
		GCP[i].ImgRight.y = _tstof(pstrData[4])/1000 ;
		GCP[i].Obj.X = _tstof(pstrData[5]);
		GCP[i].Obj.Y = _tstof(pstrData[6]);
		GCP[i].Obj.Z = _tstof(pstrData[7]);
	}
	//读取待定点
	for (int i = 0; i < iCountP; i++)
	{
		sf.ReadString(strLine);
		pstrData = SplitString(strLine, ',', n);
		for (int j = 0; j < n; j++)
			pstrData[j].Trim();
		P[i].name = pstrData[0];
		P[i].ImgLeft.x = _tstof(pstrData[1]) / 1000;
		P[i].ImgLeft.y = _tstof(pstrData[2]) / 1000;
		P[i].ImgRight.x = _tstof(pstrData[3]) / 1000;
		P[i].ImgRight.y = _tstof(pstrData[4]) / 1000;

	}
	//内方位元素
	sf.ReadString(strLine);
	 pstrData = SplitString(strLine, ',', o);
	for (int k = 0;k < o; k++)
		pstrData[k].Trim();
	f0 = _tstof(pstrData[0]) / 1000;
	x0 = _tstof(pstrData[1]) / 1000;
	y0 = _tstof(pstrData[2]) / 1000;
	delete[] pstrData;
	pstrData = NULL;
	sf.Close();
}

void CSpaIntSect::FormLeftErrorEquations(CMatrix& BLeft , CMatrix& LLeft)//组成左片误差方程,B 为系数矩阵，L为常数项向量
{
	double La1, Lb1, Lc1, La2, Lb2, Lc2, La3, Lb3, Lc3;
	//旋转矩阵系数
	La1 = cos(Lφ) * cos(Lκ) - sin(Lφ) * sin(Lκ) * sin(Lω);
	La2 = -cos(Lφ) * sin(Lκ) - sin(Lφ) * cos(Lκ) * sin(Lω);
	La3 = -sin(Lφ) * cos(Lω);
	Lb1 = cos(Lω) * sin(Lκ);
	Lb2 = cos(Lω) * cos(Lκ);
	Lb3 = -sin(Lω);
	Lc1= sin(Lφ) * cos(Lκ) + cos(Lφ) * sin(Lκ) * sin(Lω);
	Lc2 = -sin(Lφ) * sin(Lκ) + cos(Lφ) * cos(Lκ) * sin(Lω);
	Lc3= cos(Lφ) * cos(Lω);

	mLeft = sqrt(pow(GCP[0].Obj.X - GCP[1].Obj.X, 2) + pow(GCP[0].Obj.Y - GCP[1].Obj.Y, 2)) / sqrt(pow(GCP[0].ImgLeft.x - GCP[1].ImgLeft.x, 2) + pow(GCP[0].ImgLeft.y - GCP[1].ImgLeft.y, 2));
	HLeft = mLeft * f0;

	for (int i = 0; i < iCountGCP; i++)
	{
		BLeft(2 * i, 0) = -f0 / HLeft * cos(Lκ);
		BLeft(2 * i, 1) = -f0 / HLeft * sin(Lκ);
		BLeft(2 * i, 2) = -(GCP[i].ImgLeft.x - x0) / HLeft;
		BLeft(2 * i, 3) = -(f0 + pow(GCP[i].ImgLeft.x - x0, 2) / f0) * cos(Lκ) + (GCP[i].ImgLeft.x - x0)*(GCP[i].ImgLeft.y - y0) * sin(Lκ)/f0;
		BLeft(2 * i, 4) = -(GCP[i].ImgLeft.x - x0) * (GCP[i].ImgLeft.y - y0) * cos(Lκ) / f0 - (f0 + pow(GCP[i].ImgLeft.x - x0, 2) / f0) * sin(Lκ);
		BLeft(2 * i, 5) = GCP[i].ImgLeft.y - y0;
		BLeft(2 * i + 1, 0) = f0 / HLeft * sin(Lκ);
		BLeft(2 * i + 1, 1) = -f0 / HLeft * cos(Lκ);
		BLeft(2 * i + 1, 2) = -(GCP[i].ImgLeft.y - y0) / HLeft;
		BLeft(2 * i + 1, 3) = -(GCP[i].ImgLeft.x - x0) * (GCP[i].ImgLeft.y - y0) * cos(Lκ) / f0 + (f0 + pow(GCP[i].ImgLeft.y - y0, 2) / f0) * sin(Lκ);
		BLeft(2 * i + 1, 4) = -(f0 + pow(GCP[i].ImgLeft.y - y0, 2) / f0) * cos(Lκ) - (GCP[i].ImgLeft.x - x0)*(GCP[i].ImgLeft.y - y0) * sin(Lκ)/f0;
		BLeft(2 * i + 1, 5) = -(GCP[i].ImgLeft.x - x0);

		LLeft(2 * i, 0) = -(x0 - f0 * (La1 * (GCP[i].Obj.X - LXs) + Lb1 * (GCP[i].Obj.Y - LYs) + Lc1 * (GCP[i].Obj.Z - LZs)) / (La3 * (GCP[i].Obj.X - LXs) + Lb3 * (GCP[i].Obj.Y - LYs) + Lc3 * (GCP[i].Obj.Z - LZs)) - GCP[i].ImgLeft.x);
		LLeft(2 * i + 1, 0) = -(y0 - f0 * (La2 * (GCP[i].Obj.X - LXs) + Lb2 * (GCP[i].Obj.Y - LYs) + Lc2 * (GCP[i].Obj.Z - LZs)) / (La3 * (GCP[i].Obj.X - LXs) + Lb3 * (GCP[i].Obj.Y - LYs) + Lc3 * (GCP[i].Obj.Z - LZs)) - GCP[i].ImgLeft.y);
	}
}

void CSpaIntSect::FormRightErrorEquations(CMatrix& BRight, CMatrix& LRight)//组成右片误差方程,B 为系数矩阵，L为常数项向量
{
	double Ra1, Rb1, Rc1, Ra2, Rb2, Rc2, Ra3, Rb3, Rc3;
	//旋转矩阵系数
	Ra1 = cos(Rφ) * cos(Rκ) - sin(Rφ) * sin(Rκ) * sin(Rω);
	Ra2 = -cos(Rφ) * sin(Rκ) - sin(Rφ) * cos(Rκ) * sin(Rω);
	Ra3 = -sin(Rφ) * cos(Rω);
	Rb1 = cos(Rω) * sin(Rκ);
	Rb2 = cos(Rω) * cos(Rκ);
	Rb3 = -sin(Rω);
	Rc1 = sin(Rφ) * cos(Rκ) + cos(Rφ) * sin(Rκ) * sin(Rω);
	Rc2 = -sin(Rφ) * sin(Rκ) + cos(Rφ) * cos(Rκ) * sin(Rω);
	Rc3 = cos(Rφ) * cos(Rω);

	mRight = sqrt(pow(GCP[0].Obj.X - GCP[1].Obj.X, 2) + pow(GCP[0].Obj.Y - GCP[1].Obj.Y, 2)) / sqrt(pow(GCP[0].ImgRight.x - GCP[1].ImgRight.x, 2) + pow(GCP[0].ImgRight.y - GCP[1].ImgRight.y, 2));
	HRight = mRight * f0;

	for (int i = 0; i < iCountGCP; i++)
	{
		BRight(2 * i, 0) = -f0 / HRight * cos(Rκ);
		BRight(2 * i, 1) = -f0 / HRight * sin(Rκ);
		BRight(2 * i, 2) = -(GCP[i].ImgRight.x - x0) / HRight;
		BRight(2 * i, 3) = -(f0 + pow(GCP[i].ImgRight.x - x0, 2) / f0) * cos(Rκ) + (GCP[i].ImgRight.x - x0) * (GCP[i].ImgRight.y - y0) * sin(Rκ) / f0;
		BRight(2 * i, 4) = -(GCP[i].ImgRight.x - x0) * (GCP[i].ImgRight.y - y0) * cos(Rκ) / f0 - (f0 + pow(GCP[i].ImgRight.x - x0, 2) / f0) * sin(Rκ);
		BRight(2 * i, 5) = GCP[i].ImgRight.y - y0;
		BRight(2 * i + 1, 0) = f0 / HRight * sin(Rκ);
		BRight(2 * i + 1, 1) = -f0 / HRight * cos(Rκ);
		BRight(2 * i + 1, 2) = -(GCP[i].ImgRight.y - y0) / HRight;
		BRight(2 * i + 1, 3) = -(GCP[i].ImgRight.x - x0) * (GCP[i].ImgRight.y - y0) * cos(Rκ) / f0 + (f0 + pow(GCP[i].ImgRight.y - y0, 2) / f0) * sin(Rκ);
		BRight(2 * i + 1, 4) = -(f0 + pow(GCP[i].ImgRight.y - y0, 2) / f0) * cos(Rκ) - (GCP[i].ImgRight.x - x0) * (GCP[i].ImgRight.y - y0) * sin(Rκ) / f0;
		BRight(2 * i + 1, 5) = -(GCP[i].ImgRight.x - x0);

		LRight(2 * i, 0) = -(x0 - f0 * (Ra1 * (GCP[i].Obj.X - RXs) + Rb1 * (GCP[i].Obj.Y - RYs) + Rc1 * (GCP[i].Obj.Z - RZs)) / (Ra3 * (GCP[i].Obj.X - RXs) + Rb3 * (GCP[i].Obj.Y - RYs) + Rc3 * (GCP[i].Obj.Z - RZs)) - GCP[i].ImgRight.x);
		LRight(2 * i + 1, 0) = -(y0 - f0 * (Ra2 * (GCP[i].Obj.X - RXs) + Rb2 * (GCP[i].Obj.Y - RYs) + Rc2 * (GCP[i].Obj.Z - RZs)) / (Ra3 * (GCP[i].Obj.X - RXs) + Rb3 * (GCP[i].Obj.Y - RYs) + Rc3 * (GCP[i].Obj.Z - RZs)) - GCP[i].ImgRight.y);
	}
}

void CSpaIntSect::RigorousAdjust()//平差函数
{
	//-----------------------------------------------平差计算步骤------------------------------------------------------
	CMatrix L_B(2 * iCountGCP, 6), L_L(2 * iCountGCP, 1);
	CMatrix R_B(2 * iCountGCP, 6), R_L(2 * iCountGCP, 1);
	CMatrix L_BT, L_N, L_InvN, L_detX, L_V, L_Qx;
	CMatrix R_BT, R_N, R_InvN, R_detX, R_V, R_Qx;

	//初始化（取近似值）

	mLeft = sqrt(pow(GCP[0].Obj.X - GCP[1].Obj.X, 2) + pow(GCP[0].Obj.Y - GCP[1].Obj.Y, 2)) / sqrt(pow(GCP[0].ImgLeft.x - GCP[1].ImgLeft.x, 2) + pow(GCP[0].ImgLeft.y - GCP[1].ImgLeft.y, 2));
	mRight = sqrt(pow(GCP[0].Obj.X - GCP[1].Obj.X, 2) + pow(GCP[0].Obj.Y - GCP[1].Obj.Y, 2)) / sqrt(pow(GCP[0].ImgRight.x - GCP[1].ImgRight.x, 2) + pow(GCP[0].ImgRight.y - GCP[1].ImgRight.y, 2));

	HLeft = mLeft * f0;
	HRight = mRight * f0;
	LXs = (GCP[0].Obj.X + GCP[2].Obj.X) / 2;
	LYs = (GCP[0].Obj.Y + GCP[2].Obj.Y) / 2;
	LZs = (GCP[0].Obj.Z + GCP[2].Obj.Z) / 2 + HLeft;

	RXs = (GCP[0].Obj.X + GCP[2].Obj.X) / 2;
	RYs = (GCP[0].Obj.Y + GCP[2].Obj.Y) / 2;
	RZs = (GCP[0].Obj.Z + GCP[2].Obj.Z) / 2+HRight;

	Lω = Lφ  = Rω = Rφ  = 0;
	Lκ = Rκ = 0;

	//平差模块

	do {
		FormLeftErrorEquations(L_B, L_L);
		L_BT = ~L_B;
		L_N = L_BT  * L_B;

		L_InvN = L_N.Inv();
		L_detX = L_InvN * (L_BT  * L_L);

		LXs += L_detX(0, 0);
		LYs += L_detX(1, 0);
		LZs += L_detX(2, 0);
		Lφ += L_detX(3, 0);
		Lω += L_detX(4, 0);
		Lκ += L_detX(5, 0);

	} while (
		abs(L_detX(0, 0)) >= 0.0001 ||
		abs(L_detX(1, 0)) >= 0.0001 ||
		abs(L_detX(2, 0)) >= 0.0001 ||
		abs(L_detX(3, 0)) >= 0.0000048 ||
		abs(L_detX(4, 0)) >= 0.0000048 ||
		abs(L_detX(5, 0)) >= 0.0000048
		);

	do {
		FormRightErrorEquations(R_B, R_L);
		R_BT = ~R_B;
		R_N = R_BT * R_B;

		/*CStdioFile sf;
setlocale(LC_ALL, "");
if (!sf.Open(strFileName, CFile::modeCreate | CFile::modeWrite)) return;
CString strLine;
strLine.Format(_T("点号\t         左片x(m)\t         左片y(m)\t         右片x(m)\t         右片y(m)\t         X(m)\t         Y(m)\t        Z(m)\r\n"));
sf.WriteString(strLine);
for (int i = 0; i < L_L.Row(); i++)
{
	for (int j = 0; j < L_L.Col(); j++)
	{
		strLine.Format(_T("%.10f\t"), L_L(i, j));
		sf.WriteString(strLine);
	}
	strLine.Format(_T("\r\n"));
	sf.WriteString(strLine);
}
sf.Close();
*/
		R_InvN = R_N.Inv();
		R_detX = R_InvN * (R_BT * R_L);

		RXs += R_detX(0, 0);
		RYs += R_detX(1, 0);
		RZs += R_detX(2, 0);
		Rφ += R_detX(3, 0);
		Rω += R_detX(4, 0);
		Rκ += R_detX(5, 0);

	} while (
		abs(R_detX(0, 0)) >= 0.0001 ||
		abs(R_detX(1, 0)) >= 0.0001 ||
		abs(R_detX(2, 0)) >= 0.0001 ||
		abs(R_detX(3, 0)) >= 0.0000048 ||
		abs(R_detX(4, 0)) >= 0.0000048 ||
		abs(R_detX(5, 0)) >= 0.0000048);


	//评定精度(单位为米和秒)

	L_sigma = sqrt((L_detX * ~L_detX)(0, 0) / (2.0 * iCountGCP - 6.0));
	CMatrix L_Qxx = L_InvN;
	L_mx = L_sigma * sqrt(L_Qxx(0, 0)) * 1000;
	L_my = L_sigma * sqrt(L_Qxx(1, 1)) * 1000;
	L_mz = L_sigma * sqrt(L_Qxx(2, 2)) * 1000;
	L_mφ = L_sigma * sqrt(L_Qxx(3, 3)) * 206265;
	L_mω = L_sigma * sqrt(L_Qxx(4, 4)) * 206265;
	L_mκ = L_sigma * sqrt(L_Qxx(5, 5)) * 206265;

	R_sigma = sqrt((R_detX * ~R_detX)(0, 0) / (2.0 * iCountGCP - 6.0));
	CMatrix R_Qxx = R_InvN;
	R_mx = R_sigma * sqrt(R_Qxx(0, 0)) * 1000;
	R_my = R_sigma * sqrt(R_Qxx(1, 1)) * 1000;
	R_mz = R_sigma * sqrt(R_Qxx(2, 2)) * 1000;
	R_mφ = R_sigma * sqrt(R_Qxx(3, 3)) * 206265;
	R_mω = R_sigma * sqrt(R_Qxx(4, 4)) * 206265;
	R_mκ = R_sigma * sqrt(R_Qxx(5, 5)) * 206265;

}

void CSpaIntSect::SpaIntSection(CString strFileName)//前方交会
{
	RigorousAdjust();
	//计算旋转矩阵系数
	double La1, Lb1, Lc1, La2, Lb2, Lc2, La3, Lb3, Lc3;
	double Ra1, Rb1, Rc1, Ra2, Rb2, Rc2, Ra3, Rb3, Rc3;
	//左片旋转矩阵系数
	La1 = cos(Lφ) * cos(Lκ) - sin(Lφ) * sin(Lκ) * sin(Lω);
	La2 = -cos(Lφ) * sin(Lκ) - sin(Lφ) * cos(Lκ) * sin(Lω);
	La3 = -sin(Lφ) * cos(Lω);
	Lb1 = cos(Lω) * sin(Lκ);
	Lb2 = cos(Lω) * cos(Lκ);
	Lb3 = -sin(Lω);
	Lc1 = sin(Lφ) * cos(Lκ) + cos(Lφ) * sin(Lκ) * sin(Lω);
	Lc2 = -sin(Lφ) * sin(Lκ) + cos(Lφ) * cos(Lκ) * sin(Lω);
	Lc3 = cos(Lφ) * cos(Lω);
	//右片旋转矩阵系数
	Ra1 = cos(Rφ) * cos(Rκ) - sin(Rφ) * sin(Rκ) * sin(Rω);
	Ra2 = -cos(Rφ) * sin(Rκ) - sin(Rφ) * cos(Rκ) * sin(Rω);
	Ra3 = -sin(Rφ) * cos(Rω);
	Rb1 = cos(Rω) * sin(Rκ);
	Rb2 = cos(Rω) * cos(Rκ);
	Rb3 = -sin(Rω);
	Rc1 = sin(Rφ) * cos(Rκ) + cos(Rφ) * sin(Rκ) * sin(Rω);
	Rc2 = -sin(Rφ) * sin(Rκ) + cos(Rφ) * cos(Rκ) * sin(Rω);
	Rc3 = cos(Rφ) * cos(Rω);

	//构造旋转矩阵
	CMatrix L_R, R_R;
	L_R(0, 0) = La1;
	L_R(0, 1) = La2;
	L_R(0, 2) = La3;
	L_R(1, 0) = Lb1;
	L_R(1, 1) = Lb2;
	L_R(1, 2) = Lb3;
	L_R(2, 0) = Lc1;
	L_R(2, 1) = Lc2;
	L_R(2, 2) = Lc3;

	R_R(0, 0) = Ra1;
	R_R(0, 1) = Ra2;
	R_R(0, 2) = Ra3;
	R_R(1, 0) = Rb1;
	R_R(1, 1) = Rb2;
	R_R(1, 2) = Rb3;
	R_R(2, 0) = Rc1;
	R_R(2, 1) = Rc2;
	R_R(2, 2) = Rc3;

	//计算基线分量
	double Bx = RXs - LXs;
	double By = RYs - LYs;
	double Bz = RZs - LZs;

	for (int i = 0; i < iCountP; i++)
	{
		//计算像点像辅坐标
		CMatrix LObj(3, 1), RObj(3, 1);
		CMatrix LImg(3, 1), RImg(3, 1);

		LImg(0, 0) = P[i].ImgLeft.x;
		LImg(1, 0) = P[i].ImgLeft.y;
		LImg(2, 0) = -f0;
		LObj = L_R * LImg;

		RImg(0, 0) = P[i].ImgRight.x;
		RImg(1, 0) = P[i].ImgRight.y;
		RImg(2, 0) = -f0;
		RObj = R_R * RImg;

		//计算投影系数
		double LN, RN;
		LN = (Bx * RObj(2, 0) - Bz * RObj(0, 0)) / (LObj(0, 0) * RObj(2, 0) - LObj(2, 0) * RObj(0, 0));
		RN = (Bx * LObj(2, 0) - Bz * LObj(0, 0)) / (LObj(0, 0) * RObj(2, 0) - LObj(2, 0) * RObj(0, 0));

		//计算地面点像辅坐标
		double detx = LN * LObj(0, 0);
		double dety = 1.0 / 2.0 * (LN * LObj(1, 0) + RN * RObj(1, 0) + By);
		double detz = LN * LObj(2, 0);

		//计算像点地面坐标
		P[i].Obj.X = LXs + detx;
		P[i].Obj.Y = LYs + dety;
		P[i].Obj.Z = LZs + detz;
	}

	//输出成果
	CStdioFile sf;
	setlocale(LC_ALL, "");
	if (!sf.Open(strFileName, CFile::modeCreate | CFile::modeWrite)) return;
	CString strLine;
	strLine.Format(_T("待定点计算成果\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("点号\t左片x(m)\t左片y(m)\t右片x(m)\t右片y(m)\tX(m)\t Y(m)\t Z(m)\r\n"));
	sf.WriteString(strLine);
	for (int i = 0; i < iCountP; i++)
	{
	strLine.Format(_T("%s\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f %.3f %.3f\r\n"),
		P[i].name,
		P[i].ImgLeft.x*1000,
		P[i].ImgLeft.y*1000,
		P[i].ImgRight.x*1000,
		P[i].ImgRight.y*1000,
		P[i].Obj.X,
		P[i].Obj.Y,
		P[i].Obj.Z
	);
	sf.WriteString(strLine);

	}
	strLine.Format(_T("外方位元素计算成果\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("像片\tXs(m)\tYs(m)\tZs(m)\tφ(s)\t ω(s)\t κ(s)\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("%s\t%.2f\t%.2f\t%.2f\t%.2f\t %.2f\t %.2f\r\n"),
		_T("左片"), LXs, LYs, LZs, Lφ * 206265, Lω * 206265, Lκ * 206265);
	sf.WriteString(strLine);
	strLine.Format(_T("%s\t%.2f\t%.2f\t%.2f\t%.2f\t %.2f\t %.2f\r\n"),
		_T("右片"), RXs, RYs, RZs, Rφ * 206265, Rω * 206265, Rκ * 206265);
	sf.WriteString(strLine);
	strLine.Format(_T("外方位元素精度评定成果\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("像片\t单位权中误差(μm)\tXs中误差(mm)\tYs中误差(mm)\tZs中误差(mm)\tφ中误差(s)\t ω中误差(s)\t κ中误差(s)\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("%s\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t %.2f\t %.2f\r\n"),
		_T("左片"),
		L_sigma*1e6,
		L_mx,
		L_my,
		L_mz,
		L_mφ,
		L_mω,
		L_mκ
	);
	sf.WriteString(strLine);
	strLine.Format(_T("%s\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t %.2f\t %.2f\r\n"),
		_T("右片"),
		R_sigma* 1e6,
		R_mx,
		R_my,
		R_mz,
		R_mφ,
		R_mω,
		R_mκ
	);
	sf.WriteString(strLine);
	sf.Close();

}

Point CSpaIntSect::GetPoint(int pos)//返回待定点
{
		return P[pos];
}

