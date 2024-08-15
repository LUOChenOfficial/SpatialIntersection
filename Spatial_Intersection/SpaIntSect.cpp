#include "pch.h"
#include "SpaIntSect.h"
#include <locale.h>

CSpaIntSect::CSpaIntSect(void)
{
}

CSpaIntSect::~CSpaIntSect(void)
{
	//�ͷŶ�̬�����ڴ�
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

CString* CSpaIntSect::SplitString(CString str, char split, int& iSubStrs)//�ָ��ַ���
{
	int iPos = 0; //�ָ��λ��
	int iNums = 0; //�ָ��������
	CString strTemp = str;
	CString strRight;
	//�ȼ������ַ���������
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
	if (iNums == 0) //û���ҵ��ָ��
	{
		//���ַ����������ַ�������
		iSubStrs = 1;
		return NULL;
	}
	//���ַ�������
	iSubStrs = iNums + 1; //�Ӵ������� = �ָ������ + 1
	CString* pStrSplit;
	pStrSplit = new CString[iSubStrs];
	strTemp = str;
	CString strLeft;
	for (int i = 0; i < iNums; i++)
	{
		iPos = strTemp.Find(split);
		//���Ӵ�
		strLeft = strTemp.Left(iPos);
		//���Ӵ�
		strRight = strTemp.Mid(iPos + 1, strTemp.GetLength());
		strTemp = strRight;
		pStrSplit[i] = strLeft;
	}
	pStrSplit[iNums] = strTemp;
	return pStrSplit;
}

bool CSpaIntSect::LoadObsData(const CString strFileName, CString& strObsData)//��ȡ����
{
	//��ȡ����
	CStdioFile sf;
	if (!sf.Open(strFileName, CFile::modeRead)) return 0;
	CString strLine;
	strLine.Empty();
	//���Ƶ㼰���������
	BOOL bEOF = sf.ReadString(strLine);
	int tmp,m,n,o;
	CString* pstrData = SplitString(strLine, ',', tmp);
	for (int j = 0; j < tmp; j++)
		pstrData[j].Trim();
	iCountGCP = _ttoi(pstrData[0]);
	iCountP = _ttoi(pstrData[1]);
	SetPoiNum(iCountGCP, iCountP);
	//��ȡ���Ƶ�
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
	//��ȡ������
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
	//�ڷ�λԪ��
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

void CSpaIntSect::FormLeftErrorEquations(CMatrix& BLeft , CMatrix& LLeft)//�����Ƭ����,B Ϊϵ������LΪ����������
{
	double La1, Lb1, Lc1, La2, Lb2, Lc2, La3, Lb3, Lc3;
	//��ת����ϵ��
	La1 = cos(L��) * cos(L��) - sin(L��) * sin(L��) * sin(L��);
	La2 = -cos(L��) * sin(L��) - sin(L��) * cos(L��) * sin(L��);
	La3 = -sin(L��) * cos(L��);
	Lb1 = cos(L��) * sin(L��);
	Lb2 = cos(L��) * cos(L��);
	Lb3 = -sin(L��);
	Lc1= sin(L��) * cos(L��) + cos(L��) * sin(L��) * sin(L��);
	Lc2 = -sin(L��) * sin(L��) + cos(L��) * cos(L��) * sin(L��);
	Lc3= cos(L��) * cos(L��);

	mLeft = sqrt(pow(GCP[0].Obj.X - GCP[1].Obj.X, 2) + pow(GCP[0].Obj.Y - GCP[1].Obj.Y, 2)) / sqrt(pow(GCP[0].ImgLeft.x - GCP[1].ImgLeft.x, 2) + pow(GCP[0].ImgLeft.y - GCP[1].ImgLeft.y, 2));
	HLeft = mLeft * f0;

	for (int i = 0; i < iCountGCP; i++)
	{
		BLeft(2 * i, 0) = -f0 / HLeft * cos(L��);
		BLeft(2 * i, 1) = -f0 / HLeft * sin(L��);
		BLeft(2 * i, 2) = -(GCP[i].ImgLeft.x - x0) / HLeft;
		BLeft(2 * i, 3) = -(f0 + pow(GCP[i].ImgLeft.x - x0, 2) / f0) * cos(L��) + (GCP[i].ImgLeft.x - x0)*(GCP[i].ImgLeft.y - y0) * sin(L��)/f0;
		BLeft(2 * i, 4) = -(GCP[i].ImgLeft.x - x0) * (GCP[i].ImgLeft.y - y0) * cos(L��) / f0 - (f0 + pow(GCP[i].ImgLeft.x - x0, 2) / f0) * sin(L��);
		BLeft(2 * i, 5) = GCP[i].ImgLeft.y - y0;
		BLeft(2 * i + 1, 0) = f0 / HLeft * sin(L��);
		BLeft(2 * i + 1, 1) = -f0 / HLeft * cos(L��);
		BLeft(2 * i + 1, 2) = -(GCP[i].ImgLeft.y - y0) / HLeft;
		BLeft(2 * i + 1, 3) = -(GCP[i].ImgLeft.x - x0) * (GCP[i].ImgLeft.y - y0) * cos(L��) / f0 + (f0 + pow(GCP[i].ImgLeft.y - y0, 2) / f0) * sin(L��);
		BLeft(2 * i + 1, 4) = -(f0 + pow(GCP[i].ImgLeft.y - y0, 2) / f0) * cos(L��) - (GCP[i].ImgLeft.x - x0)*(GCP[i].ImgLeft.y - y0) * sin(L��)/f0;
		BLeft(2 * i + 1, 5) = -(GCP[i].ImgLeft.x - x0);

		LLeft(2 * i, 0) = -(x0 - f0 * (La1 * (GCP[i].Obj.X - LXs) + Lb1 * (GCP[i].Obj.Y - LYs) + Lc1 * (GCP[i].Obj.Z - LZs)) / (La3 * (GCP[i].Obj.X - LXs) + Lb3 * (GCP[i].Obj.Y - LYs) + Lc3 * (GCP[i].Obj.Z - LZs)) - GCP[i].ImgLeft.x);
		LLeft(2 * i + 1, 0) = -(y0 - f0 * (La2 * (GCP[i].Obj.X - LXs) + Lb2 * (GCP[i].Obj.Y - LYs) + Lc2 * (GCP[i].Obj.Z - LZs)) / (La3 * (GCP[i].Obj.X - LXs) + Lb3 * (GCP[i].Obj.Y - LYs) + Lc3 * (GCP[i].Obj.Z - LZs)) - GCP[i].ImgLeft.y);
	}
}

void CSpaIntSect::FormRightErrorEquations(CMatrix& BRight, CMatrix& LRight)//�����Ƭ����,B Ϊϵ������LΪ����������
{
	double Ra1, Rb1, Rc1, Ra2, Rb2, Rc2, Ra3, Rb3, Rc3;
	//��ת����ϵ��
	Ra1 = cos(R��) * cos(R��) - sin(R��) * sin(R��) * sin(R��);
	Ra2 = -cos(R��) * sin(R��) - sin(R��) * cos(R��) * sin(R��);
	Ra3 = -sin(R��) * cos(R��);
	Rb1 = cos(R��) * sin(R��);
	Rb2 = cos(R��) * cos(R��);
	Rb3 = -sin(R��);
	Rc1 = sin(R��) * cos(R��) + cos(R��) * sin(R��) * sin(R��);
	Rc2 = -sin(R��) * sin(R��) + cos(R��) * cos(R��) * sin(R��);
	Rc3 = cos(R��) * cos(R��);

	mRight = sqrt(pow(GCP[0].Obj.X - GCP[1].Obj.X, 2) + pow(GCP[0].Obj.Y - GCP[1].Obj.Y, 2)) / sqrt(pow(GCP[0].ImgRight.x - GCP[1].ImgRight.x, 2) + pow(GCP[0].ImgRight.y - GCP[1].ImgRight.y, 2));
	HRight = mRight * f0;

	for (int i = 0; i < iCountGCP; i++)
	{
		BRight(2 * i, 0) = -f0 / HRight * cos(R��);
		BRight(2 * i, 1) = -f0 / HRight * sin(R��);
		BRight(2 * i, 2) = -(GCP[i].ImgRight.x - x0) / HRight;
		BRight(2 * i, 3) = -(f0 + pow(GCP[i].ImgRight.x - x0, 2) / f0) * cos(R��) + (GCP[i].ImgRight.x - x0) * (GCP[i].ImgRight.y - y0) * sin(R��) / f0;
		BRight(2 * i, 4) = -(GCP[i].ImgRight.x - x0) * (GCP[i].ImgRight.y - y0) * cos(R��) / f0 - (f0 + pow(GCP[i].ImgRight.x - x0, 2) / f0) * sin(R��);
		BRight(2 * i, 5) = GCP[i].ImgRight.y - y0;
		BRight(2 * i + 1, 0) = f0 / HRight * sin(R��);
		BRight(2 * i + 1, 1) = -f0 / HRight * cos(R��);
		BRight(2 * i + 1, 2) = -(GCP[i].ImgRight.y - y0) / HRight;
		BRight(2 * i + 1, 3) = -(GCP[i].ImgRight.x - x0) * (GCP[i].ImgRight.y - y0) * cos(R��) / f0 + (f0 + pow(GCP[i].ImgRight.y - y0, 2) / f0) * sin(R��);
		BRight(2 * i + 1, 4) = -(f0 + pow(GCP[i].ImgRight.y - y0, 2) / f0) * cos(R��) - (GCP[i].ImgRight.x - x0) * (GCP[i].ImgRight.y - y0) * sin(R��) / f0;
		BRight(2 * i + 1, 5) = -(GCP[i].ImgRight.x - x0);

		LRight(2 * i, 0) = -(x0 - f0 * (Ra1 * (GCP[i].Obj.X - RXs) + Rb1 * (GCP[i].Obj.Y - RYs) + Rc1 * (GCP[i].Obj.Z - RZs)) / (Ra3 * (GCP[i].Obj.X - RXs) + Rb3 * (GCP[i].Obj.Y - RYs) + Rc3 * (GCP[i].Obj.Z - RZs)) - GCP[i].ImgRight.x);
		LRight(2 * i + 1, 0) = -(y0 - f0 * (Ra2 * (GCP[i].Obj.X - RXs) + Rb2 * (GCP[i].Obj.Y - RYs) + Rc2 * (GCP[i].Obj.Z - RZs)) / (Ra3 * (GCP[i].Obj.X - RXs) + Rb3 * (GCP[i].Obj.Y - RYs) + Rc3 * (GCP[i].Obj.Z - RZs)) - GCP[i].ImgRight.y);
	}
}

void CSpaIntSect::RigorousAdjust()//ƽ���
{
	//-----------------------------------------------ƽ����㲽��------------------------------------------------------
	CMatrix L_B(2 * iCountGCP, 6), L_L(2 * iCountGCP, 1);
	CMatrix R_B(2 * iCountGCP, 6), R_L(2 * iCountGCP, 1);
	CMatrix L_BT, L_N, L_InvN, L_detX, L_V, L_Qx;
	CMatrix R_BT, R_N, R_InvN, R_detX, R_V, R_Qx;

	//��ʼ����ȡ����ֵ��

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

	L�� = L��  = R�� = R��  = 0;
	L�� = R�� = 0;

	//ƽ��ģ��

	do {
		FormLeftErrorEquations(L_B, L_L);
		L_BT = ~L_B;
		L_N = L_BT  * L_B;

		L_InvN = L_N.Inv();
		L_detX = L_InvN * (L_BT  * L_L);

		LXs += L_detX(0, 0);
		LYs += L_detX(1, 0);
		LZs += L_detX(2, 0);
		L�� += L_detX(3, 0);
		L�� += L_detX(4, 0);
		L�� += L_detX(5, 0);

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
strLine.Format(_T("���\t         ��Ƭx(m)\t         ��Ƭy(m)\t         ��Ƭx(m)\t         ��Ƭy(m)\t         X(m)\t         Y(m)\t        Z(m)\r\n"));
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
		R�� += R_detX(3, 0);
		R�� += R_detX(4, 0);
		R�� += R_detX(5, 0);

	} while (
		abs(R_detX(0, 0)) >= 0.0001 ||
		abs(R_detX(1, 0)) >= 0.0001 ||
		abs(R_detX(2, 0)) >= 0.0001 ||
		abs(R_detX(3, 0)) >= 0.0000048 ||
		abs(R_detX(4, 0)) >= 0.0000048 ||
		abs(R_detX(5, 0)) >= 0.0000048);


	//��������(��λΪ�׺���)

	L_sigma = sqrt((L_detX * ~L_detX)(0, 0) / (2.0 * iCountGCP - 6.0));
	CMatrix L_Qxx = L_InvN;
	L_mx = L_sigma * sqrt(L_Qxx(0, 0)) * 1000;
	L_my = L_sigma * sqrt(L_Qxx(1, 1)) * 1000;
	L_mz = L_sigma * sqrt(L_Qxx(2, 2)) * 1000;
	L_m�� = L_sigma * sqrt(L_Qxx(3, 3)) * 206265;
	L_m�� = L_sigma * sqrt(L_Qxx(4, 4)) * 206265;
	L_m�� = L_sigma * sqrt(L_Qxx(5, 5)) * 206265;

	R_sigma = sqrt((R_detX * ~R_detX)(0, 0) / (2.0 * iCountGCP - 6.0));
	CMatrix R_Qxx = R_InvN;
	R_mx = R_sigma * sqrt(R_Qxx(0, 0)) * 1000;
	R_my = R_sigma * sqrt(R_Qxx(1, 1)) * 1000;
	R_mz = R_sigma * sqrt(R_Qxx(2, 2)) * 1000;
	R_m�� = R_sigma * sqrt(R_Qxx(3, 3)) * 206265;
	R_m�� = R_sigma * sqrt(R_Qxx(4, 4)) * 206265;
	R_m�� = R_sigma * sqrt(R_Qxx(5, 5)) * 206265;

}

void CSpaIntSect::SpaIntSection(CString strFileName)//ǰ������
{
	RigorousAdjust();
	//������ת����ϵ��
	double La1, Lb1, Lc1, La2, Lb2, Lc2, La3, Lb3, Lc3;
	double Ra1, Rb1, Rc1, Ra2, Rb2, Rc2, Ra3, Rb3, Rc3;
	//��Ƭ��ת����ϵ��
	La1 = cos(L��) * cos(L��) - sin(L��) * sin(L��) * sin(L��);
	La2 = -cos(L��) * sin(L��) - sin(L��) * cos(L��) * sin(L��);
	La3 = -sin(L��) * cos(L��);
	Lb1 = cos(L��) * sin(L��);
	Lb2 = cos(L��) * cos(L��);
	Lb3 = -sin(L��);
	Lc1 = sin(L��) * cos(L��) + cos(L��) * sin(L��) * sin(L��);
	Lc2 = -sin(L��) * sin(L��) + cos(L��) * cos(L��) * sin(L��);
	Lc3 = cos(L��) * cos(L��);
	//��Ƭ��ת����ϵ��
	Ra1 = cos(R��) * cos(R��) - sin(R��) * sin(R��) * sin(R��);
	Ra2 = -cos(R��) * sin(R��) - sin(R��) * cos(R��) * sin(R��);
	Ra3 = -sin(R��) * cos(R��);
	Rb1 = cos(R��) * sin(R��);
	Rb2 = cos(R��) * cos(R��);
	Rb3 = -sin(R��);
	Rc1 = sin(R��) * cos(R��) + cos(R��) * sin(R��) * sin(R��);
	Rc2 = -sin(R��) * sin(R��) + cos(R��) * cos(R��) * sin(R��);
	Rc3 = cos(R��) * cos(R��);

	//������ת����
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

	//������߷���
	double Bx = RXs - LXs;
	double By = RYs - LYs;
	double Bz = RZs - LZs;

	for (int i = 0; i < iCountP; i++)
	{
		//�������������
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

		//����ͶӰϵ��
		double LN, RN;
		LN = (Bx * RObj(2, 0) - Bz * RObj(0, 0)) / (LObj(0, 0) * RObj(2, 0) - LObj(2, 0) * RObj(0, 0));
		RN = (Bx * LObj(2, 0) - Bz * LObj(0, 0)) / (LObj(0, 0) * RObj(2, 0) - LObj(2, 0) * RObj(0, 0));

		//��������������
		double detx = LN * LObj(0, 0);
		double dety = 1.0 / 2.0 * (LN * LObj(1, 0) + RN * RObj(1, 0) + By);
		double detz = LN * LObj(2, 0);

		//��������������
		P[i].Obj.X = LXs + detx;
		P[i].Obj.Y = LYs + dety;
		P[i].Obj.Z = LZs + detz;
	}

	//����ɹ�
	CStdioFile sf;
	setlocale(LC_ALL, "");
	if (!sf.Open(strFileName, CFile::modeCreate | CFile::modeWrite)) return;
	CString strLine;
	strLine.Format(_T("���������ɹ�\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("���\t��Ƭx(m)\t��Ƭy(m)\t��Ƭx(m)\t��Ƭy(m)\tX(m)\t Y(m)\t Z(m)\r\n"));
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
	strLine.Format(_T("�ⷽλԪ�ؼ���ɹ�\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("��Ƭ\tXs(m)\tYs(m)\tZs(m)\t��(s)\t ��(s)\t ��(s)\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("%s\t%.2f\t%.2f\t%.2f\t%.2f\t %.2f\t %.2f\r\n"),
		_T("��Ƭ"), LXs, LYs, LZs, L�� * 206265, L�� * 206265, L�� * 206265);
	sf.WriteString(strLine);
	strLine.Format(_T("%s\t%.2f\t%.2f\t%.2f\t%.2f\t %.2f\t %.2f\r\n"),
		_T("��Ƭ"), RXs, RYs, RZs, R�� * 206265, R�� * 206265, R�� * 206265);
	sf.WriteString(strLine);
	strLine.Format(_T("�ⷽλԪ�ؾ��������ɹ�\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("��Ƭ\t��λȨ�����(��m)\tXs�����(mm)\tYs�����(mm)\tZs�����(mm)\t�������(s)\t �������(s)\t �������(s)\r\n"));
	sf.WriteString(strLine);
	strLine.Format(_T("%s\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t %.2f\t %.2f\r\n"),
		_T("��Ƭ"),
		L_sigma*1e6,
		L_mx,
		L_my,
		L_mz,
		L_m��,
		L_m��,
		L_m��
	);
	sf.WriteString(strLine);
	strLine.Format(_T("%s\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t %.2f\t %.2f\r\n"),
		_T("��Ƭ"),
		R_sigma* 1e6,
		R_mx,
		R_my,
		R_mz,
		R_m��,
		R_m��,
		R_m��
	);
	sf.WriteString(strLine);
	sf.Close();

}

Point CSpaIntSect::GetPoint(int pos)//���ش�����
{
		return P[pos];
}

