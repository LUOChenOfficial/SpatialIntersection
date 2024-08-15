#pragma once
#include "Matrix.h"

struct ImgCoor//������
{
	double x, y;
};
struct ObjCoor//�﷽����
{
	double X, Y, Z;
};

struct Point//��
{
	CString name;//����
	ImgCoor ImgLeft, ImgRight;//������
	ObjCoor Obj;//�﷽����
};

class CSpaIntSect
{
public:
	CSpaIntSect(void);
	~CSpaIntSect(void);
private:
	//�ⷽλԪ�أ�����Ƭ��
	double LXs, LYs, LZs, L��, L��, L��;
	double RXs, RYs, RZs, R��, R��, R��;
	//�ڷ�λԪ��(��λm)
	double f0, x0, y0;
	//���Ƶ�
     Point* GCP;
	//������
	 Point* P;
	//���Ƶ㼰���������
	int iCountGCP, iCountP;
	//����߼�������
	double HLeft, HRight;
	double mLeft, mRight;
	//���Ȳ���
	double L_sigma, R_sigma;// ��λȨ�����
	double L_mx, R_mx;//Xs�����
	double L_my, R_my;//Ys�����
	double L_mz, R_mz;//Zs�����
	double L_m��, R_m��; //�������
	double L_m��, R_m��;//�������
	double L_m��, R_m��;//�������

public:
	bool LoadObsData(const CString strFileName, CString& strObsData);//��ȡ����
	Point GetPoint(int pos);//����ƽ����
private:
	void SetPoiNum(int m,int n);//���ÿ��Ƶ�����ʹ��������
	CString* SplitString(CString str, char split, int& iSubStrs);//�ָ��ַ���
	//---------------------------------------------------�󷽽���------------------------------------------------------------------
private:
	void FormLeftErrorEquations(CMatrix& BLeft, CMatrix& LLeft);//�������,B Ϊϵ������LΪ����������
	void FormRightErrorEquations( CMatrix& BRight, CMatrix& LRight);//�������,B Ϊϵ������LΪ����������
	void RigorousAdjust();//�󷽽���

	//---------------------------------------------------ǰ������------------------------------------------------------------------
public:
	void SpaIntSection(CString strFileName);//ǰ������
};

