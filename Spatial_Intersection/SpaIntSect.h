#pragma once
#include "Matrix.h"

struct ImgCoor//像方坐标
{
	double x, y;
};
struct ObjCoor//物方坐标
{
	double X, Y, Z;
};

struct Point//点
{
	CString name;//点名
	ImgCoor ImgLeft, ImgRight;//像方坐标
	ObjCoor Obj;//物方坐标
};

class CSpaIntSect
{
public:
	CSpaIntSect(void);
	~CSpaIntSect(void);
private:
	//外方位元素（左右片）
	double LXs, LYs, LZs, Lω, Lφ, Lκ;
	double RXs, RYs, RZs, Rω, Rφ, Rκ;
	//内方位元素(单位m)
	double f0, x0, y0;
	//控制点
     Point* GCP;
	//待定点
	 Point* P;
	//控制点及待定点个数
	int iCountGCP, iCountP;
	//摄像高及比例尺
	double HLeft, HRight;
	double mLeft, mRight;
	//精度参数
	double L_sigma, R_sigma;// 单位权中误差
	double L_mx, R_mx;//Xs中误差
	double L_my, R_my;//Ys中误差
	double L_mz, R_mz;//Zs中误差
	double L_mφ, R_mφ; //φ中误差
	double L_mω, R_mω;//ω中误差
	double L_mκ, R_mκ;//κ中误差

public:
	bool LoadObsData(const CString strFileName, CString& strObsData);//读取数据
	Point GetPoint(int pos);//返回平差结果
private:
	void SetPoiNum(int m,int n);//设置控制点个数和待定点个数
	CString* SplitString(CString str, char split, int& iSubStrs);//分割字符串
	//---------------------------------------------------后方交会------------------------------------------------------------------
private:
	void FormLeftErrorEquations(CMatrix& BLeft, CMatrix& LLeft);//组成误差方程,B 为系数矩阵，L为常数项向量
	void FormRightErrorEquations( CMatrix& BRight, CMatrix& LRight);//组成误差方程,B 为系数矩阵，L为常数项向量
	void RigorousAdjust();//后方交会

	//---------------------------------------------------前方交会------------------------------------------------------------------
public:
	void SpaIntSection(CString strFileName);//前方交会
};

