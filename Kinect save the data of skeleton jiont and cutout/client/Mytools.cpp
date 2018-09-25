#include "Mytools.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <cstdio>
#include <string>
#include <Kinect.h>


Mytools::Mytools()
{
}


Mytools::~Mytools()
{
}


std::string Mytools::int2str(const int &int_temp)
{
	std::stringstream stream;
	stream << int_temp;
	std::string string_temp = stream.str();   //�˴�Ҳ������ stream>>string_temp  
	return string_temp;
}

// ת��depthͼ��cv::Mat
cv::Mat Mytools::ConvertMat_8(const UINT16* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_8UC1);
	//ָ��ͷָ��
	uchar* p_mat = img.data;
	//ָ�����һ��Ԫ�ص�ָ��
	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
	while (pBuffer < pBufferEnd)  //16λ���ֵΪ65536
	{
		//��16λ����ת����8λ
		*p_mat = *pBuffer / 65536.0 * 256;
		pBuffer++;
		p_mat++;
	}
	return img;
}

// ת��bodyindexͼ��cv::Mat
cv::Mat Mytools::ConvertMat_8(const BYTE* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_8UC1);
	//ָ��ͷָ��
	uchar* p_mat = img.data;
	//ָ�����һ��Ԫ�ص�ָ��
	const BYTE* pBufferEnd = pBuffer + (nWidth * nHeight);
	while (pBuffer < pBufferEnd)
	{
		*p_mat = *pBuffer;
		pBuffer++;
		p_mat++;
	}
	return img;
}

/// TIFF: �� 16λ������� �� UINT16�͵����� ת���� CV_16UC3��Mat����16λ�޷���������ͨ��Mat��
cv::Mat Mytools::ConvertMat_16(const UINT16* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_16UC1); // rows = nHeight cols = nWidth

	for (int i = 0; i < nHeight; i++)
	{
		UINT16* data = img.ptr<UINT16>(i);
		for (int j = 0; j < nWidth;)
		{
			USHORT depth = *pBuffer << 3;
			data[j++] = depth;
			pBuffer++;
		}
	}

	return img;
}

/// TIFF: �� 16λ������� �� UINT16�͵����� ת���� CV_16UC3��Mat����16λ�޷���������ͨ��Mat��
cv::Mat Mytools::ConvertMat_16(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	cv::Mat img(nHeight, nWidth, CV_16UC3); // rows = nHeight cols = nWidth

	for (int i = 0; i < nHeight; i++)
	{
		UINT16* data = img.ptr<UINT16>(i);
		for (int j = 0; j < nWidth * 3;)
		{
			USHORT depth = (*pBuffer >= nMinDepth) && (*pBuffer <= nMaxDepth) ? *pBuffer << 3 : 0;
			data[j++] = depth;
			data[j++] = depth;
			data[j++] = depth;
			pBuffer++;
		}
	}

	return img;
}

int Mytools::numOfNonZeroPixels(const BYTE *pBuffer, int nWidth, int nHeight) {
	//ָ�����һ��Ԫ�ص�ָ��
	const BYTE* pBufferEnd = pBuffer + (nWidth * nHeight);
	int num = 0;
	while (pBuffer < pBufferEnd)
	{
		if (*pBuffer)num++;
		pBuffer++;
	}
	return num;
}

void Mytools::Reverse(cv::Mat &mat) {
	uchar* p_mat = mat.data;
	for (int i = 0; i < mat.rows*mat.cols; i++) {
		*p_mat = 255 - *p_mat;
		p_mat++;
	}
}

