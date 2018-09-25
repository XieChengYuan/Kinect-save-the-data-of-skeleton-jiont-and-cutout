/*
written by jywyq
�˴�����ȡ������ǰ�� ��������ڴ�й¶�����հ汾
����Ϊ opencv2.4.10 32bit + kinect2 32bit // + pcl 1.8.1 32bit vs2015
*/

#pragma region include
#include <WinSock2.h>
#include <Ws2tcpip.h>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//opengl
#include <GL/glut.h>
#include <GL/glaux.h>

//normal
#include <iostream>
#include <vector>
#include <cstdio>
#include <string>
#include <Kinect.h>
#include <fstream>

#include "Mytools.h"
#include "slamBase.h"

//��PCL�ڲ�������ͻ
#undef min
#undef max

//PCL ���ù������ղؼ���
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

////g2o
//#include <g2o/types/slam3d/types_slam3d.h>
//#include <g2o/core/sparse_optimizer.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/factory.h>
//#include <g2o/core/optimization_algorithm_factory.h>
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
//#include <g2o/solvers/eigen/linear_solver_eigen.h>
//#include <g2o/core/robust_kernel.h>
//#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//
//// ��g2o�Ķ���ŵ�ǰ��
//typedef g2o::BlockSolver_6_3 SlamBlockSolver;
//typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;
#pragma comment(lib, "ws2_32.lib")  
#pragma comment ( lib, "kinect20.lib" ) 
#pragma endregion include

//#define nofiltercontrast 1

const char DEFAULT_PORT[] = "4999";
const int RECV_BUF_SIZE = 1024;

// �����������
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef struct point {
	double x = 0;
	double y = 0;
	double z = 0;
	int TrackingState = 0;
}Csvpoint;



#pragma region global
char Recv[RECV_BUF_SIZE];
char recvSave[RECV_BUF_SIZE];
char recvEnd[RECV_BUF_SIZE];
char filename_skeleton[50];
std::vector<std::vector<Csvpoint>> csvpoint;
std::vector<Csvpoint> pp;
std::vector<int> compression_params;
int imgNum = 0;
Mytools mytools;
std::string filedir = "F:/img/";
int depth_width, depth_height;
char key;
int nWidth, nHeight;
std::map<int, int> filterCollection;
bool no_person;
bool begin_save = 0;
int save_or_end;
#pragma endregion

enum CHECK_RESULT { NOT_MATCHED = 0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME };// �������֡���������

// ��ȫ�ͷ�ָ��															 // ��ȫ�ͷ�ָ��
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

void saveImage(std::string filepath, int imgnum, const cv::Mat &Img) {
	std::string filename = filedir + filepath;
	filename += mytools.int2str(imgnum);
	filename += ".png";
	cv::imwrite(filename, Img, compression_params);
}

void saveAllImage(int smooth, const cv::Mat &resultImg, const cv::Mat &depthImg, const cv::Mat &colorImg_converted, const cv::Mat &depthImg_show) {
	if (smooth) {
		saveImage("smoothresult/result", imgNum, resultImg);
		saveImage("smoothcolor/color", imgNum, colorImg_converted);
		saveImage("smoothdepthimgshow/depthshow", imgNum, depthImg_show);
	}
	else {
		saveImage("result/result", imgNum, resultImg);
		saveImage("color/color", imgNum, colorImg_converted);
		saveImage("depthimgshow/depthshow", imgNum, depthImg_show);
	}
	saveImage("depth/depth", imgNum, depthImg);
}

void saveAllImage(const cv::Mat &resultImg, const cv::Mat &smoothdepthImg) {
	saveImage("smoothresult/result", imgNum, resultImg);
	saveImage("smoothdepthImg/smoothdepthImg", imgNum, smoothdepthImg);
}

// ����ڲ�
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //���ñ�����ɫ  
}

void read_PCL(std::string filename) {//test PCL
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	if (-1 == pcl::io::loadPCDFile(filename, *cloud)) {
		cout << "error input!" << endl;
		return;
	}

	cout << cloud->points.size() << endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");     //����viewer����

	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");
}

void convert_to_pointcloud_and_resultimg(UINT16 *cdepthData, const cv::Mat &ccolorImg,
	const ColorSpacePoint* colorSpacePoint, const CameraSpacePoint* cameraCoordinates,
	cv::Mat &cresultImg, pcl::PointCloud<PointT>::Ptr ccloud)
{
	UINT16 *tmp_pointer = NULL;
	tmp_pointer = cdepthData;
	for (int i = 0; i < depth_height; i++)
	{
		for (int j = 0; j < depth_width; j++)
		{
			unsigned int index = i * depth_width + j;
			ColorSpacePoint csp = colorSpacePoint[index];
			int colorX = static_cast<int>(floor(csp.X + 0.5));
			int colorY = static_cast<int>(floor(csp.Y + 0.5));
			//ѡȡ���ڲ�ɫͼ���ϵĵ㲢��ǰ������������ֵ�ָ�,�ٴθ�����ֵ��С
			if (colorX >= 0 && colorX < nWidth && colorY >= 0 && colorY < nHeight)
			{
				//������ɫ��Ϣ
				if (*tmp_pointer < 3000) {
					cresultImg.at<cv::Vec4b>(i, j) =
						ccolorImg.at<cv::Vec4b>(colorY, colorX);
					if (!no_person && begin_save) {
						PointT pt;
						pt.x = static_cast<float>(cameraCoordinates[index].X);
						pt.y = static_cast<float>(cameraCoordinates[index].Y);
						pt.z = static_cast<float>(cameraCoordinates[index].Z);

						pt.b = cresultImg.data[index * 4 + 0];
						pt.g = cresultImg.data[index * 4 + 1];
						pt.r = cresultImg.data[index * 4 + 2];

						ccloud->points.push_back(pt);
					}
				}
			}
			tmp_pointer++;
		}
	}

}


void savePointcloud(int smooth, const pcl::PointCloud<PointT>::Ptr pt) {
	std::string name;
	if (smooth)
		name = filedir + "smoothpointcloud/pointcloud" + mytools.int2str(imgNum) + ".pcd";
	else
		name = filedir + "pointcloud/pointcloud" + mytools.int2str(imgNum) + ".pcd";

	pt->height = 1;
	pt->width = pt->points.size();
	cout << "point cloud size = " << pt->points.size() << endl;
	pt->is_dense = false;
	pcl::io::savePCDFile(name, *pt);
	cout << "Point cloud saved." << endl;
	//������Ϳ�һ��
	//read_PCL(name);
}

void DrawBone(JointType b, JointType c, ICoordinateMapper*coordinatemapper, Joint joint[], cv::Mat&a)
{
	DepthSpacePoint d1, d2;
	coordinatemapper->MapCameraPointToDepthSpace(joint[b].Position, &d1);
	coordinatemapper->MapCameraPointToDepthSpace(joint[c].Position, &d2);

	if (d1.X > 0 && d1.X < 512 && d1.Y>0 && d1.Y < 424 && d2.X>0 && d2.X < 512 && d2.Y>0 && d2.Y < 424)
		cv::line(a, cv::Point(d1.X, d1.Y), cv::Point(d2.X, d2.Y), cv::Scalar(0, 255, 0, 255), 3);
	else
		line(a, cv::Point(d1.X, d1.Y), cv::Point(d2.X, d2.Y), cv::Scalar(255, 255, 255, 255), 1);
	circle(a, cv::Point(d1.X, d1.Y), 2, cv::Scalar(255, 255, 255, 255), 4);
	circle(a, cv::Point(d2.X, d2.Y), 2, cv::Scalar(255, 255, 255, 255), 4);
}

int process()
{
#pragma region ��ʼ��kinect
	HRESULT hResult = S_OK;
	IKinectSensor*          m_pKinectSensor;
	IDepthFrameSource*      pDepthFrameSource;
	IDepthFrameReader*      m_pDepthFrameReader;
	IColorFrameSource*      pColorFrameSource;
	IColorFrameReader*      m_pColorFrameReader;
	IFrameDescription*      depthFrameDescription;
	IFrameDescription*      colorFrameDescription;
	IFrameDescription*      bodyIndexFrameDescription;
	ColorImageFormat        imageFormat = ColorImageFormat_None;
	ICoordinateMapper*      coordinateMapper;

	IBodyIndexFrameSource*		pBodyIndexFrameSource;
	IBodyIndexFrameReader*		m_pBodyIndexFrameReader;
	
	IBodyFrameSource*        bodysource;
	IBodyFrameReader*        bodyreader;

	GetDefaultKinectSensor(&m_pKinectSensor);      //��ȡĬ��kinect������
	printf("��kinect�������ɹ�\n");
	//�򿪴�����
	m_pKinectSensor->Open();

	//��������Ϣ������
	m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
	//��ò�ɫ��Ϣ������  
	m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
	//����������Ϣ������
	m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
	//������Ϣ������
	m_pKinectSensor->get_BodyFrameSource(&bodysource);
	//�������Ϣ֡��ȡ��
	pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	//�򿪲�ɫ��Ϣ֡��ȡ��  
	pColorFrameSource->OpenReader(&m_pColorFrameReader);
	//����������
	pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
	//������Ϣ��ȡ��
	bodysource->OpenReader(&bodyreader);

#pragma endregion

#pragma region ���ÿͻ���
	WSADATA wsa_data; //WSADATA����,����windows socketִ�е���Ϣ
	int i_result = 0; //���շ���ֵ
	SOCKET sock_client = INVALID_SOCKET;
	addrinfo *result = nullptr, hints;
	//��ʼ��winsock��̬��(ws2_32.dll),MAKEWORD(2, 2)��������ʹ��winsock2.2�汾
	i_result = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (i_result != 0) {
		cerr << "WSAStartup() function failed: " << i_result << "\n";
		system("pause");
	}
	SecureZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	i_result = getaddrinfo("127.0.0.1", DEFAULT_PORT, &hints, &result);
	if (i_result != 0) {
		std::cerr << "getaddrinfo() function failed with error: " << WSAGetLastError() << "\n";
		WSACleanup();
		system("pause");
		return 1;
	}

	//�����׽���
	sock_client = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (sock_client == INVALID_SOCKET) {
		cerr << "socket() function failed with error: " << WSAGetLastError() << "\n";
		WSACleanup();
		system("pause");
		return 1;
	}
	//���ӷ�����
	i_result = connect(sock_client, result->ai_addr, result->ai_addrlen);
	if (i_result == SOCKET_ERROR) {
		cerr << "connect() function failed with error: " << WSAGetLastError() << "\n";
		WSACleanup();
		system("pause");
		return 1;
	}


	cout << "connect server successfully..." << endl;
	cout << "׼������ָ��..." << endl;
	freeaddrinfo(result);
	memset(&Recv, 0, RECV_BUF_SIZE);
	recv(sock_client, (char*)Recv, RECV_BUF_SIZE, 0);
	cout << "���յ�ָ�" << Recv << endl;
	cout << "��ʼ¼��" << endl;

#pragma endregion

	bool dddddd = 1;//��֡����

	int frame = 0;
//	int have_person_frame = 0;
//   int num = atoi(Recv);
//	while (have_person_frame < num)//ÿһ֡
//	{
	while (true)//ÿһ֡
	{
		//��ȡ��������
		key = cvWaitKey(10);
		if (key == 'o')begin_save ^= 1;
		//dddddd = 0;
		if (frame % 10 == 0)
			printf("Frame: %d\n", frame);
		frame++;

#pragma region ��ȡͼ��
		IColorFrame*       pColorFrame = NULL;
		IDepthFrame*       pDepthFrame = NULL;
		IBodyIndexFrame*   pBodyIndexFrame = NULL;
		IBodyFrame*        bodyframe = NULL;
		//��ȡ���ͼ��
		while (pDepthFrame == NULL) {
			//printf("AcquireLatestFrame\n");
			//������ʱ���ȡ���������ѭ����ȡ�����֡
			m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		}
		//��ȡ��ɫͼ��
		while (pColorFrame == NULL) {
			//������ʱ���ȡ���������ѭ����ȡ�����֡
			m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
		}
		//��ȡ��������
		while (pBodyIndexFrame == NULL) {
			//����˵��������
			m_pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
		}
		//��ȡ����
		while (bodyframe == NULL) {
			//����˵��������
			bodyreader->AcquireLatestFrame(&bodyframe);
		}
#pragma endregion

#pragma region ����
		cv::Mat asd(424, 512, CV_8UC4);
		if (SUCCEEDED(hResult))
		{
			IBody* body[BODY_COUNT] = { 0 };
			hResult = bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
			if (SUCCEEDED(hResult))
			{
				for (int i = 0; i < BODY_COUNT; i++)
				{
					BOOLEAN tracked = false;
					hResult = body[i]->get_IsTracked(&tracked);
					if (SUCCEEDED(hResult) && tracked)
					{

						Joint joint[JointType_Count];
						hResult = body[i]->GetJoints(JointType_Count, joint);
						DepthSpacePoint depthspacepoint;

						if (SUCCEEDED(hResult))
						{                            //���Լ�¼�Ǽ�������csv

													 // Torso
							DrawBone(JointType_Head, JointType_Neck, coordinateMapper, joint, asd);
							DrawBone(JointType_Neck, JointType_SpineShoulder, coordinateMapper, joint, asd);
							DrawBone(JointType_SpineShoulder, JointType_SpineMid, coordinateMapper, joint, asd);
							DrawBone(JointType_SpineMid, JointType_SpineBase, coordinateMapper, joint, asd);
							DrawBone(JointType_SpineShoulder, JointType_ShoulderRight, coordinateMapper, joint, asd);
							DrawBone(JointType_SpineShoulder, JointType_ShoulderLeft, coordinateMapper, joint, asd);
							DrawBone(JointType_SpineBase, JointType_HipRight, coordinateMapper, joint, asd);
							DrawBone(JointType_SpineBase, JointType_HipLeft, coordinateMapper, joint, asd);

							// Right Arm    
							DrawBone(JointType_ShoulderRight, JointType_ElbowRight, coordinateMapper, joint, asd);
							DrawBone(JointType_ElbowRight, JointType_WristRight, coordinateMapper, joint, asd);
							DrawBone(JointType_WristRight, JointType_HandRight, coordinateMapper, joint, asd);
							DrawBone(JointType_HandRight, JointType_HandTipRight, coordinateMapper, joint, asd);
							DrawBone(JointType_WristRight, JointType_ThumbRight, coordinateMapper, joint, asd);

							// Left Arm
							DrawBone(JointType_ShoulderLeft, JointType_ElbowLeft, coordinateMapper, joint, asd);
							DrawBone(JointType_ElbowLeft, JointType_WristLeft, coordinateMapper, joint, asd);
							DrawBone(JointType_WristLeft, JointType_HandLeft, coordinateMapper, joint, asd);
							DrawBone(JointType_HandLeft, JointType_HandTipLeft, coordinateMapper, joint, asd);
							DrawBone(JointType_WristLeft, JointType_ThumbLeft, coordinateMapper, joint, asd);

							// Right Leg
							DrawBone(JointType_HipRight, JointType_KneeRight, coordinateMapper, joint, asd);
							DrawBone(JointType_KneeRight, JointType_AnkleRight, coordinateMapper, joint, asd);
							DrawBone(JointType_AnkleRight, JointType_FootRight, coordinateMapper, joint, asd);

							// Left Leg
							DrawBone(JointType_HipLeft, JointType_KneeLeft, coordinateMapper, joint, asd);
							DrawBone(JointType_KneeLeft, JointType_AnkleLeft, coordinateMapper, joint, asd);
							DrawBone(JointType_AnkleLeft, JointType_FootLeft, coordinateMapper, joint, asd);

							DepthSpacePoint d1, d2;
							coordinateMapper->MapCameraPointToDepthSpace(joint[JointType_HandLeft].Position, &d1);
							coordinateMapper->MapCameraPointToDepthSpace(joint[JointType_HandRight].Position, &d2);
							HandState left;
							body[i]->get_HandLeftState(&left);
							HandState right;
							body[i]->get_HandRightState(&right);
							switch (left)
							{
							case HandState_Closed:
								circle(asd, cv::Point(d1.X, d1.Y), 10, cv::Scalar(0, 0, 255, 1), 20); break;
							case HandState_Open:
								circle(asd, cv::Point(d1.X, d1.Y), 10, cv::Scalar(0, 255, 0, 1), 20); break;
							case HandState_Lasso:
								circle(asd, cv::Point(d1.X, d1.Y), 10, cv::Scalar(255, 0, 0, 1), 20); break;
							default:
								break;
							}
							switch (right)
							{
							case HandState_Closed:
								circle(asd, cv::Point(d2.X, d2.Y), 10, cv::Scalar(0, 0, 255, 1), 20); break;
							case HandState_Open:
								circle(asd, cv::Point(d2.X, d2.Y), 10, cv::Scalar(0, 255, 0, 1), 20); break;
							case HandState_Lasso:
								circle(asd, cv::Point(d2.X, d2.Y), 10, cv::Scalar(255, 0, 0, 1), 20); break;
							default:
								break;
							}
							//һ֡����һά����
							std::vector<Csvpoint> pp;
							for (int i = 0; i < 25; ++i) {
								Csvpoint p;
								p.x = joint[i].Position.X;
								p.y = joint[i].Position.Y;
								p.z = joint[i].Position.Z;
								p.TrackingState = joint[i].TrackingState;
								//cout << joint[i].Position.X<< joint[i].Position.Y<<joint[i].Position.Z;
								pp.push_back(p);
							}
							csvpoint.push_back(pp);
							pp.clear();
						}

					}

				}

			}

			for (int count = 0; count < BODY_COUNT; count++)

			{

				SafeRelease(body[count]);

			}
		}
		//sprintf(filename_skeleton, "ske//skeletonImg%d.jpg", a++);
		//imwrite(filename_skeleton, asd);
#pragma endregion


#pragma region ��������������Ϣ
		pBodyIndexFrame->get_FrameDescription(&bodyIndexFrameDescription);
		//��ȡ֡��������Ϣ����͸ߣ�
		int bodyindex_width, bodyindex_height;
		bodyIndexFrameDescription->get_Width(&bodyindex_width);
		bodyIndexFrameDescription->get_Height(&bodyindex_height);
		UINT nBufferSize_bodyIndex = 0;
		BYTE *pBuffer_bodyIndex = NULL;
		pBodyIndexFrame->AccessUnderlyingBuffer(&nBufferSize_bodyIndex, &pBuffer_bodyIndex);
		//pBodyIndexFrame->CopyFrameDataToArray(bodyindex_height * bodyindex_width, pBuffer_bodyIndex);
		cv::Mat bodyIndeximg = mytools.ConvertMat_8(pBuffer_bodyIndex, bodyindex_width, bodyindex_height);
		mytools.Reverse(bodyIndeximg);

		int erosion_size = 3;
		cv::Mat erodeelement = getStructuringElement(cv::MORPH_ELLIPSE,
			cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			cv::Point(erosion_size, erosion_size));
		int dilation_size = 3;
		cv::Mat dilateelement = cv::getStructuringElement(cv::MORPH_ELLIPSE,
			cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			cv::Point(dilation_size, dilation_size));
		cv::erode(bodyIndeximg, bodyIndeximg, erodeelement);
		cv::dilate(bodyIndeximg, bodyIndeximg, dilateelement);
		//printf("Begin Counting\n");
		if (mytools.numOfNonZeroPixels(pBuffer_bodyIndex, depth_width, depth_height) < 100) 
		{
			//û�˾Ͳ�����
			no_person = 1;
			printf("No person!!\n");

		//	pBodyIndexFrame->Release();
		//	continue;

		}
		else 
		{
			no_person = 0;
		}
#pragma endregion

#pragma region �������ͼ��
		pDepthFrame->get_FrameDescription(&depthFrameDescription);
		//��ȡ֡��������Ϣ����͸ߣ�
		depthFrameDescription->get_Width(&depth_width);
		depthFrameDescription->get_Height(&depth_height);
		//printf("��� width=%d height=%d\n", depth_width, depth_height);
		UINT nBufferSize_depth = 0;
		UINT16 *pBuffer_depth = NULL;
		//��ȡͼ�����ظ�����ָ��ͼ���ָ��
		pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);
		//ת��Ϊ16λ��mat   
		cv::Mat depthImg = mytools.ConvertMat_16(pBuffer_depth, depth_width, depth_height);
#ifdef nofiltercontrast
		//���⻯���������Ϊ8-bit�Ҷ�ͼ
		Mat depthImg_show = mytools.ConvertMat_8(pBuffer_depth, depth_width, depth_height);
		equalizeHist(depthImg_show, depthImg_show);
		cvtColor(depthImg_show, depthImg_show, CV_GRAY2RGB);// not sure
#endif
#pragma endregion

#pragma region ����˫���˲�����������֮�󿴿�Ч��
		UINT16* DepthDataCopy = pBuffer_depth;
		UINT16 *smoothDepth = new UINT16[depth_height*depth_width];
		int innerBandThreshold = 3;
		int outerBandThreshold = 7;
		int magnification = 8;
		int changed = 0;
		uchar *bodyIndeximgdata = bodyIndeximg.data;
		int bodyIndexsize = 0;

		// ����ÿ������
		for (int y = 0; y < depth_height; y++) {
			for (int x = 0; x < depth_width; x++, bodyIndeximgdata++) {
				int index = y * depth_width + x;
				if (*bodyIndeximgdata == 0) {
					smoothDepth[index] = 0;
					continue;
				}


				//����ÿ�����Ϊ0��
				if (DepthDataCopy[index] == 0) {
					filterCollection.clear();
					// �������ڷ��������������������ں�������ȷ����ѡ�����Ƿ��˲�
					int innerBandCount = 0;
					int outerBandCount = 0;
					//������Χ5*5ȡ����������ֵ�Ļ�
					for (int dy = -2; dy < 3; dy++) {
						for (int dx = -2; dx < 3; dx++) {
							if (dy == 0 && dx && 0)continue;
							int ix = x + dx;
							int iy = y + dy;
							if (ix < 0 || ix >= depth_width || iy < 0 || iy >= depth_height)continue;
							int iindex = iy * depth_width + ix;
							if (DepthDataCopy[iindex] != 0) {
								int tmp_depth = DepthDataCopy[iindex];
								filterCollection[(tmp_depth + (int)pow(2, magnification - 1)) >> magnification]++;

								// ȷ���������ĸ��߽��ڵ����ز�Ϊ�㣬����Ӧ��������һ
								if (abs(dx) != 2 && abs(dy) != 2)
									innerBandCount++;
								else
									outerBandCount++;
							}
						}
					}
					// �жϼ������Ƿ񳬹���ֵ�����������ڷ������ص���Ŀ��������ֵ��
					// ��Ҫ�����з����������ֵ��Ӧ��ͳ������
					if (innerBandCount >= innerBandThreshold || outerBandCount >= outerBandThreshold) {
						std::map<int, int>::iterator it;
						int tmpfrequency = 0;
						int tmpdepth = 0;

						for (it = filterCollection.begin(); it != filterCollection.end(); it++) {
							//if(it == filterCollection.begin())puts("");
							//printf("%d %d\n", it->first, it->second);
							if (it->second > tmpfrequency) {
								tmpdepth = it->first;
								tmpfrequency = it->second;
								changed++;
							}
						}
						smoothDepth[index] = tmpdepth << magnification;
						bodyIndexsize++;
					}

				}
				else {
					smoothDepth[index] = pBuffer_depth[index];
					bodyIndexsize++;
				}
			}
		}
		//printf("changed: %d\n", changed);
		cv::Mat smoothdepthImg = mytools.ConvertMat_16(smoothDepth, depth_width, depth_height);
		cv::Mat smoothdepthImg_show = mytools.ConvertMat_8(smoothDepth, depth_width, depth_height);
		cv::equalizeHist(smoothdepthImg_show, smoothdepthImg_show);
		cv::cvtColor(smoothdepthImg_show, smoothdepthImg_show, CV_GRAY2RGB);// not sure
#pragma endregion

#pragma region �����ɫͼ��
																			//��ȡͼƬ������Ϣ
		pColorFrame->get_FrameDescription(&colorFrameDescription);
		uchar *pBuffer = NULL;
		UINT nBufferSize = 0;
		colorFrameDescription->get_Width(&nWidth);
		colorFrameDescription->get_Height(&nHeight);
		//printf("��ɫ width=%d height=%d\n", nWidth, nHeight);
		pColorFrame->get_RawColorImageFormat(&imageFormat);
		//������Ϊ ColorImageFormat_Yuy2    = 5��ΪYuy2��ʽ   
		//cout << "imageformat is " << imageFormat << endl;
		//�½�һ��mat�������ڱ�������ͼ��,ע������ĸ���ǰ�����ں�
		cv::Mat colorImg(nHeight, nWidth, CV_8UC4);
		pBuffer = colorImg.data;
		nBufferSize = colorImg.rows*colorImg.step;
		pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
#pragma endregion

#pragma region δ�˲�ʱ�ĶԱ�
#ifdef nofiltercontrast
#pragma region �����֡ӳ�䵽��ɫ�ռ������ռ�
		ColorSpacePoint* colorSpacePoint;
		CameraSpacePoint* cameraCoordinates;
		//�����֡ӳ�䵽��ɫ�ռ� 
		m_pKinectSensor->get_CoordinateMapper(&coordinateMapper);
		colorSpacePoint = new ColorSpacePoint[depth_width*depth_height];
		coordinateMapper->MapDepthFrameToColorSpace(depth_width*depth_height, pBuffer_depth, depth_width*depth_height, colorSpacePoint);
		//ӳ�䵽����ռ�
		cameraCoordinates = new CameraSpacePoint[depth_width*depth_height];
		coordinateMapper->MapDepthFrameToCameraSpace(depth_width*depth_height, pBuffer_depth, depth_width*depth_height, cameraCoordinates);
#pragma endregion

#pragma region ��׼��ɫ�����ͼ
		//New a Point Cloud
		pcl::PointCloud<PointT>::Ptr cloud(new PointCloud);
		//����һ����Mat���󱣴����õ���ǰ��ͼ��
		Mat colorImg_converted(depth_height, depth_width, CV_8UC4, Scalar::all(0));
		Mat resultImg(depth_height, depth_width, CV_8UC4, Scalar::all(0));
		//תΪ���ƺ���׼ͼ
		convert_to_pointcloud_and_resultimg(pBuffer_depth, colorImg, colorSpacePoint, cameraCoordinates,
			cloud, colorImg_converted, resultImg);
#pragma endregion
#endif
#pragma endregion

#pragma region �˲��������֡ӳ�䵽��ɫ�ռ������ռ�
		ColorSpacePoint* smoothcolorSpacePoint;
		CameraSpacePoint* smoothcameraCoordinates;
		//�����֡ӳ�䵽��ɫ�ռ� 
		m_pKinectSensor->get_CoordinateMapper(&coordinateMapper);
		smoothcolorSpacePoint = new ColorSpacePoint[depth_width*depth_height];
		coordinateMapper->MapDepthFrameToColorSpace(depth_width*depth_height, smoothDepth, depth_width*depth_height, smoothcolorSpacePoint);
		//ӳ�䵽����ռ�
		smoothcameraCoordinates = new CameraSpacePoint[depth_width*depth_height];
		coordinateMapper->MapDepthFrameToCameraSpace(depth_width*depth_height, smoothDepth, depth_width*depth_height, smoothcameraCoordinates);
#pragma endregion

#pragma region �˲�����׼��ɫ�����ͼ
		//New a Point Cloud
		pcl::PointCloud<PointT>::Ptr smoothcloud(new PointCloud);
		//����һ����Mat���󱣴����õ���ǰ��ͼ��
		cv::Mat smoothcolorImg_converted(depth_height, depth_width, CV_8UC4, cv::Scalar::all(0));
		cv::Mat smoothresultImg(depth_height, depth_width, CV_8UC4, cv::Scalar::all(0));
		convert_to_pointcloud_and_resultimg(smoothDepth, colorImg, smoothcolorSpacePoint, smoothcameraCoordinates,
			smoothresultImg, smoothcloud);
#pragma endregion

#pragma region save pointcloud when press "Enter"

		fd_set rfds;
		struct timeval timeout = { 0, 0 };
		FD_ZERO(&rfds);
		FD_SET(sock_client, &rfds);
		int ret = select(NULL, &rfds, NULL, NULL, &timeout);
		if (FD_ISSET(sock_client, &rfds))
		ret = recv(sock_client, recvSave, RECV_BUF_SIZE, 0);
		save_or_end = atoi(recvSave);
		if (save_or_end == 2) {
#ifdef nofiltercontrast
			saveAllImage(0, resultImg, depthImg, colorImg_converted, depthImg_show);
			savePointcloud(0, cloud);
#endif
			saveAllImage(smoothresultImg, smoothdepthImg);
			//savePointcloud(1, smoothcloud);
			imgNum++;
//			have_person_frame++;
		}

#pragma endregion

#pragma region ��ʾͼ��
#ifdef nofiltercontrast
		//cv::imshow("colorImg_converted", colorImg_converted);
		cv::imshow("depthImg", depthImg_show);
		cv::imshow("result", resultImg);
#endif
		cv::imshow("skeleton", asd);
		cv::imshow("smoothdepthImg_show", smoothdepthImg_show);
		cv::imshow("smoothresult", smoothresultImg);
		cv::imshow("bodyindex", bodyIndeximg);
#pragma endregion

#pragma region Release	
		smoothcloud->points.clear();
		
		asd.release();
		colorImg.release();
		depthImg.release();
		smoothdepthImg.release();
		smoothdepthImg_show.release();
		smoothcolorImg_converted.release();
		smoothresultImg.release();
		bodyIndeximg.release();
		bodyframe->Release();
		pDepthFrame->Release();
		pColorFrame->Release();
		pBodyIndexFrame->Release();
#ifdef nofiltercontrast
		cloud->points.clear();

		depthImg_show.release();
		resultImg.release();
		colorImg_converted.release();
		delete[] colorSpacePoint;
		delete[] cameraCoordinates;//������ڴ�й¶
#endif
		delete[] smoothcolorSpacePoint;
		delete[] smoothcameraCoordinates;
		delete[] smoothDepth;
#pragma endregion
		//waitKey(0);
		if (save_or_end == 3) break;
		if (key == 27)break;// break if `esc' key was pressed. 
	}
	std::cout << csvpoint.size() << std::endl;

	std::ofstream ofile;

	ofile.open("F:/img/clent2.csv", std::ios::out | std::ios::trunc);
	for (int i = 0; i < csvpoint.size(); i++)
	{//��ѭ��֡��

		for (int j = 0; j < csvpoint[i].size(); j++)
		{
			ofile << "\"" << csvpoint[i][j].x << "," << csvpoint[i][j].y << "," << csvpoint[i][j].z << "," << csvpoint[i][j].TrackingState << "\"" << ",";
		}
		ofile << std::endl;
	}
	ofile.close();
	csvpoint.clear();
}

int main(int argc, char** argv) {
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);//�洢ͼƬʱ��ѹ��

									/*
									//test opencv
									Mat img = imread("pic.jpg");
									namedWindow("��Ϸԭ��");
									imshow("��Ϸԭ��", img);
									waitKey(6000);
									*/
	system("title �ͻ���1");//����cmd���ڱ���
	system("color 0B");

	process();
	system("pause");
	//read_PCL(filedir + "pointcloud/pointcloud0.pcd");

}