// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
//#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <GL/glut.h>
#include <GL/glaux.h>
#include <iostream>
#include <vector>
#include <cstdio>
#include <string>
#include <Kinect.h>
#include <fstream>

//与PCL内部方法冲突
#undef min
#undef max
//PCL 配置过程在收藏夹里
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> 
#include <pcl/filters/voxel_grid.h>


// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
	double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
	int frameID;
	cv::Mat rgb, depth; //该帧对应的彩色图与深度图
	cv::Mat desp;       //特征描述子
	std::vector<cv::KeyPoint> kp; //关键点
	~FRAME() {
		kp.clear();
	}
};

// PnP 结果
struct RESULT_OF_PNP
{
	cv::Mat rvec, tvec;
	int inliers;
};

// 函数接口
// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera);

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp(FRAME& frame, std::string detector, std::string descriptor);

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera);

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec);

// joinPointCloud 
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera);

// 参数读取类
class ParameterReader
{
public:
	ParameterReader(std::string filename = "parameters.txt")
	{
		ifstream fin(filename.c_str());
		if (!fin)
		{
			cerr << "parameter file does not exist." << endl;
			return;
		}
		while (!fin.eof())
		{
			std::string str;
			getline(fin, str);
			if (str[0] == '#')
			{
				// 以‘＃’开头的是注释
				continue;
			}

			int pos = str.find("=");
			if (pos == -1)
				continue;
			std::string key = str.substr(0, pos);
			std::string value = str.substr(pos + 1, str.length());
			data[key] = value;

			if (!fin.good())
				break;
		}
	}
	std::string getData(std::string key)
	{
		std::map<std::string, std::string>::iterator iter = data.find(key);
		if (iter == data.end())
		{
			cerr << "Parameter name " << key << " not found!" << endl;
			return 	std::string("NOT_FOUND");
		}
		return iter->second;
	}
public:
	std::map<std::string, std::string> data;
};

inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
	ParameterReader pd;
	CAMERA_INTRINSIC_PARAMETERS camera;
	camera.fx = atof(pd.getData("camera.fx").c_str());
	camera.fy = atof(pd.getData("camera.fy").c_str());
	camera.cx = atof(pd.getData("camera.cx").c_str());
	camera.cy = atof(pd.getData("camera.cy").c_str());
	camera.scale = atof(pd.getData("camera.scale").c_str());
	return camera;
}




