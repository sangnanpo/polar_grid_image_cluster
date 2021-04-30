// PCL
#include <pcl/point_cloud.h>	// 这个注释掉也可以用
#include <pcl/point_types.h>	// 这个注释掉也可以用
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <unordered_map> // 为何把它注释掉还是能用呢？
#include <vector>

#include <opencv2/opencv.hpp>

#include <cmath>

#include "loadfile.hpp"
#include "polar_grid_image_cluster.h"

#define PCL_NO_PRECOMPILE
using namespace std;

const float PI = 3.1415926;


// 点云距离滤波
void rangeFilter(	const pcl::PointCloud<pcl::PointXYZI>& cloudIn,
					pcl::PointCloud<pcl::PointXYZI>& cloudOut, float range_xy_max) {
	
	cloudOut.header = cloudIn.header;
	cloudOut.sensor_orientation_ = cloudIn.sensor_orientation_;
	cloudOut.sensor_origin_ = cloudIn.sensor_origin_;
	cloudOut.points.clear();
	for (int i = 0; i < cloudIn.size(); i++) {
		float range_xy = sqrt(	cloudIn.points[i].x * cloudIn.points[i].x + 
								cloudIn.points[i].y * cloudIn.points[i].y	);
		if (range_xy < range_xy_max) {
			cloudOut.points.push_back(cloudIn.points[i]);
		}
	}
}



// 计算得到range_xy和角度
void getRangeAndAngle(	const pcl::PointCloud<pcl::PointXYZI>& cloudIn,
						vector<float>& angle, vector<float>& range_xy	) {
	
	int points_num = cloudIn.points.size();
	for (int i = 0; i < points_num; ++i) {
		float x = cloudIn.points[i].x, y = cloudIn.points[i].y;
		float yawAngle = 0;
		if (x == 0 && y == 0) {
			yawAngle = 0;
		} else if (y >= 0) {
			yawAngle = (float) atan2(y, x);
		} else if (y <= 0) {
			yawAngle = (float) atan2(y, x) + 2 * PI;
		}

		yawAngle = yawAngle * 180 / PI; 
		float range_xy_ = sqrt(x * x + y * y);
		
		range_xy.push_back(range_xy_);
		angle.push_back(yawAngle);
	}
}

// 获得网格数量
const int getGridNum(const Param& paramIn) {
	return paramIn.height * paramIn.width;
}

// 获得每个point对应的网格的ID 
void getGridID(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, const Param& paramIn, vector<int>& gridIDOut) {
	
	const int columnNum = paramIn.width;
	const int rowNum = paramIn.height;
	const float range_xy_max = paramIn.range_xy_max;
	const float angle_gap = 360.0 / columnNum;
	const float range_gap = range_xy_max / rowNum;
	
	const int pointsNum = cloudIn.points.size();	// 只有读入文件后才能确定该值的大小，因此无法用该值来分配array内存

	vector<float> angle, range_xy;
	getRangeAndAngle(cloudIn, angle, range_xy);

	const int gridNum = rowNum * columnNum;

	int row_i, column_j, gridID;
	for (int pointID = 0; pointID < pointsNum; ++pointID) {
		// 距离为行ID，角度为列ID 
		row_i = floor(range_xy[pointID] / range_gap);
		column_j = floor(angle[pointID] / angle_gap);
		gridID = row_i * columnNum + column_j;	// 定义grid_ID，横向依次分布，0、1、2、3
		// 传ID比较方便
		gridIDOut.push_back(gridID);	// 原始点云中的第pointID个点属于网格中的第gridID个点

	}
}

// 将ID转化为IDMap：map的key为gridID；value为pointID
void ID2Map(vector<int> gridIn, unordered_multimap<int, int>& IDMapOut) {
	for (int i = 0; i < gridIn.size(); ++i) {
		IDMapOut.insert(make_pair(gridIn[i], i));
	}
}

// input:
//		cloudIn:输入点云
// 		paramIn:输入参数
// 		IDmapIn:输入IDmap
// output:
// 		unorderd_mapOut:输出map，first->gridID,second->Satistic
void pc2GridMap(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, const Param paramIn, unordered_multimap<int, int>& IDmapIn,  
			unordered_map<int, Statistics>& unordered_mapOut) {

	int gridNum = getGridNum(paramIn);

	for (int i = 0; i < gridNum; ++i) {
		auto keyRange = IDmapIn.equal_range(i);
		int len = IDmapIn.count(i);
		vector<int> pointID;
		for (auto it = keyRange.first; it != keyRange.second; ++it) {
			pointID.push_back(it->second);
		}
		Statistics statistic;
		statistic.gridPointsNum = len;
		statistic.gridToPointID = pointID;
		unordered_mapOut.insert(make_pair(i, statistic));
	}
}


int main(int argc, char** argv) {

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(
			new pcl::PointCloud<pcl::PointXYZI>);
	
	fileToCloud(argv[1], cloud); // 不给模板参数，让编译器自己推导即可
	
	Param param;
	param.height = atoi(argv[2]);
	param.width = atoi(argv[3]);
	param.range_xy_max = atof(argv[4]);
	
	rangeFilter(*cloud, *cloud_f, param.range_xy_max);	// 根据水平距离进行滤波
	cout << "before rangeFilter:" << cloud->points.size() << endl;
	cout << "after rangeFilter:" << cloud_f->points.size() << endl;

	vector<int> gridID;
	getGridID(*cloud_f, param, gridID);
	// gridID转换为IDMap
	unordered_multimap<int, int> IDMap;
	ID2Map(gridID, IDMap);

	// IDMap转化为GridMap
	unordered_map<int, Statistics> pcGridMap;
	pc2GridMap(*cloud_f, param, IDMap, pcGridMap);

	const int rowNum = param.height;
	const int columnNum = param.width;
	const float range_xy_max = param.range_xy_max;
	
	// normalization
	float gridPointsNumMax = 0;
	for (auto it = pcGridMap.begin(); it != pcGridMap.end(); ++it) {
		if (it->second.gridPointsNum > gridPointsNumMax) { 
			gridPointsNumMax = it->second.gridPointsNum; 
		}
	}

	cv::Mat bvimage = cv::Mat::zeros(rowNum, columnNum, CV_8UC1);		// CV_8U深度为0，C1为一个通道cn，灰度图
	
	int image_row_i, image_column_j;

	string filename;
	filename = "./file.txt";
	ofstream fout(filename.c_str());
	for (auto it = pcGridMap.begin(); it != pcGridMap.end(); ++it) {
		image_row_i = it->first / columnNum;
		image_column_j = it->first % columnNum;
		// 灰度图，0为黑，255为白
		bvimage.at<uchar>(image_row_i, image_column_j) = floor(it->second.gridPointsNum / gridPointsNumMax * 255); 

	}
	cv::imwrite("polar_grid_image.png", bvimage);

	cout << "image has been saved." << endl;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("pcd")); //PCLVisualizer 可视化类
	
	viewer->setBackgroundColor(0.8, 0.8, 0.8);
	viewer->addCoordinateSystem(1);
	// 原三维点云图也绘制出来
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZI> color(cloud_f, 0, 0, 0);
	viewer->addPointCloud(cloud_f, color, "cloud");

	while (!viewer->wasStopped()) {
		viewer->spin();
	}

	cout << "end" << endl;
    return 0;
}
