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

#define PCL_NO_PRECOMPILE
using namespace std;

const float PI = 3.1415926;


// 定义grid中需要保存的数据
struct Statistics {
	int gridPointsNum;		// grid中的点数
	float intensityMean;	// grid中点的平均intensity，在测试文件中的bin是以浮点数形式表示的，而txt是以整形表示的
	int zMinPointID;		// 记录最小的z坐标对应的pointID
	vector<int> gridToPointID;	// grid中的点的pointID
	int gCandiFlag;			// 判断是否为地面候选网格的flag，0表示网格为空，-1表示非候选，1表示候选
};

// 定义转换的参数
struct Param {
	int width;
	int height;
	float range_xy_max;
};

// 定义地面分割参数
struct gSegParam {
	float th_slope;
	float th_dist;
	float th_zMin;
};

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
	Statistics statistic;
	for (int i = 0; i < gridNum; ++i) {
		int len = IDmapIn.count(i);
		vector<int> pointID;
		if(len == 0) {
			statistic.gridPointsNum = 0;
			statistic.gridToPointID = pointID;
			statistic.intensityMean = 0;
			statistic.zMinPointID = 0;
			statistic.gCandiFlag = 0;
			unordered_mapOut.insert(make_pair(i, statistic));
			continue;
		}
		float intensitySum = 0;
		auto keyRange = IDmapIn.equal_range(i);
		int zMinPointID = keyRange.first->second;
		for (auto it = keyRange.first; it != keyRange.second; ++it) {
			pointID.push_back(it->second);
			intensitySum += cloudIn.points[it->second].intensity;
			if (cloudIn.points[it->second].z < cloudIn.points[zMinPointID].z) {
				zMinPointID = it->second;
			}
		}
		statistic.gridPointsNum = len;
		statistic.intensityMean = intensitySum / len;
		statistic.zMinPointID = zMinPointID;
		statistic.gridToPointID = pointID;
		statistic.gCandiFlag = 0;	// 是否为地面候选的FLAG暂时定为0，另写函数修改这个flag
		unordered_mapOut.insert(make_pair(i, statistic));
	}
}

/******************************************/
/************ Ground Remove****************/
/******************************************/
// 似乎param中只需要一个gridNum，用unordered_map的size即可，不用再多传这个参数
void extract_grid_candidate(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, 
		unordered_map<int, Statistics>& unordered_mapInOut, gSegParam param) {
	float th_zMin = param.th_zMin;
	int gridNum = unordered_mapInOut.size();
	for (int i = 0; i < gridNum; ++i) {
		auto value = unordered_mapInOut.find(i); // 从map中找到key为i的值，因为map是无序的，因此必须这样找
		int zMinPointID = value->second.zMinPointID;	// 找到后取出zMinPointID
		if (value->second.gridPointsNum == 0) {
			continue; // 表示该网格没有数据，unordered_mapInOut中的值不用动
		} else if (cloudIn.points[zMinPointID].z < th_zMin) {
			value->second.gCandiFlag = 1;	// 
		} else {
			value->second.gCandiFlag = -1;	// 表示该网格的点不可能是地面
		}
	}
}

float caud(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, int i, int j) {
	if (i == j) return 0;
	
	float dx = cloudIn.points[i].x - cloudIn.points[j].x;
	float dy = cloudIn.points[i].y - cloudIn.points[j].y;
	float dz = cloudIn.points[i].z - cloudIn.points[j].z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

float caua(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, int i, int j) {
	if (i == j) return 0;
	float dx = cloudIn.points[i].x - cloudIn.points[j].x;
	float dy = cloudIn.points[i].y - cloudIn.points[j].y;
	float dz = cloudIn.points[i].z - cloudIn.points[j].z;
	float theta = atan(fabs(dz) / sqrt(dx * dx + dy * dy)) * 180 / PI;
	return theta;
}



// 参数有点太多了吧，其实候选点云，gGridCandiIn也可以归入到mapIn中，这样能少一个参数
void GroundSeg(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, 
		const gSegParam param, unordered_map<int, Statistics>& unordered_mapIn, 
		vector<int>& g_pcID, vector<int>& ng_pcID) {
	float th_dist = param.th_dist;
	float th_slope = param.th_slope;
	int gridNum = unordered_mapIn.size();
	for (int i = 0; i < gridNum; ++i) {
		auto value = unordered_mapIn.find(i);
		int gflag = value->second.gCandiFlag;
		vector<int> PointID = value->second.gridToPointID;
		if (gflag == 0) {
			continue;
		} else if (gflag == -1) {
			for (int j = 0; j < PointID.size(); ++j) {
				ng_pcID.push_back(PointID[j]);
			}
		} else {
			for (int j = 0; j < PointID.size(); ++j) {
				int ID0 = value->second.zMinPointID;
				int IDp = PointID[j];
				if (caud(cloudIn, ID0, IDp) < th_dist || caua(cloudIn, ID0, IDp) < th_slope) {
					g_pcID.push_back(IDp);
				} else {
					ng_pcID.push_back(IDp);
				}
			}
		}
	}
}

void gReCallPort(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, 
		unordered_map<int, Statistics>& unordered_mapIn,
		const gSegParam param, 
		pcl::PointCloud<pcl::PointXYZI>& g_cloud, 
		pcl::PointCloud<pcl::PointXYZI>& ng_cloud) {
	
	extract_grid_candidate(cloudIn, unordered_mapIn, param);
	vector<int> g_pcID, ng_pcID;
	GroundSeg(cloudIn, param, unordered_mapIn, g_pcID, ng_pcID);
	for (int i = 0; i < g_pcID.size(); ++i) {
		g_cloud.push_back(cloudIn.points[g_pcID[i]]);
	}
	for (int j = 0; j < ng_pcID.size(); ++j) {
		ng_cloud.push_back(cloudIn.points[ng_pcID[j]]);
	}
}

int main(int argc, char** argv) {

	// 因为这里定义的都是指针，因此下文中函数参数那里统统都是*cloud等
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(
			new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_g(
			new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ng(
			new pcl::PointCloud<pcl::PointXYZI>);

	
	fileToCloud(argv[1], cloud); // 不给模板参数，让编译器自己推导即可
	
	Param param;
	param.height = atoi(argv[2]);
	param.width = atoi(argv[3]);
	param.range_xy_max = atof(argv[4]);

	gSegParam gparam;
	gparam.th_zMin = -0.4;	// 如何知道这个参数值呢？这种做法看来还是有问题的
	gparam.th_dist = 0.25;
	gparam.th_slope = 25;
	
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

	gReCallPort(*cloud_f, pcGridMap, gparam, *cloud_g, *cloud_ng);

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
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZI> color(cloud_g, 0, 0, 0);
	viewer->addPointCloud(cloud_g, color, "cloud");

	while (!viewer->wasStopped()) {
		viewer->spin();
	}

	cout << "end" << endl;
    return 0;
}
