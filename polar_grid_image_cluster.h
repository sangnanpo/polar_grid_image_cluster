#ifndef POLAR_GRID_IMAGE_CLUSTER_
#define POLAR_GRID_IMAGE_CLUSTER_


// 定义grid中需要保存的数据
struct Statistics {
	int gridPointsNum;		// grid中的点数
	vector<int> gridToPointID;	// grid中的点的pointID
};

// 定义转换的参数
struct Param {
	int width;
	int height;
	float range_xy_max;
};


#endif