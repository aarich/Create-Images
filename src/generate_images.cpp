#include <iostream>
#include <stdio.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/filter.h>

#include "../../Render/src/Render.h"


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "cvaux.h"

#include <pcl/io/io.h>

#define PT PointXYZRGB

using namespace pcl;
using namespace cv;
using namespace std;

void createImages(PointCloud<PT> cloud, int rollPerPoint, int pitchPerPoint, int yawPerPoint)
{
	Mat gotImage(width, size)

	makeImagefromSize(cloud, width, height, resolution, f);
}

int main (int argc, char** argv)
{
	string cloud_name;
    ifstream file(argv[1]);

    if ( !file.is_open() )
        return false;

    string str;

    getline( file, str );
    cloud_name = str;

    // width
    getline( file, str );
    int w = atoi(str.c_str());
    // height
    getline( file, str );
    int h = atoi(str.c_str());
    // resolution
    getline( file, str );
    float r = atof(str.c_str());
    // f (distance from pinhole)
    getline( file, str );
    float f = atof(str.c_str());
    // theta
    getline( file, str );
    float theta = atof(str.c_str());

    file.close();

    PointCloud<PT>::Ptr cloud (new pcl::PointCloud<PT>);

    io::loadPCDFile<PT>(cloud_name, *cloud);
}