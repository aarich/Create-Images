#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

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

#include "../../Render/src/render.h"


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


int takeImage(string dirname, PointCloud<PT>::Ptr cloud, int w, int h, float r, float f, int imNum)
{
    Mat image = render::makeImagefromSize(cloud, w, h, r, f);
    ostringstream oss;
    oss << imNum;
    string filename = dirname + oss.str() + ".png";
    cout << filename << endl;
    imwrite(filename, image);
    return imNum + 1;
}

void createImages(string dirname, PointCloud<PT>::Ptr cloud, int nop,
                                                            int w, int h, float r, float f)
{
    cout << "Creating Images\n";

    // Find the bounding box so we know where to iterate through
    octree::OctreePointCloudSearch<PT> octree (r);
    octree.setInputCloud(cloud);
    octree.defineBoundingBox();
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);

    // ====== Assuming z is the vertical direction!! ====== //

    // For now, we'll stay at the same z value
    double z = 0.0; //(max_z + min_z)/2;

    // What is the spacing between locations?
    double xdiv = (max_x - min_x)/nop;
    double ydiv = (max_y - min_y)/nop;

    // Initialize transformation object that will hold all travel transformations
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.translation() << min_x - xdiv/2, min_y - ydiv/2, z;
    transformPointCloud (*cloud, *cloud, tf);
    
    cout << "Initializing Transforms\n";

    int ypp = 8;
    // Initialize yaw object
    Eigen::Affine3f yawtf = Eigen::Affine3f::Identity();
    yawtf.rotate (Eigen::AngleAxisf (0.785398163, Eigen::Vector3f::UnitZ()));
    // Initialize yawback object (To prevent numerical precision issues)
    Eigen::Affine3f yawbacktf = Eigen::Affine3f::Identity();
    yawbacktf.rotate (Eigen::AngleAxisf (-8 * 0.785398163, Eigen::Vector3f::UnitZ()));
    // Initialize pitchup object
    Eigen::Affine3f pitchuptf = Eigen::Affine3f::Identity();
    pitchuptf.rotate (Eigen::AngleAxisf (0.523598776, Eigen::Vector3f::UnitX()));
    // Initialize pitchdown object
    Eigen::Affine3f pitchdowntf = Eigen::Affine3f::Identity();
    pitchdowntf.rotate (Eigen::AngleAxisf (-1.047197552, Eigen::Vector3f::UnitX()));

    // Image Number. In the future, this should hold location values.
    // TODO Make this hold location of perspective.
    int imageNum = 0;

    cout << "Starting Image Creation\n";

    // visualization::PCLVisualizer v;
    // v.addPointCloud(cloud, "cloud");
    // v.spinOnce();

    for (int x = 0; x < nop; x++)
    {
        cout << x/nop << " percent done.\n";

        // Move to the new x!
        tf = Eigen::Affine3f::Identity();
        tf.translation() << xdiv, 0.0, 0.0;
        transformPointCloud(*cloud, *cloud, tf);

        cout << "Moved x\n";

        for (int y = 0; y < nop; y++)
        {
            // Move to the new y!
            tf = Eigen::Affine3f::Identity();
            tf.translation() << 0.0, ydiv, 0.0;
            transformPointCloud(*cloud, *cloud, tf);

            cout << "Moved y\n";

            // We're at a location in the map: (x, y, z) -- but it looks like the origin.
            // First look up
            transformPointCloud(*cloud, *cloud, pitchuptf);
            for (double yaw = 0; yaw < ypp; yaw++)
            {
                transformPointCloud(*cloud, *cloud, yawtf);
                imageNum = takeImage(dirname, cloud, w, h, r, f, imageNum);
                // v.updatePointCloud(cloud, "cloud");
                // v.spinOnce();
            }
            transformPointCloud(*cloud, *cloud, yawbacktf);
            // Then down
            transformPointCloud(*cloud, *cloud, pitchdowntf);
            for (double yaw = 0; yaw < ypp; yaw++)
            {
                transformPointCloud(*cloud, *cloud, yawtf);
                imageNum = takeImage(dirname, cloud, w, h, r, f, imageNum);
                // v.updatePointCloud(cloud, "cloud");
                // v.spinOnce();
            }
            transformPointCloud(*cloud, *cloud, yawbacktf);
            // Then back up to normal
            transformPointCloud(*cloud, *cloud, pitchuptf);
            for (double yaw = 0; yaw < ypp; yaw++)
            {
                transformPointCloud(*cloud, *cloud, yawtf);
                imageNum = takeImage(dirname, cloud, w, h, r, f, imageNum);
                // v.updatePointCloud(cloud, "cloud");
                // v.spinOnce();
            }
            transformPointCloud(*cloud, *cloud, yawbacktf);
        }
        // We've gone through all y values. We should go back all the way to initial y.
        tf = Eigen::Affine3f::Identity();
        tf.translation() << 0.0, -1*nop*ydiv, 0.0;
        transformPointCloud(*cloud, *cloud, tf);
    }
}

int main (int argc, char** argv)
{
    ifstream file(argv[1]);

    if ( !file.is_open() )
        return false;

    string str;

// Input Cloud Name
    getline( file, str ); string cloud_name = str;
// Directory to save photos
    getline( file, str ); string dirname = str;
// // rollperpoint
//     getline( file, str ); int rpp = atoi(str.c_str());
// // pitchperpoint
//     getline( file, str ); int ppp = atoi(str.c_str());
// // yawperpoint
//     getline( file, str ); int ypp = atoi(str.c_str());
// // number of places (NOT "no operation")
    getline( file, str ); int nop = atoi(str.c_str());
// width
    getline( file, str ); int w = atoi(str.c_str());
// height
    getline( file, str ); int h = atoi(str.c_str());
// resolution
    getline( file, str ); float r = atof(str.c_str());
// f (distance from pinhole)
    getline( file, str ); float f = atof(str.c_str());

    file.close();

    PointCloud<PT>::Ptr cloud (new pcl::PointCloud<PT>);
    io::loadPCDFile<PT>(cloud_name, *cloud);

    // Transform kinect cloud so that it aligns with Matterport cloud (axes).
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate( Eigen::AngleAxisf (-90, Eigen::Vector3f::UnitY()));
    // transformPointCloud(*cloud, *cloud, transform);

    createImages(dirname, cloud, nop, w, h, r, h);

    return 0;
}