#ifndef PCL_PROJECT_V1_COMMONPROCESSES_H
#define PCL_PROJECT_V1_COMMONPROCESSES_H

#include <iostream>
#include <vector>
#include <map>
#include <cstdlib>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

class CommonProcesses {
public:
    CommonProcesses();
    ~CommonProcesses();

    /// Load the point cloud from the given PCD file
    void readPCD_file(string);
    /// Show the data of the point cloud data (in console)
    void showPCD_data(string);
    /// Scaling the point cloud data
    void scalePCD(double, string);
    /// Uniform sampling the point cloud file
    void samplePCD();
    /// Display the point cloud
    void viewPCD();

private:
    /// cloud is a variable derived from type PointCloud and in the format XYZ
    PointCloud<PointXYZ>::Ptr cloud;
    /// pointX_Pcd, pointY_Pcd, pointZ_Pcd used for dynamic arrays
    double *pointX_Pcd, *pointY_Pcd, *pointZ_Pcd;

};


#endif //PCL_PROJECT_V1_COMMONPROCESSES_H
