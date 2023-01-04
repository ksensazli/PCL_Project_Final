#ifndef PCL_PROJECT_V1_REGION_GROWING_H
#define PCL_PROJECT_V1_REGION_GROWING_H

#include "Segmentation.h"

class Region_Growing : public Segmentation {
public:
    Region_Growing(string);
    ~Region_Growing();

    /// This creates a NormalEstimation object that can be used to compute normals for a point cloud.
    void estimation();
    /// This creates a RegionGrowing object that can be used to segment a point cloud into clusters.
    void growRegion(search::Search<PointXYZ>::Ptr, PointCloud<Normal>::Ptr);
    /// This prints the some cluster datas.
    void print(vector<PointIndices>);
    /// Display the point cloud with applied region growing segmentation
    void viewer_RG(PointCloud<PointXYZRGB>::Ptr);

private:
    /// cloud is a variable derived from type PointCloud and in the format XYZ
    PointCloud<PointXYZ>::Ptr cloud;
};


#endif //PCL_PROJECT_V1_REGION_GROWING_H
