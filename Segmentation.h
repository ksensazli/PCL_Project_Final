#ifndef PCL_PROJECT_V1_SEGMENTATION_H
#define PCL_PROJECT_V1_SEGMENTATION_H

#include "CommonProcesses.h"

class Segmentation : public CommonProcesses {
public:
    Segmentation();
    ~Segmentation();

    /// Colorizing the all segments/edges of the given PCD file
    void colorizingPCD(string);
    /// Display the point cloud data after the colorizing
    void viewerPCD(PointCloud<PointXYZRGB>::Ptr);
    /// This performs extract the euclidean clusters
    void extract_EC();

private:
    /// cloud is a variable derived from type PointCloud and in the format XYZ
    PointCloud<PointXYZ>::Ptr cloud;

};


#endif //PCL_PROJECT_V1_SEGMENTATION_H
