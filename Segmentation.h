#ifndef PCL_PROJECT_V1_SEGMENTATION_H
#define PCL_PROJECT_V1_SEGMENTATION_H

#include "CommonProcesses.h"

class Segmentation : public CommonProcesses {
public:
    Segmentation();
    ~Segmentation();

    void colorizingPCD(string);
    void viewerPCD(PointCloud<PointXYZRGB>::Ptr);
    void extract_EC();

private:
    /// cloud is a variable derived from type PointCloud and in the format XYZ
    PointCloud<PointXYZ>::Ptr cloud;

};


#endif //PCL_PROJECT_V1_SEGMENTATION_H
