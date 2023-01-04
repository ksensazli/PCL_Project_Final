#ifndef PCL_PROJECT_V1_REGION_GROWING_H
#define PCL_PROJECT_V1_REGION_GROWING_H

#include "Segmentation.h"

class Region_Growing : public Segmentation {
public:
    Region_Growing(string);
    ~Region_Growing();

    void estimation();
    void growRegion(search::Search<PointXYZ>::Ptr, PointCloud<Normal>::Ptr);
    void print(vector<PointIndices>);
    void viewer_RG(PointCloud<PointXYZRGB>::Ptr);

private:
    /// cloud is a variable derived from type PointCloud and in the format XYZ
    PointCloud<PointXYZ>::Ptr cloud;
};


#endif //PCL_PROJECT_V1_REGION_GROWING_H
