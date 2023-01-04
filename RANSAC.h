#ifndef PCL_PROJECT_V1_RANSAC_H
#define PCL_PROJECT_V1_RANSAC_H

#include "Segmentation.h"

class RANSAC : public Segmentation {
public:
    RANSAC(string);
    ~RANSAC();

    /// Create RandomSampleConsensus object and compute the appropriated model
    void run_RANSAC();
    /// Display the point cloud
    void view_RANSAC(PointCloud<PointXYZ>::ConstPtr cloud);

private:
    /// cloud is a variable derived from type PointCloud and in the format XYZ
    PointCloud<PointXYZ>::Ptr cloud;
};


#endif //PCL_PROJECT_V1_RANSAC_H
