#include "Segmentation.h"

Segmentation::Segmentation() {

}

Segmentation::~Segmentation() {

}

void Segmentation::colorizingPCD() {
    PointCloud<PointXYZRGB>::Ptr color_cloud(new PointCloud<PointXYZRGB>);
    for (size_t i = 0; i < cloud->points.size(); i++) {
        PointXYZRGB point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        point.r = 255;
        point.g = 0;
        point.b = 0;
        color_cloud->points.push_back(point);
    }

    visualization::CloudViewer viewer("Point Cloud Viewer after Colorizing");
    viewer.showCloud(color_cloud);
    while (!viewer.wasStopped()) {
        // Do nothing, just wait until the viewer is closed
    }
}
