#include "RANSAC.h"

RANSAC::RANSAC(string file_path) {
    this->cloud.reset(new PointCloud<PointXYZ>);
    //This function loads a point cloud from a PCD file and stores it in the cloud object. The <PointXYZ> template parameter specifies the type of points in the point cloud.
    if (io::loadPCDFile<PointXYZ>(file_path, *cloud) == -1) {
        cout << "Cloud reading failed." << endl;
    }
}

RANSAC::~RANSAC() {

}

void RANSAC::run_RANSAC() {
    PointCloud<PointXYZ>::Ptr final(new PointCloud<PointXYZ>);
    vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model
    SampleConsensusModelPlane<PointXYZ>::Ptr model_p(new SampleConsensusModelPlane<PointXYZ>(cloud));

    RandomSampleConsensus<PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    // copies all inliers of the model computed to another PointCloud
    copyPointCloud(*cloud, inliers, *final);

    view_RANSAC(final);
}

void RANSAC::view_RANSAC(PointCloud<PointXYZ>::ConstPtr viewCloud) {
    visualization::CloudViewer viewer("RANSAC Viewer");
    viewer.showCloud(viewCloud);
    while (!viewer.wasStopped()) {
        // Do nothing, just wait until the viewer is closed
    }
}
