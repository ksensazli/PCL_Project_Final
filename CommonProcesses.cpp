#include "CommonProcesses.h"

CommonProcesses::CommonProcesses()
{

}

CommonProcesses::~CommonProcesses() {

}

void CommonProcesses::readPCD_file(string file_path) {
    this->cloud.reset(new PointCloud<PointXYZ>);

    // Load the PCL file
    if (io::loadPCDFile<PointXYZ>(file_path, *this->cloud) == -1) {
        cout << "Couldn't read file " << file_path << "\n";
    } else {
        cout << "Read the PCD file operation has finished successfully. \n";
    }
}

void CommonProcesses::showPCD_data(string file_path) {
    cout << "Loaded " << this->cloud->width * this->cloud->height
         << " data points from " << file_path << " with the following fields: "
         << endl;

    for (const auto &point: *this->cloud) {
        // This loop is made to copy variables into dynamic arrays for any 'save as' operations that can be done.
        for (int i = 0; i < this->cloud->height; i++) {
            this->pointX_Pcd[i] = point.x;
            this->pointY_Pcd[i] = point.y;
            this->pointZ_Pcd[i] = point.z;
        }
        cout << "    " << point.x << " " << point.y << " " << point.z << endl;
    }
}

void CommonProcesses::scalePCD(double multiplier, string file_path) {
    //this->cloud->resize(this->cloud->width * multiplier, this->cloud->height * multiplier);

    cout << "After the scaling: \n";
    cout << "Loaded " << this->cloud->width * this->cloud->height
         << " data points from " << file_path << endl;
}

void CommonProcesses::samplePCD() {
    // ------------------------------!!
    // This sampling code not working!!
    // ------------------------------!!
    UniformSampling<PointXYZ> uniformSampling;
    uniformSampling.setInputCloud(this->cloud);
    uniformSampling.filter(*this->cloud);
}

void CommonProcesses::viewPCD() {
    visualization::CloudViewer viewer("Point Cloud Viewer");
    viewer.showCloud(this->cloud);
    while (!viewer.wasStopped()) {
        // Do nothing, just wait until the viewer is closed
    }
}
