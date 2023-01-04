#include "Region_Growing.h"

Region_Growing::Region_Growing(string file_path) {
    this->cloud.reset(new PointCloud<PointXYZ>);
    //This function loads a point cloud from a PCD file and stores it in the cloud object. The <PointXYZ> template parameter specifies the type of points in the point cloud.
    if (io::loadPCDFile<PointXYZ>(file_path, *cloud) == -1) {
        cout << "Cloud reading failed." << endl;
    }
}

Region_Growing::~Region_Growing() {

}

void Region_Growing::estimation() {
    //This creates a pointer to a Search object with PointXYZ points and assigns it to the variable tree.
    //The search::KdTree class is a type of search method that uses a k-d tree data structure to quickly search for the nearest neighbors of a given point.
    search::Search<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    //This creates a pointer to a PointCloud object with Normal points and assigns it to the variable normals.
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    //This creates a NormalEstimation object that can be used to compute normals for a point cloud.
    //The <PointXYZ, Normal> template parameters specify the types of points and normals, respectively, that the object will be working with.
    NormalEstimation<PointXYZ, Normal> normal_estimator;
    //This sets the search method to be used by the NormalEstimation object to the tree object.
    normal_estimator.setSearchMethod(tree);
    //This sets the input point cloud for the NormalEstimation object to the cloud object.
    normal_estimator.setInputCloud(cloud);
    //This sets the number of nearest neighbors to consider when computing the normals to 50.
    normal_estimator.setKSearch(50);
    //This computes the normals for the input point cloud and stores the result in the normals object.
    normal_estimator.compute(*normals);

    growRegion(tree, normals);
}

void Region_Growing::growRegion(search::Search<PointXYZ>::Ptr tree, PointCloud<Normal>::Ptr normals) {
    //This creates a pointer to a vector of integers and assigns it to the variable indices.
    IndicesPtr indices(new vector<int>);
    //This removes any points with NaN (Not a Number) values from the cloud object and stores the indices of the remaining points in the indices vector.
    removeNaNFromPointCloud(*cloud, *indices);

    //This creates a RegionGrowing object that can be used to segment a point cloud into clusters.
    //The <PointXYZ, Normal> template parameters specify the types of points and normals, respectively, that the object will be working with.
    RegionGrowing<PointXYZ, Normal> reg;
    //This sets the minimum size for a cluster to be considered valid to 50 points.
    reg.setMinClusterSize(50);
    //This sets the maximum size for a cluster to be considered valid to 1,000,000 points.
    reg.setMaxClusterSize(1000000);
    //This sets the search method to be used by the RegionGrowing object to the tree object.
    reg.setSearchMethod(tree);
    //This sets the number of nearest neighbors to consider when performing the region growing to 30.
    reg.setNumberOfNeighbours(30);
    //This sets the input point cloud for the RegionGrowing object to the cloud object.
    reg.setInputCloud(cloud);
    //This sets the indices of the points in the input point cloud that should be considered for the region growing to the indices vector.
    reg.setIndices(indices);
    //This sets the normals for the points in the input point cloud that should be used for the region growing to the normals object.
    reg.setInputNormals(normals);
    //This sets the smoothness threshold for the region growing, which determines the maximum angle between normals of neighboring points that will be considered for the region growing.
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    //This sets the curvature threshold for the region growing, which determines the maximum curvature of points that will be considered for the region growing.
    reg.setCurvatureThreshold(1.0);

    //This creates a vector of PointIndices objects to store the clusters.
    vector<PointIndices> clusters;
    //This performs the region growing and stores the resulting clusters in the clusters vector.
    reg.extract(clusters);
    //This retrieves the colored point cloud resulting from the region growing, where each cluster is assigned a different color, and stores it in the colored_cloud object.
    PointCloud<PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

    //This performs print the specs of the clusters.
    print(clusters);
    //This creates a CloudViewer object that can be used to visualize a point cloud and assigns it to the variable viewer.
    viewer_RG(colored_cloud);
}

void Region_Growing::print(vector<PointIndices> clusters) {
    cout << "Number of clusters is equal to " << clusters.size() << endl;
    cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
    cout << "These are the indices of the points of the initial" <<
         endl << "cloud that belong to the first cluster:" << endl;
    size_t counter = 0;
    while (counter < clusters[0].indices.size()) {
        cout << clusters[0].indices[counter] << ", ";
        counter++;
        if (counter % 10 == 0)
            cout << endl;
    }
    cout << endl;
}

void Region_Growing::viewer_RG(PointCloud<PointXYZRGB>::Ptr viewCloud) {
    //The string parameter specifies the name of the window that will be created to display the point cloud.
    visualization::CloudViewer viewer("Region Growing Viewer");
    //This displays the colored_cloud object in the CloudViewer window.
    viewer.showCloud(viewCloud);
    //This starts a loop that runs until the CloudViewer window is closed.
    while (!viewer.wasStopped()) {
        // Do nothing, just wait until the viewer is closed
    }
}