#include "Segmentation.h"

Segmentation::Segmentation() {

}

Segmentation::~Segmentation() {

}

void Segmentation::colorizingPCD(string file_path) {
    this->cloud.reset(new PointCloud<PointXYZ>);
    io::loadPCDFile(file_path, *cloud);

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

    viewerPCD(color_cloud);
}

void Segmentation::viewerPCD(PointCloud<PointXYZRGB>::Ptr viewCloud) {
    visualization::CloudViewer viewer("Point Cloud Viewer after Colorizing");
    viewer.showCloud(viewCloud);
    while (!viewer.wasStopped()) {
        // Do nothing, just wait until the viewer is closed
    }
}

void Segmentation::extract_EC() {
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    VoxelGrid<PointXYZ> vg;
    PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    SACSegmentation<PointXYZ> seg;
    PointIndices::Ptr inliers (new PointIndices);
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointCloud<PointXYZ>::Ptr cloud_plane (new PointCloud<PointXYZ> ());
    PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    PointCloud<PointXYZ>::Ptr cloud_f (new PointCloud<PointXYZ>);

    int i=0, nr_points = (int) cloud_filtered->size ();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            cout << "Could not estimate a planar model for the given dataset." << endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        ExtractIndices<PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloud<PointXYZ>::Ptr cloud_cluster (new PointCloud<PointXYZ>);
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << endl;
        stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
    }
}
