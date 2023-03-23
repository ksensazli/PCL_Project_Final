# Point Cloud Library Project for Object Oriented Programming I Course
Object Oriented Programming I Course PCL Project

# ➢ Description

**1. CommonProcesses** <br /> <br />
&emsp; This is a C++ class called CommonProcesses that provides functions for working with point cloud data stored in PCD (Point Cloud Data) files.
The class has a member variable cloud of type PointCloud<PointXYZ> which is a data structure for storing point cloud data in the PCL (Point Cloud Library) library. <br />
 <br />&emsp; The class has the following functions: <br /> <br />
**• readPCD_file(string file_path)** - This function reads a PCD file from the given file_path and stores the data in the cloud member variable. If the file could not be read, it prints an error message. <br />
**• showPCD_data(string file_path)** - This function iterates through the points in the cloud member variable and prints the x, y, and z coordinates of each point. It also stores the x, y, and z coordinates in dynamic arrays pointX_Pcd, pointY_Pcd, and pointZ_Pcd respectively.<br />
**• scalePCD(double multiplier, string file_path)** - This function scales the cloud member variable by the given multiplier and then prints the number of points in the scaled cloud. <br />
**• samplePCD()** - This function uses the UniformSampling filter from the PCL library to downsample the cloud member variable. <br />
**• viewPCD()** - This function opens a window for visualizing the cloud member variable using the PCL visualization library. The window remains open until it is closed by the user. <br /> <br />
**2. Segmentation** <br /> <br />
&emsp; This is a C++ class called Segmentation that provides functions for colorizing and segmenting point cloud data stored in PCD (Point Cloud Data) files.
The class has a member variable cloud of type PointCloud<PointXYZ> which is a data structure for storing point cloud data in the PCL (Point Cloud Library) library. <br />
<br /> &emsp; The class has the following functions: <br /> <br />
**• colorizingPCD(string file_path)** - This function reads a PCD file from the given file_path and stores the data in the cloud member variable. It then creates a new point cloud of type PointCloud<PointXYZRGB> where each point in the original cloud is colored red (255, 0, 0). The colored point cloud is then passed to the viewerPCD() function for visualization. <br />
**• viewerPCD(PointCloud<PointXYZRGB>::Ptr viewCloud)** - This function opens a window for visualizing the given point cloud viewCloud using the PCL visualization library. The window remains open until it is closed by the user. <br />
**• extract_EC()** - This function segments the cloud member variable into planes and returns the inliers (points belonging to the planes) and outliers (points not belonging to the planes). It does this by first downsampling the point cloud using a voxel grid filter with a leaf size of 1cm. It then fits a plane model to the downsampled point cloud using the RANSAC (Random Sample Consensus) algorithm and extracts the inliers as the points belonging to the plane. The process is repeated until there are fewer than 30% of the original number of points in the point cloud. The inliers and outliers are then returned as separate point clouds. <br />
**• extract_clusters()** - This function segments the cloud member variable into clusters using the Euclidean Clustering algorithm. It first creates a search tree using the KdTree (K-Dimensional Tree) data structure and then uses the tree to extract the clusters. The function returns a vector of point clouds, where each point cloud corresponds to a cluster. <br /> <br />
**3. Region_Growing** <br /> <br />
&emsp; This is a C++ class called Region_Growing that provides functions for performing region growing on a point cloud stored in a PCD (Point Cloud Data) file.
The class has a member variable cloud of type PointCloud<PointXYZ> which is a data structure for storing point cloud data in the PCL (Point Cloud Library) library. <br />
<br />&emsp; The class has the following functions: <br /> <br />
**• Region_Growing(string file_path)** - This is the constructor for the class. It reads a PCD file from the given file_path and stores the data in the cloud member variable. If the file could not be read, it prints an error message. <br />
**• ~Region_Growing()** - This is the destructor for the class. It is called when the object is destroyed. <br />
**• estimation()** - This function computes normals for the points in the cloud member variable and then calls the growRegion() function to perform region growing on the point cloud. <br />
**• growRegion(search::Search<PointXYZ>::Ptr tree, PointCloud<Normal>::Ptr normals)** - This function performs region growing on the cloud member variable using the given search method tree and normals normals. It creates a RegionGrowing object from the PCL library and sets various parameters for the region growing process, such as the minimum and maximum cluster sizes, the number of nearest neighbors to consider, and the smoothness and curvature thresholds. It then performs the region growing and visualizes the resulting clusters using the PCL visualization library. <br />
**• visualize()** - This function visualizes the cloud member variable using the PCL visualization library. The window remains open until it is closed by the user. <br /> <br />
**4. RANSAC** <br /> <br />
&emsp; This is a C++ class called RANSAC that provides functions for performing RANSAC (Random Sample Consensus) on a point cloud stored in a PCD (Point Cloud Data) file.
The class has a member variable cloud of type PointCloud<PointXYZ> which is a data structure for storing point cloud data in the PCL (Point Cloud Library) library. <br />
<br />&emsp; The class has the following functions: <br /> <br />
**• RANSAC(string file_path)** - This is the constructor for the class. It reads a PCD file from the given file_path and stores the data in the cloud member variable. If the file could not be read, it prints an error message. <br />
**• ~RANSAC()** - This is the destructor for the class. It is called when the object is destroyed. <br />
**• run_RANSAC()** - This function performs RANSAC on the cloud member variable to identify a plane in the point cloud. It creates a SampleConsensusModelPlane object from the PCL library which is used to fit a plane to the point cloud. It then creates a RandomSampleConsensus object and uses it to compute the model and identify the inliers (points that fit the model). It stores the inliers in a new PointCloud object and then calls the view_RANSAC() function to visualize the resulting point cloud. <br />
**• view_RANSAC(PointCloud<PointXYZ>::ConstPtr viewCloud)** - This function visualizes the given viewCloud point cloud using the PCL visualization library. The window remains open until it is closed by the user. <br />
# ➢ Implementation of the Project
This Project was developed and builded using CLion 2022.3 in Ubuntu 20.04.5 LTS environment. <br />
Libraries : PCL(Point Cloud Library) 1.10 <br />
CMake : CMake version 3.24 <br />
C++ Standart : C++ 11 <br />
# ➢ How to install PCL 1.10 on Ubuntu 20.04.5 LTS
To install PCL 1.10 in Ubuntu 20.04.5 LTS environment, some commands are needed first. The commands I used during the installation phase are as follows: <br /> <br />
--sudo apt update <br />
--sudo apt upgrade <br />
--sudo apt install build-essential <br />
--sudo apt install libpcl-dev <br /> <br />
After the packages triggered by the commands are installed, we need to prepare a CMakeList by creating a new project on CLion 2022.3. Example CMakeList is given below: <br /> <br />
cmake_minimum_required(VERSION 3.24) <br />
project(PCL_Project_v1) <br />
set(CMAKE_CXX_STANDARD 11) <br />
find_package(PCL 1.10 REQUIRED) <br />
include_directories(${PCL_INCLUDE_DIRS}) <br />
link_directories(${PCL_LIBRARY_DIRS}) <br />
add_definitions(${PCL_DEFINITIONS}) <br />
add_executable(PCL_Project_v1 main.cpp) <br />
target_link_libraries (PCL_Project_v1 ${PCL_LIBRARIES}) <br /> <br />
With this CMakeList, we introduce the directories containing all the necessary files of the PCL library to the IDE. I also require the minimum PCL 1.10 library to be installed for the project to run. <br />
# ➢ How to run PCL_Project_v1
To run this project, it is sufficient to remove the comment line (//) command at the beginning of the desired methods from the objects left ready in main.cpp. The documentation can be reviewed for the functionality of the desired methods. The variable given as file_path must contain the file path of the corresponding pcd file. For example; In the CLion 2022.3 environment, the desired path can be given to compile and run this project, or when the pcd files are copied into the cmake-build-debug folder in the relevant project folder, it can be run only with the file name without specifying any path.
