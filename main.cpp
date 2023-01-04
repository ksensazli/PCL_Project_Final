#include "CommonProcesses.h"
#include "Segmentation.h"
#include "Region_Growing.h"
#include "RANSAC.h"

int main() {
    string file_path = "depth_image0101_sampling.pcd";

    CommonProcesses commonProcesses;
    Segmentation segmentation;
    Region_Growing regionGrowing(file_path);
    RANSAC ransac;

    commonProcesses.readPCD_file(file_path);
    //commonProcesses.showPCD_data(file_path);
    //commonProcesses.scalePCD(5, file_path);                               //This '5' is multiplier for scale.
    //commonProcesses.samplePCD();
    //commonProcesses.viewPCD();

    //segmentation.colorizingPCD(file_path);
    //segmentation.extract_EC();

    regionGrowing.estimation();

    return 0;
}
