#include "CommonProcesses.h"
#include "Segmentation.h"
#include "Region_Growing.h"
#include "RANSAC.h"

int main() {
    string file_path = "depth_image0101_sampling.pcd";                      //enter the full path of pcd file. CLion IDE runs in cmake-build-debug base directory.

    CommonProcesses commonProcesses;
    Segmentation segmentation;
    Region_Growing regionGrowing(file_path);
    RANSAC ransac(file_path);

    //commonProcesses.readPCD_file(file_path);
    //commonProcesses.showPCD_data(file_path);
    //commonProcesses.scalePCD(5, file_path);                               //This '5' is multiplier for scale.
    //commonProcesses.samplePCD();
    //commonProcesses.viewPCD();

    //segmentation.colorizingPCD(file_path);
    //segmentation.extract_EC();

    //regionGrowing.estimation();                                           //estimation method runs all specs of region_growing class

    //ransac.run_RANSAC();

    return 0;
}
