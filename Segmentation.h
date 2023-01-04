#ifndef PCL_PROJECT_V1_SEGMENTATION_H
#define PCL_PROJECT_V1_SEGMENTATION_H

#include "CommonProcesses.h"

class Segmentation : public CommonProcesses {
public:
    Segmentation();
    ~Segmentation();

    void colorizingPCD();

};


#endif //PCL_PROJECT_V1_SEGMENTATION_H
