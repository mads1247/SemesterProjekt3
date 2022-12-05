#ifndef CAM_H
#define CAM_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <pylon/PylonIncludes.h>

class cam

{
public:
    cam();
    int grabImmage(std::string sti);
};

#endif // CAM_H
