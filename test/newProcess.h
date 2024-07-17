#include "configurator.h"
#include "libcam2opencv.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
#include "Iir.h"
#include "CppTimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#define _USE_MATH_DEFINES

std::vector <BodyFeatures> WorldBuilder::processData(CoordinateContainer points){
    std::vector <BodyFeatures> result;
    BodyFeatures feature;
    cv::Rect2f getRect(nb, feature);
    result.push_back(feature);
    return result;
}