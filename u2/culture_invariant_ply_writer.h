#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <numeric>

class CultureInvariantPlyWriter
{
    private:
        std::string out_file_name;
        std::vector<cv::Point3d> obj_points;
        char seperation = ',';
    public:
        CultureInvariantPlyWriter(std::string file, std::vector<cv::Point3d>& input_pts);

        void Start();
};