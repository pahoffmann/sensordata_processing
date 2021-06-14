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
#include <sstream>


enum Status {
    BAD_FILE_READ,
    WRONG_FILE_FORMAT
};

class CultureInvariantPlyReader
{
    private:
        std::string in_file_name;
        char seperation = ',';
    public:


        CultureInvariantPlyReader(std::string file);

        Status Start(std::vector<cv::Point3d>& output_arr);
};