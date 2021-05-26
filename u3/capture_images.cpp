#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"


int main(int argc, char** argv)
{

    cv::VideoCapture camera1(0);

    std::cout << __LINE__ << std::endl;

    cv::VideoCapture camera2(2);

    std::cout << __LINE__ << std::endl;

    cv::namedWindow("image1");
    cv::namedWindow("image2");

    cv::Mat image1, image2;

    while(true)
    {
        camera1 >> image1;
        camera2 >> image2;
        
        // find chessboard corners
        // if found: save them under specific name.

        int key = cv::waitKey(500);
        if(key == 27)
        {
            // add to array
        }
    }

}