#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"
#include <numeric>

// Camera parameters
cv::Mat _camera_mat;
cv::Mat _dist_coeffs;
cv::Mat _R;
cv::Mat _T;


//lines
cv::Vec4i _left;
cv::Vec4i _right;

// red spectrum

cv::Scalar _low_red_l(175, 70, 170);
cv::Scalar _high_red_l(180, 255, 255);
cv::Scalar _low_red_r(0, 70, 170);
cv::Scalar _high_red_r(5, 255, 255);

/**
 * @brief reads camera parameters from the extrinsics yaml
 * 
 * @return int status code
 */
int readCameraParameters()
{
    cv::FileStorage fs;
    fs.open("extrinsicParams.yaml", cv::FileStorage::READ);
    
    //check if it could be opened
    if(!fs.isOpened())
    {
        std::cerr << "no intrinsics available!" << std::endl;
        return 1;
    }

    fs["camera_matrix"] >> _camera_mat;
    fs["distortion_coefficients"] >> _dist_coeffs;
    fs["rotation_mat"] >> _R;
    fs["translation_mat"] >> _T;

    std::cout << "Camera Matrix:" << std::endl << _camera_mat << std::endl;
    std::cout << "Dist Coeffs:" << std::endl << _dist_coeffs << std::endl;
    std::cout << "Rotation mat:" << std::endl << _R << std::endl;
    std::cout << "Translation mat:" << std::endl << _T << std::endl;

    return 0;
}

void findLines()
{
    cv::VideoCapture camera(0);
    cv::Mat frame, canny, res, hsv, mask_l, mask_r, mask;
    cv::namedWindow("Webcam");
    cv::namedWindow("Mask");
    cv::namedWindow("Canny");
    cv::namedWindow("Hough");
    std::vector<cv::Vec4i> linesP;
    std::vector<cv::Vec2f> lines;

    while(true)
    {
        camera >> frame;

        // convert to hsv

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);


        cv::inRange(hsv, _low_red_l, _high_red_l, mask_l);
        cv::inRange(hsv, _low_red_r, _high_red_r, mask_r);

        cv::bitwise_or(mask_l, mask_r, mask);

        cv::imshow("Mask", mask);

        cv::Canny(mask, canny, 50, 200, 3); // use canny to detect edges alla

        cv::imshow("Canny", canny);
        cv::HoughLines(canny, lines, 1, CV_PI / 180, 70, 0, 0);

        for( size_t i = 0; i < lines.size(); i++ )
        {
            std::cout << "line" << std::endl;
            float rho = lines[i][0], theta = lines[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line(frame, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
        }

        cv::imshow("Webcam", frame);
        cv::waitKey(10);
    }
    
} 

int main(int argc, char **argv)
{
    int status = readCameraParameters();
    if(status == 1)
    {
        std::cout << "couldnt read extrinsics" << std::endl;
        return EXIT_FAILURE;
    }

    findLines();
}