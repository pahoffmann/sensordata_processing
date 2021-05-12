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

cv::Scalar _low_red_l(175, 70, 160);
cv::Scalar _high_red_l(180, 255, 255);
cv::Scalar _low_red_r(0, 70, 160);
cv::Scalar _high_red_r(5, 255, 255);


// futher params

// maximum angle variance
int _max_angle_var = 5;
int _max_dist_var = 20;

// camera resolution
int _image_width = 640;
int _image_height = 480;

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

/**
 * @brief Goes through the lines detected by the hough transform and returns the average lines for each direction
 * 
 * @param in_lines lines found by the hough transform
 * @param out_line1  first avg line
 * @param out_line2  second avg line
 * @return int status, if the operation was successful
 */
int calcCenterLine(std::vector<cv::Vec2f> &in_lines, cv::Vec4i out_line1, cv::Vec4i out_line2)
{
    if(in_lines.size() < 2)
    {
        return 1;
    }

    int counter_l = 0;
    int counter_r = 0;

    cv::Vec2f avg_1(0,0);
    cv::Vec2f avg_2(0,0);

    for(auto& line : in_lines)
    {
        float rho = line[0], theta = line[1];
        
        if(avg_1[0] == 0 && avg_2[0] == 0 )
        {
            avg_1[0] += rho;
            avg_1[0] += theta;
            counter_l += 1;
        }
        else if(avg_1[0] == 0 && avg_2[0] != 0) // one already filled
        {
            // if the lines matches the other one
            if(cv::abs(avg_2[0] / counter_r  -  rho) <= _max_dist_var && cv::abs(avg_2[1] / counter_r  -  theta) <= _max_angle_var)
            {
                avg_2[0] += rho;
                avg_2[1] += theta;
                counter_r++;
            }
            else
            {
                avg_1[0] += rho;
                avg_1[1] += theta;
                counter_l++;
            }

        }
        else if(avg_2[0] == 0 && avg_1[0] != 0)
        {
            
        }
        else{

        }
    }

    

    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
    cv::Point pt1, pt2;


    cv::Vec4i vec(pt1.x, pt1.y, pt2.x, pt2.y);

}

void findLines()
{
    cv::VideoCapture camera(0);
    cv::Mat frame, canny, res, hsv, mask_l, mask_r, mask;
    cv::namedWindow("Webcam");
    cv::namedWindow("Mask");
    cv::namedWindow("Canny");
    cv::namedWindow("Hough");
    std::vector<cv::Vec2f> lines;
    std::vector<cv::Vec4i> pix_lines;
    cv::Vec4i avg_line1;
    cv::Vec4i avg_line2;

    while(true)
    {
        camera >> frame;
        //std::cout << "Cols: " << frame.cols << " Rows: " << frame.rows << std::endl;

        // convert to hsv

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);


        cv::inRange(hsv, _low_red_l, _high_red_l, mask_l);
        cv::inRange(hsv, _low_red_r, _high_red_r, mask_r);

        cv::bitwise_or(mask_l, mask_r, mask);

        cv::imshow("Mask", mask);

        cv::Canny(mask, canny, 50, 200, 3); // use canny to detect edges alla

        cv::imshow("Canny", canny);

        //applying houghlines operation on the canny image
        cv::HoughLines(canny, lines, 1, CV_PI / 180, 70, 0, 0);
        
        //for each line, which exceeds the threshold, calc endpoints
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0], theta = lines[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));

            //std::cout << "X1 " << pt1.x << "Y1 " << pt1.y <<"X2 " << pt2.x <<"Y2 " << pt2.y << std::endl;
            
            cv::line(frame, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
            //cv::Vec4i vec(pt1.x, pt1.y, pt2.x, pt2.y);
            //pix_lines.push_back(vec);
        }


        calcCenterLine(lines, avg_line1, avg_line2);

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