#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"
#include <numeric>
#include "culture_invariant_ply_writer.h"

// Camera parameters
cv::Mat _camera_mat;
cv::Mat _dist_coeffs;
cv::Mat _R;
cv::Mat _T;

// slope line struct stuff
struct category {
    double avg_slope_angle;
    double max_length;
    cv::Vec4i longest_line;
    std::vector<cv::Vec4i> lines;
};

//lines
cv::Vec4i _left;
cv::Vec4i _right;

// red spectrum

cv::Scalar _low_red_l(177, 50, 180);
cv::Scalar _high_red_l(180, 150, 255);
cv::Scalar _low_red_r(0, 50, 180);
cv::Scalar _high_red_r(3, 150, 255);


// futher params

// slope variance
double slope_var = 5;

// maximum angle variance
int _max_angle_var = 5;
int _max_dist_var = 20;

// camera resolution
int _image_width = 640;
int _image_height = 480;


// obj points
std::vector<cv::Point3f> obj_points;

/**
 * @brief generates random obj points to test the ply writer
 * 
 */
void randomObjPoints()
{
    for(int i = 0; i < 10; i++)
    {
        obj_points.push_back(cv::Point3f(i,i,i));
    }
}

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
int calcCenterLine(std::vector<cv::Vec2f>& in_lines, cv::Vec4i& out_line1, cv::Vec4i& out_line2)
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
        
        if(avg_1[0] == 0 && avg_2[0] == 0 ) // first line
        {
            avg_1[0] += rho;
            avg_1[1] += theta;
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
             // if the lines matches the first one
            if(cv::abs(avg_1[0] / counter_l  -  rho) <= _max_dist_var && cv::abs(avg_1[1] / counter_l  -  theta) <= _max_angle_var)
            {
                avg_1[0] += rho;
                avg_1[1] += theta;
                counter_l++;
            }
            else
            {
                avg_2[0] += rho;
                avg_2[1] += theta;
                counter_r++;
            }
        }
        else{
            
            // check if the line matches either the right or the left line
            if(cv::abs(avg_1[0] / counter_l  -  rho) <= _max_dist_var && cv::abs(avg_1[1] / counter_l  -  theta) <= _max_angle_var)
            {
                avg_1[0] += rho;
                avg_1[1] += theta;
                counter_l++;
            }
            else if (cv::abs(avg_2[0] / counter_r  -  rho) <= _max_dist_var && cv::abs(avg_2[1] / counter_r  -  theta) <= _max_angle_var)
            {
                avg_2[0] += rho;
                avg_2[1] += theta;
                counter_r++;
            }
            else continue; // skip line
        }
    }
    
    // we need two lines, so this scenario is a no go
    if(counter_l == 0 || counter_r == 0)
    {
        return 1;
    }

    avg_1 /= counter_l;
    avg_2 /= counter_r;
    
    float rho1 = avg_1[0], theta1 = avg_1[1], rho2 = avg_2[0], theta2 = avg_2[1];
    double a1 = cos(theta1), b1 = sin(theta1), a2 = cos(theta2), b2 = sin(theta2);
    double x0 = a1 * rho1, y0 = b1 * rho1, x1 = a2 * rho2, y1 = b2 * rho2;
    cv::Vec4i line_1(
        cvRound(x0 + 1000*(-b1)),
        cvRound(y0 + 1000*(a1)),
        cvRound(x0 - 1000*(-b1)),
        cvRound(y0 - 1000*(a1))
    );

    cv::Vec4i line_2(
        cvRound(x1 + 1000*(-b2)),
        cvRound(y1 + 1000*(a2)),
        cvRound(x1 - 1000*(-b2)),
        cvRound(y1 - 1000*(a2))
    );

    out_line1[0] = line_1[0];
    out_line1[1] = line_1[1];
    out_line1[2] = line_1[2];
    out_line1[3] = line_1[3];

    out_line2[0] = line_2[0];
    out_line2[1] = line_2[1];
    out_line2[2] = line_2[2];
    out_line2[3] = line_2[3];
    
    return 0;
}

/**
 * @brief calculates the intersection between the two lines and draws it onto the frame
 * 
 * @param line_1 
 * @param line_2 
 */
bool calcAndDisplayIntersection(cv::Vec4i line_1, cv::Vec4i line_2, cv::Mat& frame)
{
    cv::Point2f o1(line_1[0], line_1[1]);
    cv::Point2f o2(line_2[0], line_2[1]);

    cv::Point2f p1(line_1[2], line_1[3]);
    cv::Point2f p2(line_2[2], line_2[3]);

    cv::Point2f x = o2 - o1;
    cv::Point2f d1 = p1 - o1;
    cv::Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    cv::Point2f r = o1 + d1 * t1;

    cv::circle(frame, r, 4, cv::Scalar(255,0,255), 3);
}

/**
 * @brief identifies the object points in ze mask. very nice.
 * 
 * @param mask 
 * @param line_1 
 * @param line_2 
 */
void identifyObjectPoints(cv::Mat& mask, cv::Vec4i line_1, cv::Vec4i line_2)
{
    // for(int i = 0; i < mask.rows; i++)
    // {
    //     for(int j = 0; i < mask.cols; j++)
    //     {
    //         // check if current pixel is denoted as red, if so: check if the point belongs to the line or if it doesnt.
    //         if(mask.at<int>(i,j) > 200)
    //         {
    //             std::cout << mask.at<int>(i,j) << std::endl;
    //         }
    //     }
    // }

    // remove line points using this quick and easy hack, by drawing black lines on top, basically only leaving the obj points
    cv::line(mask, cv::Point(line_1[0],line_1[1]), cv::Point(line_1[2], line_1[3]), cv::Scalar(0,0,0), 30, cv::LINE_AA);
    cv::line(mask, cv::Point(line_2[0],line_2[1]), cv::Point(line_2[2], line_2[3]), cv::Scalar(0,0,0), 30, cv::LINE_AA);
}

/**
 * @brief filteres found lines
 * 
 * @param filteredLines one line for similar angles
 * @param lines 
 */
int filterLines(std::vector<cv::Vec4i>& filteredLines, std::vector<cv::Vec4i> lines) 
{
    std::vector<category> categories;

    //std::cout << "num lines incoming: " << lines.size() << std::endl;

    for (auto line : lines) 
    {
        //std::cout << "X1: " << line[0] << " X2: " << line[2] << std::endl;
        double angle;
        // calculate angle of line
        /*
        if (line[3] > line[1])
        {
            angle = 360 - (atan( ((line[3] - line[1]) / (line[2] - line[0])) ) * 180 / M_PI);    
        } else {
            angle = atan( ((line[1] - line[3]) / (line[2] - line[0])) ) * 180 / M_PI;
        }
        */
        angle = atan2(line[3] - line[1], line[2] - line[0]) * 180 / M_PI;

        std::cout << "angle: " << angle << std::endl;

        double length = cv::norm(cv::Point2i(line[0] - line[2], line[1] - line[3]));
        bool inserted = false;

        // put line into category based on angle
        for (auto& categ : categories)
        {
            if (categ.avg_slope_angle - slope_var < angle && categ.avg_slope_angle + slope_var > angle) {
                categ.avg_slope_angle = (categ.avg_slope_angle * categ.lines.size() + angle) / (categ.lines.size() + 1);
                
                categ.lines.push_back(line);
                inserted = true;
                
                if (length > categ.max_length) {
                    categ.max_length = length;
                    categ.longest_line = line;
                }
            }

            //std::cout << "line angle: " << categ.avg_slope_angle << std::endl;

        }
        
        // new category
        if (!inserted) {
            category new_cat;
            new_cat.avg_slope_angle = angle;
            new_cat.lines.push_back(line);
            new_cat.longest_line = line;
            new_cat.max_length = length;
            categories.push_back(new_cat);
        }        
    }

    filteredLines.clear();

    if (categories.size() < 2) {
        return -1;
    } else if (categories.size() > 2) {
        std::sort( categories.begin( ), categories.end( ), [ ]( const category& lhs, const category& rhs )
        {
            return lhs.max_length > rhs.max_length;
        });

        filteredLines.push_back(categories[0].longest_line);
        filteredLines.push_back(categories[1].longest_line);

        return 0;

    } else {
        filteredLines.push_back(categories[0].longest_line);
        filteredLines.push_back(categories[1].longest_line);
        return 0;
    }

}

/**
 * @brief finds lines alla
 * 
 */
void findLines()
{
    cv::VideoCapture camera(0);

    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return;
    }

    cv::Mat frame, canny, res, hsv, mask_l, mask_r, mask, tmp;
    cv::namedWindow("Webcam");
    cv::namedWindow("Mask");
    cv::namedWindow("Canny");
    cv::namedWindow("Hough");
    cv::namedWindow("ObjectPoints");
    std::vector<cv::Vec2f> lines;
    std::vector<cv::Vec4i> linesP;
    std::vector<cv::Vec4i> pix_lines;
    //std::vector<float> linesP_slopes;
    std::vector<cv::Vec4i> filteredLines;
    cv::Vec4i avg_line1;
    cv::Vec4i avg_line2;

    while(true)
    {
        camera >> tmp;
        //std::cout << "Cols: " << frame.cols << " Rows: " << frame.rows << std::endl;

        // undistort first
        cv::undistort(tmp, frame, _camera_mat, _dist_coeffs);

        // convert to hsv
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);


        cv::inRange(hsv, _low_red_l, _high_red_l, mask_l);
        cv::inRange(hsv, _low_red_r, _high_red_r, mask_r);

        cv::bitwise_or(mask_l, mask_r, mask);

        cv::imshow("Mask", mask);

        cv::Canny(mask, canny, 50, 200, 3); // use canny to detect edges alla

        cv::imshow("Canny", canny);

        //applying houghlines operation on the canny image
        //cv::HoughLines(mask, lines, 1, CV_PI / 180, 70, 0, 0);

        cv::HoughLinesP(mask, linesP, 1, CV_PI / 180, 80, 160, 90);
        
        //for each line, which exceeds the threshold, calc endpoints
        for( size_t i = 0; i < linesP.size(); i++ )
        {
            // cv::Point pt1, pt2;
            // float rho = lines[i][0], theta = lines[i][1];
            // double a = cos(theta), b = sin(theta);
            // double x0 = a*rho, y0 = b*rho;
            // pt1.x = cvRound(x0 + 1000*(-b));
            // pt1.y = cvRound(y0 + 1000*(a));
            // pt2.x = cvRound(x0 - 1000*(-b));
            // pt2.y = cvRound(y0 - 1000*(a));

            //std::cout << "X1 " << pt1.x << "Y1 " << pt1.y <<"X2 " << pt2.x <<"Y2 " << pt2.y << std::endl;
            
            cv::line(frame, cv::Point(linesP[i][0], linesP[i][1]), 
                     cv::Point(linesP[i][2], linesP[i][3]), cv::Scalar(0,255,0), 3, cv::LINE_AA);
            //cv::line(frame, pt1, pt2, cv::Scalar(0,255,0), 3, cv::LINE_AA);
            //cv::Vec4i vec(pt1.x, pt1.y, pt2.x, pt2.y);
            //pix_lines.push_back(vec);
        }

        // filter lines
        int status = filterLines(filteredLines, linesP);

        //int status = calcCenterLine(lines, avg_line1, avg_line2);
        
        // less than two lines returned
        if(status == -1)
        {
            cv::imshow("Webcam", frame);
            char key = (char)cv::waitKey(10);
        
            if (key == 27)
            {
                break;
            }

            continue;
        }

        avg_line1 = filteredLines[0];
        avg_line2 = filteredLines[1];

        cv::line(frame, cv::Point(avg_line1[0],avg_line1[1]), cv::Point(avg_line1[2], avg_line1[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
        cv::line(frame, cv::Point(avg_line2[0],avg_line2[1]), cv::Point(avg_line2[2], avg_line2[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
        
        calcAndDisplayIntersection(avg_line1, avg_line2, frame);

        identifyObjectPoints(mask, avg_line1, avg_line2);

        cv::imshow("ObjectPoints", mask);
        cv::imshow("Webcam", frame);
        
        char key = (char)cv::waitKey(10);
        
        if (key == 27)
        {
            break;
        }

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

    randomObjPoints();

    CultureInvariantPlyWriter writer("test.ply", obj_points);
    writer.Start();
}