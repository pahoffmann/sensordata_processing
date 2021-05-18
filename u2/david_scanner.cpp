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
cv::Mat _world_to_camera;
cv::Mat _camera_to_world;
cv::Mat _R_rodrigues;

// slope line struct stuff
struct category {
    double avg_slope_angle;
    double max_length;
    cv::Vec4i longest_line;
    std::vector<cv::Vec4i> lines;
};

// used to represent the 3D surface enclosed by the two laser lines
struct surface {
    cv::Mat_<double> surface_point; // stützvec
    cv::Mat_<double> normal;
};

//lines
cv::Vec4i _left;
cv::Vec4i _right;

// red spectrum

cv::Scalar _low_red_l(177, 50, 180);
cv::Scalar _high_red_l(180, 150, 255);
cv::Scalar _low_red_r(0, 50, 180);
cv::Scalar _high_red_r(3, 150, 255);

// cv::Scalar _low_red_l(175, 50, 120);
// cv::Scalar _high_red_l(180, 200, 255);
// cv::Scalar _low_red_r(0, 50, 120);
// cv::Scalar _high_red_r(5, 200, 255);

// futher params

// slope variance
double slope_var = 10;

// maximum angle variance
int _max_angle_var = 5;
int _max_dist_var = 20;

// camera resolution
int _image_width = 640;
int _image_height = 480;


// obj points
std::vector<cv::Point3d> obj_points;

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

    // convert rotation vector to rotation matrix
    cv::Rodrigues(_R, _R_rodrigues);
    cv::Mat_<double> _transform_mat(4,4);
    //std::cout << _R_rodrigues << std::endl;

    _transform_mat.at<double>(0, 0) = _R_rodrigues.at<double>(0, 0);
    _transform_mat.at<double>(0, 1) = _R_rodrigues.at<double>(0, 1);
    _transform_mat.at<double>(0, 2) = _R_rodrigues.at<double>(0, 2);

    _transform_mat.at<double>(1, 0) = _R_rodrigues.at<double>(1, 0);
    _transform_mat.at<double>(1, 1) = _R_rodrigues.at<double>(1, 1);
    _transform_mat.at<double>(1, 2) = _R_rodrigues.at<double>(1, 2);

    _transform_mat.at<double>(2, 0) = _R_rodrigues.at<double>(2, 0);
    _transform_mat.at<double>(2, 1) = _R_rodrigues.at<double>(2, 1);
    _transform_mat.at<double>(2, 2) = _R_rodrigues.at<double>(2, 2);

    _transform_mat.at<double>(0, 3) = _T.at<double>(0, 0);
    _transform_mat.at<double>(1, 3) = _T.at<double>(1, 0);
    _transform_mat.at<double>(2, 3) = _T.at<double>(2, 0);

    _transform_mat.at<double>(3, 0) = 0;
    _transform_mat.at<double>(3, 1) = 0;
    _transform_mat.at<double>(3, 2) = 0;
    _transform_mat.at<double>(3, 3) = 1;

    _world_to_camera = _transform_mat;
    _camera_to_world = _world_to_camera.inv();

    //std::cout << "Camera Matrix:" << std::endl << _camera_mat << std::endl;
    //std::cout << "Dist Coeffs:" << std::endl << _dist_coeffs << std::endl;
    // std::cout << "Rotation mat:" << std::endl << _R << std::endl;
    // std::cout << "Rotation mat rod:" << std::endl << _R_rodrigues << std::endl;
    // std::cout << "Translation mat:" << std::endl << _T << std::endl;
    // std::cout << "Transform mat:" << std::endl << _world_to_camera << std::endl;

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
    cv::line(mask, cv::Point(line_1[0],line_1[1]), cv::Point(line_1[2], line_1[3]), cv::Scalar(0,0,0), 50, cv::LINE_AA);
    cv::line(mask, cv::Point(line_2[0],line_2[1]), cv::Point(line_2[2], line_2[3]), cv::Scalar(0,0,0), 50, cv::LINE_AA);
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

        //std::cout << "angle: " << angle << std::endl;

        double length = cv::norm(cv::Point2i(line[0] - line[2], line[1] - line[3]));
        bool inserted = false;

        // put line into category based on angle
        for (auto& categ : categories)
        {
            if ((categ.avg_slope_angle - slope_var < angle) && (categ.avg_slope_angle + slope_var > angle)) {
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

        /*
        std::sort( categories.begin( ), categories.end( ), [ ]( const category& lhs, const category& rhs )
        {
            return lhs.max_length > rhs.max_length;
        });

        // DEBUG
        // std::cout << "angle 1: " << categories[0].avg_slope_angle << std::endl;
        // std::cout << "angle 2: " << categories[1].avg_slope_angle << std::endl;

        // order by angle to determine left hand side or right hand side
        if (categories[0].avg_slope_angle < categories[1].avg_slope_angle) {
            filteredLines.push_back(categories[0].longest_line);
            filteredLines.push_back(categories[1].longest_line);
        } else {
            filteredLines.push_back(categories[1].longest_line);
            filteredLines.push_back(categories[0].longest_line);
        }

        return 0;
        */

       return -1;

    } else {
        // DEBUG
        // std::cout << "angle 1: " << categories[0].avg_slope_angle << std::endl;
        // std::cout << "angle 2: " << categories[1].avg_slope_angle << std::endl;

        // filter out horizontal lines, as they may be invalid
        if(categories[0].avg_slope_angle - 5 < 0 && categories[0].avg_slope_angle + 5 > 0
            || categories[1].avg_slope_angle - 5 < 0 && categories[1].avg_slope_angle + 5 > 0) 
        {
            return -1;
        }

        // order by angle to determine left hand side or right hand side
        if (categories[0].avg_slope_angle < categories[1].avg_slope_angle) {
            filteredLines.push_back(categories[0].longest_line);
            filteredLines.push_back(categories[1].longest_line);
        } else {
            filteredLines.push_back(categories[1].longest_line);
            filteredLines.push_back(categories[0].longest_line);
        }

        return 0;
    }

}

/**
 * @brief displays the laser surface on the 2d plane
 * 
 * @param frame 
 * @param _surface 
 */
void displayLaserSurface(cv::Mat& frame, surface _surface, cv::Mat_<double> vec_1, cv::Mat_ <double> vec_2)
{
    int num_x = 100;
    int num_y = 100;

    double norm_1 = cv::norm(vec_1);
    double norm_2 = cv::norm(vec_2);

    cv::Mat_<double> point(3,1);

    for(int i = 0; i < num_x; i++)
    {
        for(int j = 0; j < num_y; j++)
        {
            point = _surface.surface_point + i * ( vec_1 / norm_1 ) + j * (vec_2 / norm_2);
            point = _camera_mat * point;
            point /= point(2,0);
            cv::drawMarker(frame, cv::Point(point(0,0), point(1,0)), cv::Scalar(0,0,255));
        }
    }
}

/**
 * @brief calculate the surface enclosed by the laser lines
 * 
 * @param one one dir vec
 * @param two second dir vec
 * @param intersection calculated intersection point
 * @return surface 
 */
surface calculateLaserSurface(cv::Vec4i one, cv::Vec4i two, cv::Mat& frame)
{
    cv::Mat_<double> rA(3,1),rB(3,1),rC(3,1),rD(3,1); // rays to the outer points of the detected laser lines.

    cv::Mat_<double> A_pix(3,1),B_pix(3,1),C_pix(3,1),D_pix(3,1); // points in pixel coords
    cv::Mat_<double> A(3,1),B(3,1),C(3,1),D(3,1); // points in pixel coords
    cv::Mat_<double> worldBaseVec(3,1);
    
    cv::Mat_<double> ncL(3,1), ncR(3,1); // normal vector for the left and right plane

    cv::Mat_<double> surface_normal(3,1);


    if(one[0] < two[0]) // line 'one' is the left most line
    {
        A_pix(0,0) = one[2];
        A_pix(1,0) = one[3]; // center point of left line
        A_pix(2,0) = 1.0f;
        B_pix(0,0) = one[0];
        B_pix(1,0) = one[1];
        B_pix(2,0) = 1.0f;
        C_pix(0,0) = two[0];
        C_pix(1,0) = two[1];
        C_pix(2,0) = 1.0f;
        D_pix(0,0) = two[2];
        D_pix(1,0) = two[3];
        D_pix(2,0) = 1.0f;
    }
    else // line 'two' is the left most line
    {
        A_pix(0,0) = two[2];
        A_pix(1,0) = two[3]; // center point of left line
        A_pix(2,0) = 1.f;
        B_pix(0,0) = two[0];
        B_pix(1,0) = two[1];
        B_pix(2,0) = 1.f;
        C_pix(0,0) = one[0];
        C_pix(1,0) = one[1];
        C_pix(2,0) = 1.f;
        D_pix(0,0) = one[2];
        D_pix(1,0) = one[3];
        D_pix(2,0) = 1.f;
    }

    //std::cout << "edge points: " << std::endl;
    //std::cout << A_pix << B_pix << C_pix << D_pix << std::endl;

    // define surfaces
    worldBaseVec = _T;


    ncL.at<double>(0,0) = 0;
    ncL.at<double>(1,0) = 0;
    ncL.at<double>(2,0) = 1;

    //std::cout << _R_rodrigues << ncL << std::endl;
    
    ncR.at<double>(0,0) = 1;
    ncR.at<double>(1,0) = 0;
    ncR.at<double>(2,0) = 0;


    ncL = _R_rodrigues * ncL;
    ncR = _R_rodrigues * ncR;


    // invert camera Mat
    cv::Mat _camera_mat_inv = _camera_mat.inv();


    //calc rays
    rA = _camera_mat_inv * A_pix;
    rB = _camera_mat_inv * B_pix;
    rC = _camera_mat_inv * C_pix;
    rD = _camera_mat_inv * D_pix;


    //std::cout << rA << rB << rC << rD << std::endl;

    A = ((worldBaseVec.dot(ncL)) / (rA.dot(ncL))) * rA;
    B = ((worldBaseVec.dot(ncL)) / (rB.dot(ncL))) * rB;
    C = ((worldBaseVec.dot(ncR)) / (rC.dot(ncR))) * rC;
    D = ((worldBaseVec.dot(ncR)) / (rD.dot(ncR))) * rD;

    //std::cout << "points (cam coords): " << std::endl;
    //std::cout << A << B << C << D << std::endl;

    //std::cout << "points (WORLD coords): " << std::endl;
    cv::Mat_<double> linePointWorld1(4,1);
    cv::Mat_<double> linePointWorld2(4,1);
    cv::Mat_<double> linePointWorld3(4,1);
    cv::Mat_<double> linePointWorld4(4,1);
    linePointWorld1(0,0) = A(0,0);
    linePointWorld1(1,0) = A(1,0);
    linePointWorld1(2,0) = A(2,0);
    linePointWorld1(3,0) = 1.f;

    linePointWorld2(0,0) = C(0,0);
    linePointWorld2(1,0) = C(1,0);
    linePointWorld2(2,0) = C(2,0);
    linePointWorld2(3,0) = 1.f;

    linePointWorld3(0,0) = B(0,0);
    linePointWorld3(1,0) = B(1,0);
    linePointWorld3(2,0) = B(2,0);
    linePointWorld3(3,0) = 1.f;

    linePointWorld4(0,0) = D(0,0);
    linePointWorld4(1,0) = D(1,0);
    linePointWorld4(2,0) = D(2,0);
    linePointWorld4(3,0) = 1.f;

    //std::cout << _camera_to_world * linePointWorld1 << std::endl;
    //std::cout << _camera_to_world * linePointWorld2 << std::endl;
    //std::cout << _camera_to_world * linePointWorld3 << std::endl;
    //std::cout << _camera_to_world * linePointWorld4 << std::endl;
    
    // ebenen_norm = norm(B-A x D-C)
    surface_normal = (B - A).cross(D - C); // calc surface normal
    surface_normal /= cv::norm(surface_normal); // normalize the normal

    surface _laserSurface;
    _laserSurface.surface_point = A; // define stützvec
    _laserSurface.normal = surface_normal;

    //std::cout << "normal: " << std::endl;
    //std::cout << surface_normal << std::endl;

    displayLaserSurface(frame, _laserSurface, B - A, D - C);
    
    return _laserSurface;
}

/**
 * @brief calculates world coordinates of object points
 * 
 * @param frame 
 */
void calculateObjectPoints(cv::Mat frame, surface _surface) 
{
    cv::Mat _camera_mat_inv = _camera_mat.inv();
    cv::Mat_<double> obj_point_ray(3,1);
    cv::Mat_<double> obj_point_cam(3,1);
    cv::Mat_<double> obj_point_pix(3,1);
    cv::Mat_<double> obj_point_world(4,1);
    cv::Mat_<double> obj_point_world_3d(3,1);
    cv::Mat_<double> R_rodrigues_inv(3,3);
    int counter = 0;

    R_rodrigues_inv = _R_rodrigues.inv();

    //std::cout << "Num Points before: " << obj_points.size() << std::endl;

    for(int i = 0; i < frame.rows; i++)
    {
        for(int j = 0; j < frame.cols; j++)
        {
            if(frame.at<int>(i,j) > 0)
            {
                counter ++;
                // don't swap i and j here
                obj_point_pix(0,0) = j; // col for x value
                obj_point_pix(1,0) = i; // row for y
                obj_point_pix(2,0) = 1.f;
                
                obj_point_ray = _camera_mat_inv * obj_point_pix; //ray calculation
                
                // camera coordinate obj point
                obj_point_cam = ((_surface.surface_point.dot(_surface.normal)) 
                                / (obj_point_ray.dot(_surface.normal))) * obj_point_ray;

                obj_point_world(0,0) = obj_point_cam(0,0);
                obj_point_world(1,0) = obj_point_cam(1,0);
                obj_point_world(2,0) = obj_point_cam(2,0);
                obj_point_world(3,0) = 1.f;

                //transform to world coords
                //obj_point_world = _camera_to_world * obj_point_world;
                //obj_point_world /= obj_point_world(3, 0); // divide through homogen coordinate
                obj_point_world_3d = (R_rodrigues_inv * obj_point_cam) - (R_rodrigues_inv * _T);

                //std::cout << "World Point: " << obj_point_world << std::endl;

                // create vector and push to object point array                
                // cv::Point3d tmp(obj_point_world.at<double>(0,0), obj_point_world.at<double>(1,0),obj_point_world.at<double>(2,0));
                cv::Point3d tmp(obj_point_world_3d(0,0), obj_point_world_3d(1,0), obj_point_world_3d(2,0));

                // only save valid object points
                if ((tmp.x <= 100) && (tmp.y <= 100) && (tmp.z <= 100)
                    && (tmp.x >= -8) && (tmp.y >= -8) && (tmp.z >= -8))
                {
                    obj_points.push_back(tmp);
                }
                


                //std::cout << tmp << std::endl;
            }
        }
    }
    //std::cout << "NUMBER OF AWESOME POINTS: " << counter << std::endl;

    //std::cout << "Num Points after: " << obj_points.size() << std::endl;

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
    //cv::namedWindow("Canny");
    cv::namedWindow("Hough");
    cv::namedWindow("ObjectPoints");
    std::vector<cv::Vec2f> lines;
    std::vector<cv::Vec4i> linesP;
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

        // cv::Canny(mask, canny, 50, 200, 3); // use canny to detect edges alla
        //cv::Canny(mask, canny, 50, 200, 3); // use canny to detect edges alla

        //cv::imshow("Canny", canny);

        // cv::HoughLinesP(canny, linesP, 1, CV_PI / 180, 80, 160, 90);
        cv::HoughLinesP(mask, linesP, 1, CV_PI / 180, 80, 160, 120);
        
        
        //for each line, which exceeds the threshold, calc endpoints
        /* for( size_t i = 0; i < linesP.size(); i++ )
        {
            cv::line(frame, cv::Point(linesP[i][0], linesP[i][1]), 
                     cv::Point(linesP[i][2], linesP[i][3]), cv::Scalar(0,255,0), 3, cv::LINE_AA);
        } */

        // filter lines
        int status = filterLines(filteredLines, linesP);


        
        // less than two lines returned
        if(status == -1)
        {
            cv::imshow("Mask", mask);
            cv::imshow("Webcam", frame);
            char key = (char)cv::waitKey(10);
        
            if (key == 27)
            {
                break;
            }
        }
        else
        {
            avg_line1 = filteredLines[0];
            avg_line2 = filteredLines[1];

            cv::line(frame, cv::Point(avg_line1[0],avg_line1[1]), cv::Point(avg_line1[2], avg_line1[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
            cv::line(frame, cv::Point(avg_line2[0],avg_line2[1]), cv::Point(avg_line2[2], avg_line2[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
            
            calcAndDisplayIntersection(avg_line1, avg_line2, frame);

            cv::Mat eroded, closed;
            //cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
            //cv::erode(closed, eroded, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
            identifyObjectPoints(mask, avg_line1, avg_line2);

            surface _surface = calculateLaserSurface(avg_line1, avg_line2, frame);

            // debug: make laser surface visible...
            cv::Mat_<double> point_2d_1 = _camera_mat * _surface.surface_point;
            cv::Mat_<double> point_2d_2 = _camera_mat * (_surface.surface_point + 40 * _surface.normal);
            point_2d_1 /= point_2d_1(0,2);
            point_2d_2 /= point_2d_2(0,2);

            // draw laser surface normal
            cv::arrowedLine(frame, cv::Point(point_2d_1(0,0), point_2d_1(1,0)), cv::Point(point_2d_2(0,0), point_2d_2(1,0)), cv::Scalar(100,255,50));

            cv::imshow("ObjectPoints", mask);
            cv::imshow("Webcam", frame);

            calculateObjectPoints(mask, _surface);
            
            char key = (char)cv::waitKey(10);
            
            if (key == 27)
            {
                break;
            }
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

    //randomObjPoints();

    CultureInvariantPlyWriter writer("test.ply", obj_points);
    writer.Start();
}