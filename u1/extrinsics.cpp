#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"

enum MODE {
   FOLDERINPUT,
   WEBCAMINPUT 
};

// definitions for the chessboard
int _num_corners_hor = 9;
int _num_corners_vert = 6;
float _hor_offset = 1.0f;
float _vert_offset = 2.2f;
float _size_of_square = 2.4f;

cv::Mat _camera_mat;
cv::Mat _dist_coeffs;

cv::Mat _R;
cv::Mat _T;


//points of the board in its world coordinates
std::vector<cv::Point3f> objp;

/**
 * @brief initializes a 3d representation of the chessboard corners
 * 
 */
void initializeWorldPoints()
{
    //define the world coords of my schachbrett alla -> using one schachbrett alla
    for(int i = 1 ; i <= _num_corners_vert; i++)
    {
        for(int j = 1 ; j <= _num_corners_hor; j++)
        {
            float coord_x = j * _size_of_square + _hor_offset;
            float coord_y = i * _size_of_square + _vert_offset;
            float coord_z = 0;
            objp.push_back(cv::Point3f(coord_x, coord_y, coord_z));
        }
    }
}

/**
 * @brief reads the intrisics, which have been written to a yaml file.
 * 
 * @return int 
 */
int initializeIntrinsicsFromYAML()
{
    //todo: read yaml stuff
    cv::FileStorage fs;
    fs.open("intrinsicParams.yaml", cv::FileStorage::READ);
    
    //check if it could be opened
    if(!fs.isOpened())
    {
        std::cerr << "no intrinsics available!" << std::endl;
        return 1;
    }

    fs["camera_matrix"] >> _camera_mat;
    fs["distortion_coefficients"] >> _dist_coeffs;

    std::cout << "Camera Matrix:" << std::endl << _camera_mat << std::endl;
    std::cout << "Dist Coeffs:" << std::endl << _dist_coeffs << std::endl;

    return 0;
}

/**
 * @brief actual calculation of the extrinsics.
 * 
 */
void calculateExtrinsics()
{
    // open the first webcam plugged in the computer
    cv::VideoCapture camera(0);

    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam");

    // this will contain the image from the webcam
    cv::Mat frame, gray_img, calib;
    std::vector<cv::Point2f> point_buf;
    bool found;
    int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
    
    // display the frame until you press a key
    while (1) {
        // capture the next frame from the webcam
        camera >> frame;
        
        cv::cvtColor(frame, gray_img, cv::COLOR_RGB2GRAY);
        found = cv::findChessboardCorners(gray_img, cv::Size(_num_corners_hor, _num_corners_vert), point_buf, chessBoardFlags);
        if(found)
        {
            std::cout << "Found chessboard!" << std::endl;

            // criteria used for refinement
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            
            //refine corners
            cv::cornerSubPix(gray_img, point_buf, cv::Size(11,11), cv::Size(-1,-1),criteria);

            //draw them on the image
            cv::drawChessboardCorners(frame, cv::Size(_num_corners_vert, _num_corners_hor), point_buf, found);
        }

        // show the image on the window, in color
        cv::imshow("Webcam", frame);

        // wait (10ms) for a key to be pressed
        if(found){
            cv::waitKey(3000);
            break;
        }
    }

    cv::solvePnP(objp, point_buf, _camera_mat, _dist_coeffs, _R, _T);
    cv::destroyAllWindows();
}

void showUndistortedImageLoop()
{
      // open the first webcam plugged in the computer
    cv::VideoCapture camera(0);

    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return;
    }
    // create a window to display the images from the webcam
    cv::namedWindow("Webcam");

    // this will contain the image from the webcam
    cv::Mat frame, tmp;
    bool show_undistorted = false;
    
    // display the frame until you press a key
    while (1) {
        // capture the next frame from the webcam
        camera >> frame;
        
        if(show_undistorted)
        {
            cv::undistort(frame, tmp, _camera_mat, _dist_coeffs);
            cv::imshow("Webcam", tmp);
        }
        // show the image on the window, in color
        else
        {
            cv::drawFrameAxes(frame, _camera_mat, _dist_coeffs, _R, _T, 10);
            cv::imshow("Webcam", frame);

        }

        char key = (char)cv::waitKey(10);
        if (key == 27)
        {
            break;
        }
        else if(key >= 0)
        {
            show_undistorted = !show_undistorted;
        }
    }
}

/**
 * @brief Prints the calculated extrinsics
 * 
 */
void printExtrinsics()
{
    std::cout << "Rotation: " << std::endl << _R << std::endl;
    std::cout << "Translation: " << std::endl << _T << std::endl;
}

/**
 * @brief Main method of the program, calls the necessary functions
 *          to calculate the extrinsics of a camera
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    initializeWorldPoints();
    int status = initializeIntrinsicsFromYAML();
    if(status == 1)
    {
        return EXIT_FAILURE;
    }
    calculateExtrinsics();
    printExtrinsics();
    showUndistortedImageLoop();

    return EXIT_SUCCESS;
}

