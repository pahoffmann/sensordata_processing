#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"
#include <numeric>

enum MODE {
   FOLDERINPUT,
   WEBCAMINPUT 
};

// definitions for the chessboard
int _num_corners_hor = 9;
int _num_corners_vert = 6;
float _size_of_square = 2.4f;

//left
float _hor_offset_l = 1.0f;
float _vert_offset_l = 2.2f;

//right
float _hor_offset_r = 1.2f;
float _vert_offset_r = 1.2f;




cv::Mat _camera_mat;
cv::Mat _dist_coeffs;

cv::Mat _R;
cv::Mat _T;


//points of the board in its world coordinates
std::vector<cv::Point3f> objp_l;
std::vector<cv::Point3f> objp_r;
std::vector<cv::Point3f> objp;

/**
 * @brief initializes a 3d representation of the chessboard corners
 * 
 */
void initializeWorldPoints()
{
    // define the world coords of my schachbrett alla -> using one schachbrett alla
    
    // first left chessboard (down right to up-left)
    for(int i = 1 ; i <= _num_corners_vert; i++)
    {
        for(int j = 1 ; j <= _num_corners_hor; j++)
        {
            float coord_x = j * _size_of_square + _hor_offset_l;
            float coord_y = i * _size_of_square + _vert_offset_l;
            float coord_z = 0;
            objp_l.push_back(cv::Point3f(coord_x, coord_y, coord_z));
        }
    }
    
    // then right (up left to down right)
    for(int i = _num_corners_vert ; i >= 1; i--)
    {
        for(int j = 1 ; j <= _num_corners_hor; j++)
        {
            float coord_x = 0;
            float coord_y = i * _size_of_square + _vert_offset_r;
            float coord_z = j * _size_of_square + _hor_offset_r;
            objp_r.push_back(cv::Point3f(coord_x, coord_y, coord_z));
        }
    }

    // push to array (dont do this later on)
    objp.reserve( objp_l.size() + objp_r.size() ); // preallocate memory
    objp.insert( objp.end(), objp_l.begin(), objp_l.end() ); 
    objp.insert( objp.end(), objp_r.begin(), objp_r.end() );
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
    cv::Mat frame, gray_img;
    std::vector<cv::Point2f> point_buf, point_buf1, point_buf2;
    bool found;
    int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
    int found_counter = 0; // used to cound how many have been found
    
    camera >> frame;

    // find the (two) chessboards inside the image
    while (found_counter != 2) {
        
        cv::cvtColor(frame, gray_img, cv::COLOR_RGB2GRAY);
        if(found_counter == 0)
        {
            found = cv::findChessboardCorners(gray_img, cv::Size(_num_corners_hor, _num_corners_vert), point_buf1, chessBoardFlags);
        }
        else
        {
            found = cv::findChessboardCorners(gray_img, cv::Size(_num_corners_hor, _num_corners_vert), point_buf2, chessBoardFlags);
        }
        
        if(found)
        {

            std::cout << "Found chessboard!" << std::endl;

            // criteria used for refinement
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            
            //refine corners
            if(found_counter == 0)
            {
                cv::cornerSubPix(gray_img, point_buf1, cv::Size(11,11), cv::Size(-1,-1),criteria);

            }
            else
            {
                cv::cornerSubPix(gray_img, point_buf2, cv::Size(11,11), cv::Size(-1,-1),criteria);
            }
            

            //draw them on the image
            if(found_counter == 0)
            {
                cv::drawChessboardCorners(frame, cv::Size(_num_corners_vert, _num_corners_hor), point_buf1, found);
            }
            else
            {
                cv::drawChessboardCorners(frame, cv::Size(_num_corners_vert, _num_corners_hor), point_buf2, found);
            }
        }

        
        // show the image on the window, in color
        cv::imshow("Webcam", frame);

        // wait (10ms) for a key to be pressed
        if(found){
            found_counter++;
            cv::waitKey(3000);
        }
    }

    //check which points belong left and which right
    cv::Point2f sum1 = std::accumulate(
        point_buf1.begin(), point_buf1.end(), // Run from begin to end
        cv::Point2f(0.0f,0.0f),       // Initialize with a zero point
        std::plus<cv::Point2f>()      // Use addition for each point (default)
    );
    cv::Point2f mean1 = sum1 / (int)point_buf1.size(); // Divide by count to get mean

    cv::Point2f sum2 = std::accumulate(
        point_buf2.begin(), point_buf2.end(), // Run from begin to end
        cv::Point2f(0.0f,0.0f),       // Initialize with a zero point
        std::plus<cv::Point2f>()      // Use addition for each point (default)
    );
    cv::Point2f mean2 = sum2 / (int)point_buf2.size(); // Divide by count to get mean

    std::cout << "Mean1: " << mean1 << "| Mean 2: " << mean2 << std::endl;
    //reserve mem
    point_buf.reserve(point_buf1.size() + point_buf2.size());

    //update point buffer
    if(mean1.x > mean2.x) 
    {
        point_buf.insert(point_buf.end(), point_buf2.begin(), point_buf2.end());
        point_buf.insert(point_buf.end(), point_buf1.begin(), point_buf1.end());
    }
    else
    {
        point_buf.insert(point_buf.end(), point_buf1.begin(), point_buf1.end());
        point_buf.insert(point_buf.end(), point_buf2.begin(), point_buf2.end());
    }

    cv::solvePnP(objp, point_buf, _camera_mat, _dist_coeffs, _R, _T);
    cv::destroyAllWindows();
}

/**
 * @brief LOOP, which shows the coordinate system, as well as undistorted images
 * 
 */
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

