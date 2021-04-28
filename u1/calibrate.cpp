#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"

enum MODE {
   FOLDERINPUT,
   WEBCAMINPUT 
};

//hardcodet for now.
int numCornersHor = 9;
int numCornersVer = 6;


void calibrateUsingWebcam()
{
    std::vector<cv::Point2f> point_buf;
    bool found;
    int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;

    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i < numCornersHor; i++)
    {
        for(int j{0}; j < numCornersVer; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }

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

    // display the frame until you press a key
    while (1) {
        // capture the next frame from the webcam
        camera >> frame;
        
        cv::cvtColor(frame, gray_img, cv::COLOR_RGB2GRAY);
        found = cv::findChessboardCorners(gray_img, cv::Size(numCornersHor, numCornersVer), point_buf, chessBoardFlags);
        if(found)
        {
            std::cout << "Found chessboard!" << std::endl;

            // criteria used for refinement
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            
            //refine corners
            cv::cornerSubPix(gray_img, point_buf, cv::Size(11,11), cv::Size(-1,-1),criteria);

            //draw them on the image
            cv::drawChessboardCorners(frame, cv::Size(numCornersHor, numCornersVer), point_buf, found);

            objpoints.push_back(objp);
            imgpoints.push_back(point_buf);
        }

        // show the image on the window, in color
        cv::imshow("Webcam", frame);

        // wait (10ms) for a key to be pressed
        if (cv::waitKey(300) >= 0)
        {
            break;
        }
    }

    // use found points for calibration
    cv::destroyAllWindows();

    cv::Mat cameraMatrix, distCoeffs, R, T;


    double reprojectionError = cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray_img.rows,gray_img.cols), cameraMatrix, distCoeffs, R, T);

    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
	std::cout << "distCoeffs : " << distCoeffs << std::endl;
	std::cout << "Rotation vector : " << R << std::endl;
	std::cout << "Translation vector : " << T << std::endl;
    std::cout << "Reprojection Error: " << reprojectionError << std::endl;

    cv::namedWindow("Calibrated");
    
    while(1)
    {
        camera >> frame;
        cv::undistort(frame, calib, cameraMatrix, distCoeffs);
        cv::flip(calib, calib, 1); //flip to make easier usability possible
        cv::imshow("Calibrated", calib);
        if (cv::waitKey(10) >= 0)
        {
            break;
        }
    }

    /* save camera params */
    cv::FileStorage fs("intrinsicParams.yaml", cv::FileStorage::WRITE);

    if (!R.empty())
        fs << "nr_of_framnes" << R.size();
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "rpe" << reprojectionError;

    
}

void calibrateUsingImages(std::string folder_path)
{
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i < numCornersHor; i++)
    {
        for(int j{0}; j < numCornersVer; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }

    std::cout << "Starting to calibrate using images" << std::endl;

    std::cout << "Path: " << folder_path << std::endl;
    std::vector<cv::String> images;

    try
    {
            cv::glob(folder_path, images);
    }
    catch (cv::Exception)
    {
        std::cout << "No images found in the specified location" << std::endl;
        return;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Images");
    cv::Mat frame, gray_img;
    std::vector<cv::Point2f> point_buf;
    bool found;
    int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;

    for(int i = 0; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame, gray_img, cv::COLOR_RGB2GRAY);
        found = cv::findChessboardCorners(gray_img, cv::Size(numCornersHor, numCornersVer), point_buf, chessBoardFlags);
        if(found)
        {
             std::cout << "Found chessboard!" << std::endl;

             // criteria used for refinement
             cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            
            //refine corners
             cv::cornerSubPix(gray_img, point_buf, cv::Size(11,11), cv::Size(-1,-1),criteria);

            //draw them on the image
             cv::drawChessboardCorners(frame, cv::Size(numCornersHor, numCornersVer), point_buf, found);

            objpoints.push_back(objp);
	        imgpoints.push_back(point_buf);
        }

        // show the image on the window, in color
        cv::imshow("Images", frame);
        cv::waitKey(2000);
    }

    cv::destroyAllWindows();

    cv::Mat cameraMatrix, distCoeffs, R, T;

    double reprojectionError = cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray_img.rows,gray_img.cols), cameraMatrix, distCoeffs, R, T);

    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
	std::cout << "distCoeffs : " << distCoeffs << std::endl;
	std::cout << "Rotation vector : " << R << std::endl;
	std::cout << "Translation vector : " << T << std::endl;
    std::cout << "Reprojection error: " << reprojectionError << std::endl;

        /* save camera params */
    cv::FileStorage fs("intrinsicParams.yaml", cv::FileStorage::WRITE);

    if (!R.empty())
        fs << "nr_of_framnes" << R.size();
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "rpe" << reprojectionError;
}


int main(int argc, char** argv) {
    
    std::string location_ = "";
    MODE mode_;

    // std:: cout << "Enter location of the files, type 'cam' to use the webcam instead." << std::endl;
    // std::getline(std::cin, location_);

    if(argc == 2)
        location_ = argv[1];

    // check, wether an empty string was put into the program, if so: use webcam feed
    if(location_ == "")
    {
        mode_ = WEBCAMINPUT;
        std::cout << "Using webcam as input!"  << std::endl;
    }
    else
    {
        mode_ = FOLDERINPUT;
        std::cout << "You entered the following location: " << location_  << std::endl;
    }

    if(mode_ == WEBCAMINPUT)
    {
        calibrateUsingWebcam();
    }
    else{
        calibrateUsingImages(location_);
    }
    

    return 0;
}