#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv )
{

    std::vector<cv::Mat> cornerVector;
    std::vector<std::vector<cv::Vec3f>> objectPointsVector;
    cv::Size boardSize = cv::Size(9, 6);
    cv::Mat cameraMatrix, distCoeffs;
    cv::_OutputArray rvecs, tvecs;
    cv::_OutputArray stdDeviationIntrinsics, stdDeviationExtrinsics, perViewErrors;
    cv::Mat img;

    /* object points don't change, if object plane is planar (floor is made out of floor) 
     * and does not change. 
     */

    std::vector<cv::Vec3f> objectPoints;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 9; j++) {
            // box size is 23mm
            objectPoints.push_back(cv::Vec3f(static_cast<float>(i) * 23, static_cast<float>(j) * 23, 0));
        }
    }

    /*
    for(auto & elem : objectPoints)
    {
        std::cout<<elem<<", ";
    }
    std::cout << std::endl;
    */

    if ( argc == 1 )
    {
        /* grab images from webcam feed */
        std::cout << "TODO: grab images from webcam feed" << std::endl;

        cv::Mat frame;
        cv::VideoCapture cap;
        cap.open(0);
        if ( !cap.isOpened() ) {
            std::cerr << "ERROR! Unable to open camera!" << std::endl;
            return -1;
        }

        std::cout << "Start grabbing" << std::endl
            << "Press any key to terminate" << std::endl;

        for (;;) {
            cap.read(frame);
            if ( frame.empty() ) {
                std::cerr << "ERROR! blank frame grabbed" << std::endl;
                break;
            }

            /* convert frame to greyscale */
            cv::Mat frame_grscl;
            cv::cvtColor(frame, frame_grscl, cv::COLOR_BGR2GRAY);

            cv::Mat corners;
            bool found = cv::findChessboardCorners( frame, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH );

            if (found) {
                cv::cornerSubPix(frame_grscl, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
                cv::drawChessboardCorners(frame, boardSize, cv::Mat(corners), found);

                /* save the corner coordinates */
                cornerVector.push_back(corners);    
                objectPointsVector.push_back(objectPoints);

                std::cout << "found chessboard corners." << std::endl;

            }

            frame.copyTo(img);

            /* display found chessboard corners */
            cv::imshow("Live", frame);
            if ( cv::waitKey(5) >= 0 )
                break;

            sleep(2);
        }
    } else {

        /* process images specified as arguments */

        for (int i = 1; i < argc; i++) {

            cv::Mat image;
            cv::Mat corners;
            image = cv::imread( argv[i]);

            if ( !image.data )
            {
                printf("No image data \n");
                return -1;
            }

            cv::Mat image_grscl;
            cv::cvtColor(image, image_grscl, cv::COLOR_BGR2GRAY);

            //printf( "width: %u, height: %u\n", boardSize.width, boardSize.height );
            bool found = cv::findChessboardCorners( image, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH );
            if (found) {
                cv::cornerSubPix(image_grscl, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
                cv::drawChessboardCorners(image, boardSize, cv::Mat(corners), found);

                /* save the corner coordinates */
                cornerVector.push_back(corners);
                objectPointsVector.push_back(objectPoints);

                /* save the image */
                std::string saveFilename = std::string(argv[i]) + "_chessboard";
                bool writeSuccess = cv::imwrite(saveFilename, image);
                if (writeSuccess) {
                    std::cout << "image saved here: " << saveFilename << std::endl;
                } else {
                    std::cerr << "ERROR: Could not save image" << std::endl;
                }
            }
        }
    }


    /* calibrate the camera with the given chessboard points to get intrinsics */
    double reProjectionError = cv::calibrateCamera(objectPointsVector, cornerVector, boardSize, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationIntrinsics, 
                                                        stdDeviationExtrinsics, perViewErrors, 0, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, DBL_EPSILON));

    std::cout << "\nrpe: " << reProjectionError << std::endl;
    std::cout << "Camera Matrix: " << cameraMatrix << std::endl;
    std::cout << "distCoeffs: " << distCoeffs << std::endl;

    cv::Mat undistortedImage;
    //cv::Mat img = cv::imread("res/IMG_20210419_170611.jpg");

    cv::undistort(img, undistortedImage, cameraMatrix, distCoeffs);

    bool writeSuccess = cv::imwrite("res/undistortedImage.jpg", undistortedImage);
    if (writeSuccess) {
        std::cout << "image saved here: " << "res/undistortedImage" << std::endl;
    } else {
        std::cerr << "ERROR: Could not save image" << std::endl;
    }


    /* save camera params */
    cv::FileStorage fs("intrinsicParams.yaml", cv::FileStorage::WRITE);

    if (!rvecs.empty())
        fs << "nr_of_framnes" << rvecs.size();
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "rpe" << reProjectionError;

    return 0;
}