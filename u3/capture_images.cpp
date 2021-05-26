#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;

int main(int argc, char** argv)
{

    cv::VideoCapture camera1(0);

    cv::VideoCapture camera2(2);

    cv::namedWindow("image1");
    cv::namedWindow("image2");

    cv::Mat image1, image2, gray_1, gray_2, tmp_1, tmp_2;
    bool found1, found2;

    std::vector<cv::Point2f> point_buf;

    int counter = 1;

    while(true)
    {
        camera1 >> image1;
        camera2 >> image2;

        image1.copyTo(tmp_1);
        image2.copyTo(tmp_2);


        cv::cvtColor(image1, gray_1, cv::COLOR_RGB2GRAY);
        cv::cvtColor(image2, gray_2, cv::COLOR_RGB2GRAY);
        
        found1 = cv::findChessboardCorners(gray_1, cv::Size(9, 6), point_buf, chessBoardFlags);

        cv::drawChessboardCorners(image1, cv::Size(9,6), point_buf, true);


        found2 = cv::findChessboardCorners(gray_2, cv::Size(9, 6), point_buf, chessBoardFlags);

        cv::drawChessboardCorners(image2, cv::Size(9,6), point_buf, true);
        // find chessboard corners
        // if found: save them under specific name.

        cv::imshow("image1", image1);
        cv::imshow("image2", image2);

        int key = cv::waitKey(50);
        if(key == 27 && found1 && found2)
        {
            std::cout << "Writing found images" << std::endl;

            std::string num = counter < 10 ? "0" + std::to_string(counter) : std::to_string(counter);

            //std::string left_img = "../../u3/data/left" + num + ".jpg";
            //std::string right_img = "../../u3/data/right" + num + ".jpg";

            std::string left_img = "left" + num + ".jpg";
            std::string right_img = "right" + num + ".jpg";

            std::cout << left_img << " | " << right_img << std::endl;

            bool written1 = cv::imwrite(left_img, tmp_1);
            bool written2 = cv::imwrite(right_img, tmp_2);

            if(!written1 || !written2)
            {
                std::cout << "i bims, 1 fehler" << std::endl;
            }

            counter++;
        }
        else if(key == 32)
        {
            break;
        }


    }

}