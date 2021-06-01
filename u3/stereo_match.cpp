/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <sstream>
#include <iostream>

using namespace cv;
using namespace cv::ximgproc;

static void print_help(char** argv)
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds. "
            "Reading images from webcam. Press \"s\" to save point cloud \n");
    printf("\nUsage: %s <left_image> <right_image> [--algorithm=bm|sgbm|hh|hh4|sgbm3way] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i=<intrinsic_filename>] [-e=<extrinsic_filename>]\n"
           "[--no-display] [--color] [-o=<disparity_image>] [-p=<point_cloud_file>]\n"
           "[-sigma=<sigma>] [--loop]", argv[0]);
}

static void saveXYZ(const char* filename, const Mat& mat, const Mat& img)
{
    const double max_z = 1.0e2;
    const double max_y = 1.0e2;
    const double max_x = 1.0e2;

    fpos_t pos;
    int pointCount = 0;
    char * line = NULL;
    size_t len = 0;
    ssize_t read;

    FILE* fp = fopen("temp", "wt");
    fgetpos(fp, &pos);
    FILE* fply = fopen(filename, "wt");

    // ply header
    

    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            Vec3b color = img.at<Vec3b>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            if(point[0] > max_x || point[1] > max_y || point[2] > max_z) continue;
            fprintf(fp, "%f %f %f ", point[0], point[1], point[2]);
            // color is bgr, not rgb
            fprintf(fp, "%u %u %u\n", color[2], color[1], color[0]);
            pointCount++;
        }
    }

    fclose(fp);

    printf("temp file written");

    FILE* fpread = fopen("temp", "rt");

    fprintf(fply, "ply\n");
    fprintf(fply, "format ascii 1.0\n");
    fprintf(fply, "element vertex ");
    fprintf(fply, "%s\n", std::to_string(pointCount).c_str());
    fprintf(fply, "property float x\n");
    fprintf(fply, "property float y\n");
    fprintf(fply, "property float z\n");
    fprintf(fply, "property uchar red\n");
    fprintf(fply, "property uchar green\n");
    fprintf(fply, "property uchar blue\n");
    fprintf(fply, "end_header\n");
   
    while ((read = getline(&line, &len, fpread)) != -1) {
        fprintf(fply, "%s", line);
    }
    
    fclose(fpread);
    fclose(fply);
}

int main(int argc, char** argv)
{
    std::string img1_filename = "";
    std::string img2_filename = "";
    std::string intrinsic_filename = "";
    std::string extrinsic_filename = "";
    std::string disparity_filename = "";
    std::string point_cloud_filename = "";

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4, STEREO_HH4=5 };
    int alg = STEREO_SGBM;
    int SADWindowSize, numberOfDisparities;
    bool no_display;
    bool color_display;
    bool loop;
    float scale;

    Ptr<StereoBM> bm = StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    Ptr<DisparityWLSFilter> wls_filter;


    cv::VideoCapture camera1(0);

    cv::VideoCapture camera2(2);


    cv::CommandLineParser parser(argc, argv,
        "{@arg1||}{@arg2||}{help h||}{algorithm||}{max-disparity|0|}{blocksize|0|}{no-display||}{color||}{scale|1|}{i||}{e||}{o||}{p||}{sigma||}{loop||}");
    if(parser.has("help"))
    {
        print_help(argv);
        return 0;
    }
    img1_filename = samples::findFile(parser.get<std::string>(0));
    img2_filename = samples::findFile(parser.get<std::string>(1));
    if (parser.has("algorithm"))
    {
        std::string _alg = parser.get<std::string>("algorithm");
        alg = _alg == "bm" ? STEREO_BM :
            _alg == "sgbm" ? STEREO_SGBM :
            _alg == "hh" ? STEREO_HH :
            _alg == "var" ? STEREO_VAR :
            _alg == "hh4" ? STEREO_HH4 :
            _alg == "sgbm3way" ? STEREO_3WAY : -1;
    }
    numberOfDisparities = parser.get<int>("max-disparity");
    SADWindowSize = parser.get<int>("blocksize");
    scale = parser.get<float>("scale");
    no_display = parser.has("no-display");
    loop = parser.has("loop");
    color_display = parser.has("color");
    if( parser.has("i") )
        intrinsic_filename = parser.get<std::string>("i");
    if( parser.has("e") )
        extrinsic_filename = parser.get<std::string>("e");
    if( parser.has("o") )
        disparity_filename = parser.get<std::string>("o");
    if( parser.has("p") )
        point_cloud_filename = parser.get<std::string>("p");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    if( alg < 0 )
    {
        printf("Command-line parameter error: Unknown stereo algorithm\n\n");
        print_help(argv);
        return -1;
    }
    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
    {
        printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        print_help(argv);
        return -1;
    }
    if (scale < 0)
    {
        printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
        return -1;
    }
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }
    if( img1_filename.empty() || img2_filename.empty() )
    {
        printf("Command-line parameter error: both left and right images must be specified\n");
        return -1;
    }
    if( (!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if( extrinsic_filename.empty() && !point_cloud_filename.empty() )
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

    Mat img1, img2;
    int filenameCount = 0;

    while(true)
    {
        camera1 >> img1;
        camera2 >> img2;

        int color_mode = alg == STEREO_BM ? 0 : -1;
        //Mat img1 = imread(img1_filename, color_mode);
        //Mat img2 = imread(img2_filename, color_mode);
        Mat left_orig;
        img1.copyTo(left_orig);

        if (img1.empty())
        {
            printf("Command-line parameter error: could not load the first input image file\n");
            return -1;
        }
        if (img2.empty())
        {
            printf("Command-line parameter error: could not load the second input image file\n");
            return -1;
        }

        if (scale != 1.f)
        {
            Mat temp1, temp2;
            int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
            resize(img1, temp1, Size(), scale, scale, method);
            img1 = temp1;
            resize(img2, temp2, Size(), scale, scale, method);
            img2 = temp2;
        }

        Size img_size = img1.size();

        Rect roi1, roi2;
        Mat Q;
        

        if( !intrinsic_filename.empty() )
        {
            // reading intrinsic parameters
            FileStorage fs(intrinsic_filename, FileStorage::READ);
            if(!fs.isOpened())
            {
                printf("Failed to open file %s\n", intrinsic_filename.c_str());
                return -1;
            }

            Mat M1, D1, M2, D2;
            fs["M1"] >> M1;
            fs["D1"] >> D1;
            fs["M2"] >> M2;
            fs["D2"] >> D2;

            M1 *= scale;
            M2 *= scale;

            fs.open(extrinsic_filename, FileStorage::READ);
            if(!fs.isOpened())
            {
                printf("Failed to open file %s\n", extrinsic_filename.c_str());
                return -1;
            }

            Mat R, T, R1, P1, R2, P2;
            fs["R"] >> R;
            fs["T"] >> T;

            stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

            Mat map11, map12, map21, map22;
            initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
            initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

            Mat img1r, img2r;
            remap(img1, img1r, map11, map12, INTER_LINEAR);
            remap(img2, img2r, map21, map22, INTER_LINEAR);

            img1 = img1r;
            img2 = img2r;
        }

        

        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

        bm->setROI1(roi1);
        bm->setROI2(roi2);
        bm->setPreFilterCap(31);
        bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
        bm->setMinDisparity(0);
        bm->setNumDisparities(numberOfDisparities);
        bm->setTextureThreshold(10);
        bm->setUniquenessRatio(15);
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(32);
        bm->setDisp12MaxDiff(1);

        sgbm->setPreFilterCap(63);
        int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
        sgbm->setBlockSize(sgbmWinSize);

        int cn = img1.channels();

        sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        if(alg==STEREO_HH)
            sgbm->setMode(StereoSGBM::MODE_HH);
        else if(alg==STEREO_SGBM)
            sgbm->setMode(StereoSGBM::MODE_SGBM);
        else if(alg==STEREO_HH4)
            sgbm->setMode(StereoSGBM::MODE_HH4);
        else if(alg==STEREO_3WAY)
            sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

        


        Mat disp, disp8, right_disp, filtered_disp, solved_disp, solved_filtered_disp;
        //Mat img1p, img2p, dispp;
        //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
        //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

        int64 t = getTickCount();
        float disparity_multiplier = 1.0f;
    

        if( alg == STEREO_BM )
        {
            wls_filter = createDisparityWLSFilter(bm);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(bm);

            cvtColor(img1,  img1,  COLOR_BGR2GRAY);
            cvtColor(img2, img2, COLOR_BGR2GRAY);

            bm->compute(img1, img2, disp);
            right_matcher->compute(img2, img1, right_disp);
            if (disp.type() == CV_16S)
                disparity_multiplier = 16.0f;
        }
        else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_HH4 || alg == STEREO_3WAY )
        {
            wls_filter = createDisparityWLSFilter(sgbm);
            // StereoMatcher for disparity filtering
            Ptr<StereoMatcher> right_matcher = createRightMatcher(sgbm);
            sgbm->compute(img1, img2, disp);
            right_matcher->compute(img2, img1, right_disp);

            if (disp.type() == CV_16S)
                disparity_multiplier = 16.0f;
        }

        

        double sigma = parser.has("sigma") ? parser.get<double>("sigma") : 1.5;
        wls_filter->setLambda(8000.0);
        wls_filter->setSigmaColor(sigma);
        

        wls_filter->filter(disp,left_orig,filtered_disp,right_disp);

        

        Mat raw_disp_vis;
        getDisparityVis(disp,raw_disp_vis,1.0);
        namedWindow("raw disparity", WINDOW_AUTOSIZE);
        imshow("raw disparity", raw_disp_vis);
        Mat filtered_disp_vis;
        getDisparityVis(filtered_disp,filtered_disp_vis,1.0);
        namedWindow("filtered disparity", WINDOW_AUTOSIZE);
        imshow("filtered disparity", filtered_disp_vis);

        

        // while(1)
        // {
        //     char key = (char)waitKey();
        //     if( key == 27 || key == 'q' || key == 'Q') // 'ESC'
        //         break;
        // }

        t = getTickCount() - t;
        printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

        //disp = dispp.colRange(numberOfDisparities, img1p.cols);
        if( alg != STEREO_VAR )
            filtered_disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        else
            filtered_disp.convertTo(disp8, CV_8U);

        Mat disp8_3c;
        if (color_display)
            cv::applyColorMap(disp8, disp8_3c, COLORMAP_TURBO);
            //cv::applyColorMap(disp8, disp8_3c, 20);

        if(!disparity_filename.empty())
            imwrite(disparity_filename, color_display ? disp8_3c : disp8);


        int key = cv::waitKey(50);

        if(!point_cloud_filename.empty() && key == 115)
        {
            std::string filename = point_cloud_filename;
            filename.append(std::to_string(filenameCount));
            filename.append(".ply");
            printf("storing the point cloud here: %s", filename.c_str());
            fflush(stdout);
            Mat xyz;
            Mat floatDisp;
            filtered_disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
            reprojectImageTo3D(floatDisp, xyz, Q, true);
            saveXYZ(filename.c_str(), xyz, left_orig);
            printf("\n");
            filenameCount++;
        }

        if( !no_display )
        {
            std::ostringstream oss;
            oss << "disparity  " << (alg==STEREO_BM ? "bm" :
                                    alg==STEREO_SGBM ? "sgbm" :
                                    alg==STEREO_HH ? "hh" :
                                    alg==STEREO_VAR ? "var" :
                                    alg==STEREO_HH4 ? "hh4" :
                                    alg==STEREO_3WAY ? "sgbm3way" : "");
            oss << "  blocksize:" << (alg==STEREO_BM ? SADWindowSize : sgbmWinSize);
            oss << "  max-disparity:" << numberOfDisparities;
            std::string disp_name = oss.str();

            namedWindow("left", cv::WINDOW_NORMAL);
            //imshow("left", img1);
            namedWindow("right", cv::WINDOW_NORMAL);
            //imshow("right", img2);
            namedWindow(disp_name, cv::WINDOW_AUTOSIZE);
            imshow(disp_name, color_display ? disp8_3c : disp8);

            printf("press ESC key or CTRL+C to close...");
            fflush(stdout);
            printf("\n");
            
            if (key == 27){
                exit(0);
            }
        }
    }
    

    return 0;
}