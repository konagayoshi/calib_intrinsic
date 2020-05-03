/*
  Copyright 2020 coyote009

  This file is part of raspivideocap.

  raspivideocap is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  raspivideocap is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with raspivideocap.  If not, see <http://www.gnu.org/licenses/>.

 */

#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include <raspivideocap.h>
#include "find_corners.h"

#ifdef _WIN32

#define OCV_VER_STR CVAUX_STR(CV_VERSION_MAJOR) CVAUX_STR(CV_VERSION_MINOR) CVAUX_STR(CV_VERSION_REVISION)

#ifdef _DEBUG
#define OCV_LIB_EXT "d.lib"
#else
#define OCV_LIB_EXT ".lib"
#endif

#pragma comment( lib, "opencv_core" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_imgproc" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_calib3d" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_highgui" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_videoio" OCV_VER_STR OCV_LIB_EXT )

#endif


struct calib_param
{
    enum lens_model_ { PINHOLE = 0, FISHEYE = 1, OMNIDIR = 2 };
    
    int file_input; /* 0:Camera input 1:File input */

    /* Parameters for camera input */
    cv::Size size_in;
    int frame_rate;
    std::string fname_imgs_out;
    
    /* Parameters for file input */
    std::string fname_imgs_in;

    /* Common parameters */
    cv::Size size_pat;  /* Chessboard pattern size */
    float pat_len;      /* Chessboard pattern length */
    cv::Size size_show; /* Size to be shown on the screen */
    cv::Rect roi_proc;  /* ROI of the image to be processed */
    std::string fname_calib;
    int lens_model;

    calib_param() :
        file_input( 0 ),
        size_in( 1280, 960 ),
        fname_imgs_out( "../work/calib_imgs_out.avi" ),
        size_pat( 6, 5 ),
        pat_len( 0.03 ),
        size_show( 640, 480 ),
        fname_calib( "../work/calib_intrinsics.yml" ),
        lens_model( PINHOLE )
    {}
};

int read_parameters( const std::string &fname_param, calib_param &param )
{
    cv::FileStorage fs;
    if( fs.open( fname_param, cv::FileStorage::READ ) )
    {
        fs["FILE_INPUT"] >> param.file_input;
        fs["SIZE_IN"] >> param.size_in;
        fs["FRAME_RATE"] >> param.frame_rate;
        fs["FNAME_IMGS_OUT"] >> param.fname_imgs_out;
        fs["FNAME_IMGS_IN"] >> param.fname_imgs_in;
        fs["SIZE_PAT"] >> param.size_pat;
        fs["PAT_LEN"] >> param.pat_len;
        fs["SIZE_SHOW"] >> param.size_show;
        fs["ROI_PROC"] >> param.roi_proc;
        fs["FNAME_CALIB"] >> param.fname_calib;
        fs["LENS_MODEL"] >> param.lens_model;
        fs.release();
    }

    std::cout << "FILE_INPUT=" << param.file_input << std::endl;
    std::cout << "SIZE_IN=" << param.size_in << std::endl;
    std::cout << "FRAME_RATE=" << param.frame_rate << std::endl;
    std::cout << "FNAME_IMGS_OUT=" << param.fname_imgs_out << std::endl;
    std::cout << "FNAME_IMGS_IN=" << param.fname_imgs_in << std::endl;
    std::cout << "SIZE_PAT=" << param.size_pat << std::endl;
    std::cout << "PAT_LEN=" << param.pat_len << std::endl;
    std::cout << "SIZE_SHOW=" << param.size_show << std::endl;
    std::cout << "ROI_PROC=" << param.roi_proc << std::endl;
    std::cout << "FNAME_CALIB=" << param.fname_calib << std::endl;
    std::cout << "LENS_MODEL=" << param.lens_model << std::endl;

    return 0;
}

struct intrinsic_param
{
    cv::Mat mat_cam;
    cv::Mat vec_dist;
    cv::Mat xi;
};

static double calibrate( int lens_model,
                         const cv::Size &size_img, const cv::Size &size_pat, float pat_len,
                         std::vector< std::vector<cv::Point2f> > &corner2d_list,
                         std::vector< std::vector<cv::Point3f> > &corner3d_list,
                         intrinsic_param &calib )
{
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    double err;
    if( lens_model == calib_param::OMNIDIR )
    {
        err = cv::omnidir::calibrate( corner3d_list, corner2d_list, size_img,
                                      calib.mat_cam, calib.xi, calib.vec_dist,
                                      rvecs, tvecs, 0,
                                      cv::TermCriteria( cv::TermCriteria::EPS ||
                                                        cv::TermCriteria::COUNT, 100, DBL_EPSILON ) );
    }
    else if( lens_model == calib_param::FISHEYE )
    {
        err = cv::fisheye::calibrate( corner3d_list, corner2d_list, size_img,
                                      calib.mat_cam, calib.vec_dist, rvecs, tvecs,
                                      cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                                      cv::fisheye::CALIB_CHECK_COND |
                                      cv::fisheye::CALIB_FIX_SKEW /*|
                                      cv::fisheye::CALIB_FIX_K3 |
                                      cv::fisheye::CALIB_FIX_K4*/ );
    }
    else
    {
        /* assume HFoV=60deg for initial guess */
        double f = size_img.width / 2.0 / tan( 30.0 / 180.0 * CV_PI );
        calib.mat_cam = ( cv::Mat_<double>( 3, 3 ) <<
                          f,   0.0, size_img.width / 2.0,
                          0.0,   f, size_img.height / 2.0,
                          0.0, 0.0, 1.0 );

        err = cv::calibrateCamera( corner3d_list, corner2d_list, size_img,
                                   calib.mat_cam, calib.vec_dist, rvecs, tvecs,
                                   cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO );
    }
    
    printf( "Reprojection error = %f\n", err );

    return err;
}

static void scale_show( const std::string &win_name, const cv::Mat &img, const cv::Size &size_show )
{
    cv::Mat img_show;
    
    if( size_show != img.size() )
    {
        cv::resize( img, img_show, size_show, 0, 0, cv::INTER_LINEAR );
    }
    else
    {
        img_show = img;
    }

    cv::imshow( win_name, img_show );
}

int main( int argc, char **argv )
{
    std::string fname_param;
    if( argc >= 2 )
    {
        fname_param = argv[1];
    }
    else
    {
        fname_param = "../work/calib_param.yml";
    }
    
    calib_param param;
    if( read_parameters( fname_param, param ) )
    {
        return 1;
    }

    RaspiVideoCapture capc;
    cv::VideoCapture capf;
    if( param.file_input )
    {
        if( !capf.open( param.fname_imgs_in ) )
        {
            fprintf( stderr, "Failed to open input file\n" );
            return 1;
        }

        param.size_in.width = (int) capf.get( cv::CAP_PROP_FRAME_WIDTH );
        param.size_in.height = (int) capf.get( cv::CAP_PROP_FRAME_HEIGHT );
    }
    else
    {
        if( !capc.open( param.size_in.width, param.size_in.height, param.frame_rate, 0, 1, 1 ) )
        {
            fprintf( stderr, "Failed to open camera\n" );
            return 1;
        }
    }

    cv::Size size_proc;
    if( param.roi_proc.empty() )
    {
        size_proc = param.size_in;
    }
    else
    {
        size_proc = param.roi_proc.size();
    }

    cv::VideoWriter vw;
    if( !param.fname_imgs_out.empty() )
    {
        if( !vw.open( param.fname_imgs_out, CV_FOURCC('H','F','Y','U'), 30.0, param.size_in ) )
        {
            fprintf( stderr, "Failed to open output file\n" );
            return 1;
        }
    }

    cv::Size size_subpic = size_proc / 4;
    cv::Mat img_history;
    img_history = cv::Mat::zeros( size_proc, CV_8UC3 );
    scale_show( "img_history", img_history, param.size_show );

    charuco_board_data board;
    charuco_initialize( board );

    intrinsic_param calib;
    std::vector< std::vector<cv::Point2f> > corner2d_list;
    std::vector< std::vector<cv::Point3f> > corner3d_list;

    cv::FileStorage fs;
    if( !fs.open( param.fname_calib, cv::FileStorage::WRITE ) )
    {
        fprintf( stderr, "Failed to open calibration file\n" );
        return 1;
    }

    while( 1 )
    {
        cv::Mat img_in;
        if( param.file_input )
        {
            if( !capf.read( img_in ) ) { break; }
        }
        else
        {
            if( !capc.read( img_in ) ) { break; }
        }

        cv::Mat img_proc;
        if( param.roi_proc.empty() )
        {
            img_proc = img_in;
        }
        else
        {
            img_in( param.roi_proc ).copyTo( img_proc );
        }

        scale_show( "img_proc", img_proc, param.size_show );

        int key = cv::waitKey( 1 );
        if( key == 27 )
        {
            break;
        }
        else if( key == ' ' || param.file_input )
        {
            cv::Mat img_corners;
            img_proc.copyTo( img_corners );

            board_corners corners;
            bool res = charuco_find_corners( board, img_proc, corners, img_corners );
            if( !res )
            {
                res = checker_find_corners( param.size_pat, param.pat_len, img_proc,
                                            corners, img_corners );
            }
            
            if( res )
            {
                cv::putText( img_proc, "Use corners of this picture(y/n)?", cv::Point( 15, 15 ),
                             cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,255,0), 2 );
                scale_show( "img_proc", img_proc, param.size_show );
                scale_show( "img_corners", img_corners, param.size_show );

                if( cv::waitKey( param.file_input ) == 'y' || param.file_input )
                {
                    int corner_idx = corner2d_list.size();
                    cv::Point img_idx( corner_idx % 4, corner_idx / 4 );
                    cv::Rect roi( size_subpic.width * img_idx.x, size_subpic.height * img_idx.y,
                                  size_subpic.width, size_subpic.height );
                    cv::resize( img_corners, img_history( roi ), size_subpic, 0, 0, cv::INTER_LINEAR );
                    scale_show( "img_history", img_history, param.size_show );
                    cv::waitKey( 1 );

                    corner2d_list.push_back( corners.corner2d );
                    corner3d_list.push_back( corners.corner3d );

                    if( !param.file_input && !param.fname_imgs_out.empty() )
                    {
                        vw.write( img_in );
                    }

                    if( corner2d_list.size() == 16 )
                    {
                        double err = calibrate( param.lens_model,
                                                size_proc, param.size_pat, param.pat_len,
                                                corner2d_list, corner3d_list, calib );

                        fs << "MAT_CAM" << calib.mat_cam;
                        fs << "VEC_DIST" << calib.vec_dist;
                        fs << "XI" << calib.xi;
                        fs << "SIZE_IN" << size_proc;
                        fs << "FRAME_RATE" << param.frame_rate;
                        fs << "ERROR" << err;

                        break;
                    }
                }
            }
            else
            {
                cv::putText( img_proc, "Failed to find corners. Press something to continue.",
                             cv::Point( 15, 15 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,255,0), 2 );
                scale_show( "img_proc", img_proc, param.size_show );
                cv::waitKey();
            }
        }
    }

    if( corner2d_list.size() < 16 )
    {
        return 1;
    }

    if( param.file_input )
    {
        return 0;
    }
    
    /* Test undistortion */
    cv::Mat mat_cam_new;
    cv::Mat mapx, mapy;
    if( param.lens_model == calib_param::OMNIDIR )
    {
        cv::Mat mat_cam_new = ( cv::Mat_<float>( 3, 3 ) <<
                                size_proc.width/2, 0, size_proc.width/2,
                                0, size_proc.width/2, size_proc.height/2,
                                0, 0, 1 );
        cv::omnidir::initUndistortRectifyMap( calib.mat_cam, calib.vec_dist, calib.xi,
                                              cv::Mat(), mat_cam_new,
                                              size_proc, CV_32FC1, mapx, mapy,
                                              cv::omnidir::RECTIFY_PERSPECTIVE );
    }
    else if( param.lens_model == calib_param::FISHEYE )
    {
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            calib.mat_cam, calib.vec_dist, size_proc, cv::Mat(),
            mat_cam_new );
        cv::fisheye::initUndistortRectifyMap( calib.mat_cam, calib.vec_dist,
                                              cv::Mat(), mat_cam_new,
                                              size_proc, CV_32FC1, mapx, mapy );
    }
    else
    {
        mat_cam_new = cv::getOptimalNewCameraMatrix( calib.mat_cam, calib.vec_dist,
                                                     size_proc, 1 );
        cv::initUndistortRectifyMap( calib.mat_cam, calib.vec_dist, cv::Mat(), mat_cam_new,
                                     size_proc, CV_32FC1, mapx, mapy );
    }

    while( 1 )
    {
        cv::Mat img_in;
        if( param.file_input )
        {
            if( !capf.read( img_in ) ) { break; }
        }
        else
        {
            if( !capc.read( img_in ) ) { break; }
        }

        cv::Mat img_proc;
        if( param.roi_proc.empty() )
        {
            img_proc = img_in;
        }
        else
        {
            img_in( param.roi_proc ).copyTo( img_proc );
        }

        cv::Mat img_undist;
        cv::remap( img_proc, img_undist, mapx, mapy, cv::INTER_LANCZOS4 );

        scale_show( "img_proc", img_proc, param.size_show );
        scale_show( "img_undist", img_undist, param.size_show );

        int key = cv::waitKey( 1 );
        if( key == 27 )
        {
            break;
        }
    }
    
    return 0;
}
