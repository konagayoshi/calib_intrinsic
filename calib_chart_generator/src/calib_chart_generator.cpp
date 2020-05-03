/*
  Copyright 2019 coyote009

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
#include <opencv2/aruco/charuco.hpp>

#ifdef _WIN32

#define OCV_VER_STR CVAUX_STR( CV_VERSION_MAJOR ) CVAUX_STR( CV_VERSION_MINOR ) CVAUX_STR( CV_VERSION_REVISION )

#ifdef _DEBUG
#define OCV_LIB_EXT "d.lib"
#else
#define OCV_LIB_EXT ".lib"
#endif

#pragma comment( lib, "opencv_core" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_imgproc" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_aruco" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_highgui" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_imgcodecs" OCV_VER_STR OCV_LIB_EXT )

#endif

struct chart_param
{
    int chart_type; // 0:checker board, 1:charuco board
    int dictionary_type;
    cv::Size num_squares;
    float square_size;
    float marker_size;
    int img_square_size;
    int init_charuco_id;

    chart_param() :
        chart_type( 0 ),
        dictionary_type( cv::aruco::DICT_4X4_250 ),
        num_squares( 7, 6 ),
        square_size( 0.03 ),
        marker_size( 0.015 ),
        img_square_size( 100 ),
        init_charuco_id( 0 )
    {}
};

static int read_chart_param( const cv::String &fname_param, chart_param &param )
{
    cv::FileStorage fs;
    if( !fs.open( fname_param, cv::FileStorage::READ ) )
    {
        return 1;
    }

    fs["CHART_TYPE"] >> param.chart_type;
    fs["DICTIONARY_TYPE"] >> param.dictionary_type;
    fs["NUM_SQUARES"] >> param.num_squares;
    fs["SQUARE_SIZE"] >> param.square_size;
    fs["MARKER_SIZE"] >> param.marker_size;
    fs["IMG_SQUARE_SIZE"] >> param.img_square_size;
    fs["INIT_CHARUCO_ID"] >> param.init_charuco_id;

    return 0;
}

int main( int argc, char **argv )
{
    const cv::String keys =
        "{help | | Show this message}"
        "{@param_file | | Parameter file}"
        "{out | | Output file}"
        "{show | | Just show the chart (no generation)}"
        ;
    cv::CommandLineParser parser( argc, argv, keys );
    parser.about( argv[0] );

    if( parser.has( "help" ) )
    {
        parser.printMessage();
        return 1;
    }

    chart_param param;
    if( parser.has( "@param_file" ) )
    {
        cv::String fname_param = parser.get<cv::String>( "@param_file" );
        if( read_chart_param( fname_param, param ) )
        {
            fprintf( stderr, "Failed to open calibration file\n" );
            return 1;
        }
    }

    cv::String fname_out = "chart.png";
    if( parser.has( "out" ) )
    {
        fname_out = parser.get<cv::String>( "out" );
    }

    bool show_chart = false;
    if( parser.has( "show" ) )
    {
        show_chart = parser.get<bool>( "show" );
    }

    cv::Size size_img = param.num_squares * param.img_square_size;

    // Checker board
    if( param.chart_type == 0 )
    {
        cv::Mat img_board = cv::Mat::ones( size_img, CV_8UC1 ) * 255;

        for( int j = 0; j < param.num_squares.height; j++ )
        {
            for( int i = 0; i < param.num_squares.width; i++ )
            {
                if( (i & 1) ^ (j & 1) )
                {
                    continue;
                }

                cv::rectangle( img_board,
                               cv::Point( i*param.img_square_size,
                                                     j*param.img_square_size ),
                               cv::Point( (i + 1)*param.img_square_size - 1,
                                          (j + 1)*param.img_square_size - 1 ),
                               cv::Scalar::all(0), cv::FILLED );
            }
        }

        if( show_chart )
        {
            cv::imshow( "img_board", img_board );
            cv::waitKey();
        }
        else
        {
            cv::imwrite( fname_out, img_board );
        }
    }
    // Charuco board
    else
    {
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary( param.dictionary_type );
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

        int corner_id = param.init_charuco_id;
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create( param.num_squares.width, param.num_squares.height,
                                                                                  param.square_size, param.marker_size, dictionary );

        for( int j = 0; j < board->ids.size(); j++ )
        {
            board->ids[j] = corner_id;
            corner_id++;
        }

        cv::Mat img_board;
        board->draw( size_img, img_board, 0, 1 );

        if( show_chart )
        {
            cv::imshow( "img_board", img_board );

            std::vector< int > ids;
            std::vector< std::vector< cv::Point2f > > corners, rejected;
            cv::aruco::detectMarkers( img_board, dictionary, corners, ids, params, rejected );
            cv::aruco::refineDetectedMarkers( img_board, board, corners, ids, rejected );

            cv::Mat board_corners, corner_ids;
            if( ids.size() > 0 )
            {
                cv::aruco::interpolateCornersCharuco( corners, ids, img_board, board, board_corners,
                                                      corner_ids, cv::noArray(), cv::noArray(), 1 );
            }

            cv::Mat img_show;
            cv::cvtColor( img_board, img_show, cv::COLOR_GRAY2BGR );
            if( ids.size() > 0 )
            {
                cv::aruco::drawDetectedMarkers( img_show, corners );
            }

            if( board_corners.total() > 0 )
            {
                cv::aruco::drawDetectedCornersCharuco( img_show, board_corners, corner_ids );
            }

            cv::imshow( "img_corners", img_show );

            cv::waitKey();
        }
        else
        {
            cv::imwrite( fname_out, img_board );
        }
    }

    return 0;
}
