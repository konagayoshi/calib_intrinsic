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

#include "find_corners.h"

void charuco_initialize( charuco_board_data &board )
{
    board.param.num_square_list.resize( 1, cv::Size( 7, 6 ) );
    board.param.square_size = 0.03;
    board.param.marker_size = 0.015;
    board.param.dictionary_name = cv::aruco::DICT_4X4_250;
    board.param.num_boards = board.param.num_square_list.size();

    for( int i = 0; i < board.param.num_boards; i++ )
    {
        cv::Size num_squares = board.param.num_square_list[i];
        board.param.initial_id.push_back( i * num_squares.width * num_squares.height / 2 );
        //printf( "%d\n", board.param.initial_id[i] );
    }

    board.dictionary = cv::aruco::getPredefinedDictionary( board.param.dictionary_name );

    board.boards.resize( board.param.num_boards );
    for( int i = 0; i < board.param.num_boards; i++ )
    {
        board.boards[i] = cv::aruco::CharucoBoard::create( board.param.num_square_list[i].width,
                                                           board.param.num_square_list[i].height,
                                                           board.param.square_size,
                                                           board.param.marker_size,
                                                           board.dictionary );

        //printf( "id_size = %d\n", board.boards[i]->ids.size() );
        for( int j = 0; j < board.boards[i]->ids.size(); j++ )
        {
            board.boards[i]->ids[j] = board.param.initial_id[i] + j;
        }
    }

    board.detector_params = cv::aruco::DetectorParameters::create();
    board.detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
}

bool charuco_find_corners( const charuco_board_data &board,
                           const cv::Mat &img_in, board_corners &corner_list, cv::Mat &img_show )
{
    corner_list.clear();
    
    for( int i = 0; i < board.param.num_boards; i++ )
    {
        std::vector< int > ids;
        std::vector< std::vector< cv::Point2f > > corners, rejected;
        cv::aruco::detectMarkers( img_in, board.dictionary, corners, ids,
                                  board.detector_params, rejected, cv::Mat(), cv::Mat() );
        cv::aruco::refineDetectedMarkers( img_in, board.boards[i], corners, ids,
                                          rejected, cv::Mat(), cv::Mat() );

        std::vector<cv::Point2f> corners_charuco;
        std::vector<int> ids_charuco;
        if( ids.size() > 0 )
        {
            cv::aruco::interpolateCornersCharuco( corners, ids, img_in, board.boards[i],
                                                  corners_charuco, ids_charuco,
                                                  cv::Mat(), cv::Mat(), 1 );
        }

        if( !img_show.empty() )
        {
            if( ids.size() > 0 )
            {
                cv::aruco::drawDetectedMarkers( img_show, corners /*, ids*/ );
            }
            if( corners_charuco.size() > 0 )
            {
                cv::aruco::drawDetectedCornersCharuco( img_show, corners_charuco /*, ids_charuco*/ );

                char buf[32];
                sprintf( buf, "Board%d", i );
                cv::putText( img_show, buf, corners_charuco[0], cv::FONT_HERSHEY_SIMPLEX,
                             0.5, CV_RGB(0,255,0) );
            }
        }

        corner_list.corner2d.insert( corner_list.corner2d.end(),
                                     corners_charuco.begin(), corners_charuco.end() );
        
        for( int j = 0; j < ids_charuco.size(); j++ )
        {
            corner_list.corner3d.push_back(
                board.boards[i]->chessboardCorners[ids_charuco[j]] );
        }
    }

    if( corner_list.corner2d.size() >= 4 )
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool find_corners( const cv::Size &size_pat, const cv::Mat &img_in,
                          std::vector<cv::Point2f> &corners )
{
    cv::Mat img_proc;
    if( img_in.channels() == 3 )
    {
        cv::cvtColor( img_in, img_proc, cv::COLOR_BGR2GRAY );
    }
    else
    {
        img_proc = img_in;
    }

    bool found = cv::findChessboardCorners( img_proc, size_pat, corners,
                                            cv::CALIB_CB_FAST_CHECK |
                                            cv::CALIB_CB_ADAPTIVE_THRESH |
                                            cv::CALIB_CB_NORMALIZE_IMAGE );

    if( found )
    {
        cv::cornerSubPix( img_proc, corners, cv::Size( 11, 11 ), cv::Size(-1,-1),
                          cv::TermCriteria( cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1 ) );
    }

    return found;
}

bool checker_find_corners( const cv::Size &size_pat, float pat_len, const cv::Mat &img_in,
                           board_corners &corner_list, cv::Mat &img_show )
{
    corner_list.clear();
    
    if( !find_corners( size_pat, img_in, corner_list.corner2d ) )
    {
        return false;
    }

    for( int y = 0; y < size_pat.height; y++ )
    {
        for( int x = 0; x < size_pat.width; x++ )
        {
            corner_list.corner3d.push_back( cv::Point3f( x * pat_len, y * pat_len, 0.0 ) );
        }
    }

    cv::drawChessboardCorners( img_show, size_pat, corner_list.corner2d, true );

    return true;
}
