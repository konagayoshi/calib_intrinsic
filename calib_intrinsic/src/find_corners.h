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

#ifndef FIND_CORNERS_H_
#define FIND_CORNERS_H_

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

struct charuco_parameter
{
    std::vector<cv::Size> num_square_list;
    float square_size;
    float marker_size;
    int dictionary_name;
    std::vector<int> initial_id;
    int num_boards;
};

struct charuco_board_data
{
    charuco_parameter param;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector< cv::Ptr<cv::aruco::CharucoBoard> > boards;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params;
};

struct board_corners
{
    std::vector<cv::Point2f> corner2d;
    std::vector<cv::Point3f> corner3d;

    void clear()
    {
        corner2d.clear();
        corner3d.clear();
    }
};

void charuco_initialize( charuco_board_data &board );
bool charuco_find_corners( const charuco_board_data &board,
                           const cv::Mat &img_in, board_corners &corner_list,
                           cv::Mat &img_show );

bool checker_find_corners( const cv::Size &size_pat, float pat_len, const cv::Mat &img_in,
                           board_corners &corner_list, cv::Mat &img_show );

#endif
