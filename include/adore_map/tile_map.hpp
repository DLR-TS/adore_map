/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#include <cmath>

#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

#include "adore_map/rasterizer.hpp"

namespace adore
{

namespace map
{


template<typename TileFunction>
class TileMap
{
public:

  TileMap( std::shared_ptr<Map> map, TileFunction tile_generation_function, int mat_size, double pixel_size, double initial_x,
           double initial_y ) :
    map( map ),
    tile_generation_function( tile_generation_function ),
    mat_size( mat_size ),
    pixel_size( pixel_size )
  {
    center_x = initial_x;
    center_y = initial_y;
    initialize_grid();
  }

  template<typename Point>
  cv::Mat
  get_cropped_mat( const Point& point, int crop_size )
  {
    int     half_crop = crop_size / 2;
    cv::Mat cropped( crop_size, crop_size, CV_8UC1, cv::Scalar( 0 ) );

    // 1. Calculate the position of the point in the overall tile map
    double point_global_x = ( point.x - center_x ) / pixel_size + ( 3 * mat_size / 2 );
    double point_global_y = 3 * mat_size / 2 - ( point.y - center_y ) / pixel_size;

    // std::cerr << "Point global position: (" << point_global_x << ", " << point_global_y << ")\n";

    for( int i = 0; i < 3; ++i )
    {
      for( int j = 0; j < 3; ++j )
      {
        // Calculate the top-left corner of the current tile in global coordinates
        double tile_origin_x = i * mat_size;
        double tile_origin_y = j * mat_size;

        // 2. Calculate the position of the point in that tile
        double point_in_tile_x = point_global_x - tile_origin_x;
        double point_in_tile_y = point_global_y - tile_origin_y;

        // Define the ROI around the point in tile coordinates
        int roi_start_x = std::max( 0, static_cast<int>( std::round( point_in_tile_x - half_crop ) ) );
        int roi_start_y = std::max( 0, static_cast<int>( std::round( point_in_tile_y - half_crop ) ) );
        int roi_end_x   = std::min( mat_size, static_cast<int>( std::round( point_in_tile_x + half_crop ) ) );
        int roi_end_y   = std::min( mat_size, static_cast<int>( std::round( point_in_tile_y + half_crop ) ) );

        if( roi_start_x >= roi_end_x || roi_start_y >= roi_end_y )
        {
          // Skip if there's no valid overlap
          // std::cerr << "No valid ROI for Tile (" << i << ", " << j << ")\n";
          continue;
        }

        cv::Rect tile_roi( roi_start_x, roi_start_y, roi_end_x - roi_start_x, roi_end_y - roi_start_y );

        // 3. Calculate the position of the ROI in the cropped matrix
        int cropped_start_x = roi_start_x + tile_origin_x - point_global_x + half_crop;
        int cropped_start_y = roi_start_y + tile_origin_y - point_global_y + half_crop;
        int cropped_width   = tile_roi.width;
        int cropped_height  = tile_roi.height;

        if( cropped_start_x < 0 || cropped_start_y < 0 || cropped_start_x + cropped_width > crop_size
            || cropped_start_y + cropped_height > crop_size )
        {
          // Skip if the ROI doesn't fit into the cropped matrix
          // std::cerr << "Cropped ROI out of bounds for Tile (" << i << ", " << j << ")\n";
          continue;
        }

        cv::Rect cropped_roi( cropped_start_x, cropped_start_y, cropped_width, cropped_height );

        // 4. Copy values
        try
        {
          grid[i][j]( tile_roi ).copyTo( cropped( cropped_roi ) );
          // std::cerr << "Copied from Tile (" << i << ", " << j << ") ROI: " << tile_roi << " to Cropped ROI: " << cropped_roi << "\n";
        }
        catch( const cv::Exception& e )
        {
          // std::cerr << "OpenCV Exception during copyTo: " << e.what() << "\n";
        }
      }
    }

    return cropped;
  }

  template<typename Point>
  void
  update( const Point& point )
  {
    double delta_x = point.x - center_x;
    double delta_y = point.y - center_y;

    // Determine if the grid needs to shift
    int shift_x = std::floor( delta_x / ( mat_size * pixel_size ) );
    int shift_y = std::floor( delta_y / ( mat_size * pixel_size ) );

    if( shift_x != 0 || shift_y != 0 )
    {
      shift_grid( shift_x, shift_y );
    }
  }


private:

  std::shared_ptr<Map>              map;
  TileFunction                      tile_generation_function; // Function to generate tiles
  double                            center_x, center_y;       // Center coordinates of the grid in world space
  int                               mat_size;                 // Size of each individual cv::Mat
  double                            pixel_size;               // Size of each pixel in meters
  std::vector<std::vector<cv::Mat>> grid;                     // 3x3 grid of cv::Mat

  void
  initialize_grid()
  {
    grid.resize( 3, std::vector<cv::Mat>( 3 ) );
    for( int i = 0; i < 3; ++i )
    {
      for( int j = 0; j < 3; ++j )
      {
        // Center tile is at (1, 1); offset others accordingly
        grid[i][2 - j] = tile_generation_function( map, center_x + ( i - 1 ) * mat_size * pixel_size,
                                                   center_y + ( j - 1 ) * mat_size * pixel_size, mat_size, pixel_size );
      }
    }
  }

  // Shift the grid and redraw the necessary matrices
  void
  shift_grid( int shift_x, int shift_y )
  {
    center_x += shift_x * mat_size * pixel_size;
    center_y += shift_y * mat_size * pixel_size;
    std::vector<std::vector<cv::Mat>> new_grid( 3, std::vector<cv::Mat>( 3 ) );

    // Copy existing tiles to new positions
    for( int i = 0; i < 3; ++i )
    {
      for( int j = 0; j < 3; ++j )
      {
        int new_i = i - shift_x;
        int new_j = j - shift_y;

        if( new_i >= 0 && new_i < 3 && new_j >= 0 && new_j < 3 )
        {
          new_grid[new_i][new_j] = grid[i][j];
        }
      }
    }

    // Fill in new tiles for positions that have shifted
    for( int i = 0; i < 3; ++i )
    {
      for( int j = 0; j < 3; ++j )
      {
        // if( new_grid[i][j].empty() )
        // {
        new_grid[i][2 - j] = tile_generation_function( map, center_x + ( i - 1 ) * mat_size * pixel_size,
                                                       center_y + ( j - 1 ) * mat_size * pixel_size, mat_size, pixel_size );
        // }
      }
    }

    // Update the grid and recalculate the center
    grid = std::move( new_grid );

    std::cerr << "Grid shifted. New center: center_x = " << center_x << ", center_y = " << center_y << "\n";
  }
};
} // namespace map
} // namespace adore
