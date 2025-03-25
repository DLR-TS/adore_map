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
#include "adore_map/rasterizer.hpp"

namespace adore
{
namespace map
{
// Convert a MapPoint to pixel coordinates
cv::Point2i
map_point_to_pixel( const MapPoint& point, const MapPoint& origin, int image_size, double pixel_size )
{
  int x_pixel = static_cast<int>( ( point.x - origin.x ) / pixel_size ) + image_size / 2;
  int y_pixel = image_size / 2 - static_cast<int>( ( point.y - origin.y ) / pixel_size );
  return cv::Point2i( x_pixel, y_pixel );
}

// Function to draw lane centerlines
cv::Mat
raster_lane_centerlines( const Map& map, const MapPoint& center, int image_size, double pixel_size )
{

  // Create an OpenCV Mat with the given size
  cv::Mat output( image_size, image_size, CV_8UC1, cv::Scalar( 255 ) );

  // Define the boundary of the map to consider
  double                       half_size_meters = image_size * pixel_size / 2.0;
  Quadtree<MapPoint>::Boundary query_boundary   = { center.x - half_size_meters, center.x + half_size_meters, center.y - half_size_meters,
                                                    center.y + half_size_meters };

  // Query the quadtree to get points within the boundary
  std::vector<MapPoint> points;
  map.quadtree.query( query_boundary, points );

  // Collect unique lane IDs
  std::unordered_set<size_t> unique_lane_ids;
  for( const auto& point : points )
  {
    unique_lane_ids.insert( point.parent_id );
  }

  // Draw centerlines for each lane
  for( const auto& lane_id : unique_lane_ids )
  {
    auto lane_it = map.lanes.find( lane_id );
    if( lane_it != map.lanes.end() )
    {
      const Border& center_border = lane_it->second->borders.center;

      // Draw interpolated points as a polyline
      std::vector<cv::Point2i> polyline_points;
      for( const auto& interpolated_point : center_border.interpolated_points )
      {
        cv::Point2i pixel_point = map_point_to_pixel( interpolated_point, center, image_size, pixel_size );

        // Ensure the pixel point is within the image bounds
        // if( pixel_point.x >= 0 && pixel_point.x < image_size && pixel_point.y >= 0 && pixel_point.y < image_size )
        // {
        polyline_points.push_back( pixel_point );
        // }
      }

      if( polyline_points.size() > 1 )
      {
        cv::polylines( output, polyline_points, false, cv::Scalar( 0 ) );
      }
    }
  }

  return output;
}

cv::Mat
raster_lane_center_distances( const Map& map, const MapPoint& center, int image_size, double pixel_size )
{
  // Draw lane centerlines as a binary image
  cv::Mat lane_centerlines = raster_lane_centerlines( map, center, image_size, pixel_size );

  cv::distanceTransform( lane_centerlines, lane_centerlines, cv::DIST_L2, 3 );

  return lane_centerlines;
}

} // namespace map
} // namespace adore
