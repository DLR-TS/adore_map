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

#include <limits>

#include <opencv2/opencv.hpp>

#include <adore_map/map.hpp>

namespace adore
{
namespace map
{
// Convert a MapPoint to pixel coordinates
cv::Point2i map_point_to_pixel( const MapPoint& point, const MapPoint& origin, int image_size, double pixel_size );

// Function to draw lane centerlines
cv::Mat raster_lane_centerlines( const Map& map, const MapPoint& center, int image_size, double pixel_size );

// Function to rasterize lane center distances
cv::Mat raster_lane_center_distances( const Map& map, const MapPoint& center, int image_size, double pixel_size );
} // namespace map
} // namespace adore
