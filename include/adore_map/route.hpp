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
#pragma once
#include <stdlib.h>

#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "adore_map/lane.hpp"
#include "adore_map/map.hpp"
#include "adore_map/quadtree.hpp"
#include "adore_map/r2s_parser.h"
#include "adore_map/road_graph.hpp"
#include "adore_math/distance.h"
#include "adore_math/point.h"
#include "adore_math/pose.h"

#include <unsupported/Eigen/Splines>

namespace adore
{
namespace map
{

struct RouteSection
{
  size_t lane_id;
  double route_s;
  double start_s;
  double end_s;
};

struct Route
{
  Route() = default;

  std::unordered_map<size_t, std::shared_ptr<RouteSection>> lane_to_sections;
  std::map<double, std::shared_ptr<RouteSection>>           s_to_sections;
  std::deque<std::shared_ptr<RouteSection>>                 sections;
  std::shared_ptr<Map>                                      map;
  adore::math::Point2d                                      start;
  adore::math::Point2d                                      destination;
  std::map<double, MapPoint>                                center_lane;

  double               get_length() const;
  void                 add_route_section( Border& points, const MapPoint& start_point, const MapPoint& end_point, bool reverse );
  std::deque<MapPoint> get_shortened_route( double start_s, double desired_length ) const;
  MapPoint             get_map_point_at_s( double distance ) const;
  math::Pose2d         get_pose_at_s( double distance ) const;
  void                 initialize_center_lane();
  void                 initialize_spline();

  template<typename StartPoint, typename EndPoint>
  Route( const StartPoint& start_point, const EndPoint& end, const Map& reference_map );

  template<typename State>
  double get_s( const State& state ) const;

  template<typename TPoint>
  TPoint interpolate_at_s( double distance ) const;

private:

  using Spline1d = Eigen::Spline<double, 1>;

  Spline1d spline_1d_x_;
  Spline1d spline_1d_y_;
  bool     spline_initialized_ = false;
};

template<typename StartPoint, typename EndPoint>
Route::Route( const StartPoint& start_point, const EndPoint& end, const Map& reference_map )
{
  start.x       = start_point.x;
  start.y       = start_point.y;
  destination.x = end.x;
  destination.y = end.y;
  map           = std::make_shared<Map>( reference_map );

  double route_cumulative_s = 0;

  double min_start_dist      = std::numeric_limits<double>::max();
  auto   nearest_start_point = map->quadtree.get_nearest_point( start, min_start_dist );

  double min_end_dist      = std::numeric_limits<double>::max();
  auto   nearest_end_point = map->quadtree.get_nearest_point( end, min_end_dist );

  if( nearest_start_point && nearest_end_point )
  {
    size_t start_lane_id = nearest_start_point->parent_id;
    size_t end_lane_id   = nearest_end_point->parent_id;

    auto lane_id_route = map->lane_graph.get_best_path( start_lane_id, end_lane_id );

    for( size_t i = 0; i < lane_id_route.size(); ++i )
    {
      auto lane = map->lanes.at( lane_id_route[i] );
      add_route_section( lane->borders.center, *nearest_start_point, *nearest_end_point, lane->left_of_reference );
    }

    initialize_center_lane();
  }
}

template<typename State>
double
Route::get_s( const State& state ) const
{
  if( !map )
  {
    std::cerr << "route needs map!" << std::endl;
    return std::numeric_limits<double>::infinity();
  }

  double min_dist = std::numeric_limits<double>::max();
  auto   nearest  = map->quadtree.get_nearest_point( state, min_dist, [&]( const MapPoint& p ) {
    return ( lane_to_sections.find( p.parent_id ) != lane_to_sections.end() );
  } );

  if( !nearest )
  {
    std::cerr << "no nearest" << std::endl;
    return std::numeric_limits<double>::infinity();
  }

  auto near_sec = lane_to_sections.at( nearest->parent_id );

  double dist_along_sec = near_sec->start_s < near_sec->end_s ? ( nearest->s - near_sec->start_s ) : near_sec->start_s - nearest->s;

  return near_sec->route_s + dist_along_sec;
}

template<typename TPoint>
TPoint
Route::interpolate_at_s( double distance ) const
{
  TPoint result;

  if( !spline_initialized_ )
  {
    std::cerr << "Route spline not initialized.\n";
    return result;
  }

  // Evaluate position
  result.x = spline_1d_x_( distance )( 0 );
  result.y = spline_1d_y_( distance )( 0 );

  // If TPoint supports yaw, compute heading from dx/ds and dy/ds
  if constexpr( requires { result.yaw; } )
  {
    auto dx = spline_1d_x_.template derivatives<1>( distance );
    auto dy = spline_1d_y_.template derivatives<1>( distance );

    double dx_ds = dx( 1 );
    double dy_ds = dy( 1 );

    result.yaw = std::atan2( dy_ds, dx_ds );
  }

  return result;
}


} // namespace map
} // namespace adore
