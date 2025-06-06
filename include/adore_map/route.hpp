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

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "adore_map/lane.hpp"
#include "adore_map/map.hpp"
#include "adore_map/quadtree.hpp"
#include "adore_map/r2s_parser.h"
#include "adore_map/road_graph.hpp"
#include "adore_math/distance.h"
#include "adore_math/point.h"
#include "adore_math/pose.h"

namespace adore
{
namespace map
{

// new struct
struct RouteSection
{
  size_t lane_id;
  double route_s;
  double start_s;
  double end_s;
};

struct Route
{
  Route() {};
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

  template<typename StartPoint, typename EndPoint>
  Route( const StartPoint& start_point, const EndPoint& end, const Map& reference_map );

  template<typename State>
  double get_s( const State& state ) const;

  template<typename TPoint>
  TPoint interpolate_at_s( double distance ) const;
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

  // Find nearest start and end points using the quadtree
  double min_start_dist      = std::numeric_limits<double>::max();
  auto   nearest_start_point = map->quadtree.get_nearest_point( start, min_start_dist );

  double min_end_dist      = std::numeric_limits<double>::max();
  auto   nearest_end_point = map->quadtree.get_nearest_point( end, min_end_dist );


  if( nearest_start_point && nearest_end_point )
  {
    size_t start_lane_id = nearest_start_point->parent_id;
    size_t end_lane_id   = nearest_end_point->parent_id;

    // Find the best path between the start and end lanes
    auto lane_id_route = map->lane_graph.get_best_path( start_lane_id, end_lane_id );

    // Iterate over the route and process each lane
    for( size_t i = 0; i < lane_id_route.size(); ++i )
    {
      auto lane = map->lanes.at( lane_id_route[i] );
      add_route_section( lane->borders.center, *nearest_start_point, *nearest_end_point, lane->left_of_reference );
    }

    initialize_center_lane();
  }
}

// get distance to object along route and if the object is within the lane
template<typename State>
double
Route::get_s( const State& state ) const
{
  if( !map ) // no map => we cannot proceed
  {
    std::cerr << "route needs map!" << std::endl;
    return std::numeric_limits<double>::infinity();
  }

  double min_dist = std::numeric_limits<double>::max();
  auto   nearest  = map->quadtree.get_nearest_point( state, min_dist, [&]( const MapPoint& p ) {
    // Return true only if p's lane_id is in our route_lane_ids
    return ( lane_to_sections.find( p.parent_id ) != lane_to_sections.end() );
  } );

  // If we didn't find any point that meets the filter
  if( !nearest )
  {
    std::cerr << "no nearest" << std::endl;
    return std::numeric_limits<double>::infinity();
  }

  auto near_sec = lane_to_sections.at( nearest->parent_id );

  double dist_along_sec = near_sec->start_s < near_sec->end_s ? ( nearest->s - near_sec->start_s ) : near_sec->start_s - nearest->s;

  double route_distance = near_sec->route_s + dist_along_sec;

  return route_distance;
}

template<typename TPoint>
TPoint
Route::interpolate_at_s( double distance ) const
{
  TPoint result;

  // Early exit for empty or single-point lanes
  if( center_lane.empty() )
  {
    return result;
  }

  if( center_lane.size() == 1 )
  {
    const auto& sp = center_lane.begin()->second;
    result.x       = sp.x;
    result.y       = sp.y;

    // If T has a member called 'yaw', set it to 0.0
    if constexpr( requires { result.yaw; } )
    {
      result.yaw = 0.0;
    }
    return result;
  }

  // Interpolation logic
  auto upper_it = center_lane.lower_bound( distance );
  auto lower_it = upper_it;

  double frac = 0.0;

  if( upper_it == center_lane.end() )
  {
    upper_it--;
    lower_it = std::prev( upper_it );
    frac     = 1.0;
  }
  else if( upper_it == center_lane.begin() )
  {
    upper_it++;
    frac = 0.0;
  }
  else
  {
    lower_it--;
    double s1    = lower_it->first;
    double s2    = upper_it->first;
    double denom = ( s2 - s1 );
    frac         = ( std::fabs( denom ) < 1e-9 ) ? 0.0 : ( distance - s1 ) / denom;
  }

  // Linear Interpolation
  double x1 = lower_it->second.x;
  double y1 = lower_it->second.y;
  double x2 = upper_it->second.x;
  double y2 = upper_it->second.y;

  result.x = x1 + frac * ( x2 - x1 );
  result.y = y1 + frac * ( y2 - y1 );

  // Yaw calculation only if T has 'yaw'
  if constexpr( requires { result.yaw; } )
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    if( !( std::fabs( dx ) < 1e-9 && std::fabs( dy ) < 1e-9 ) )
    {
      result.yaw = std::atan2( dy, dx );
    }
  }

  if constexpr( requires {
                  result.s;
                  result.parent_id;
                } )
  {
    auto upper_it = center_lane.lower_bound( distance );
    auto lower_it = upper_it;

    if( upper_it == center_lane.end() )
    {
      upper_it--;
      lower_it = std::prev( upper_it );
    }
    else if( upper_it == center_lane.begin() )
    {
      lower_it = upper_it;
    }
    else
    {
      lower_it = std::prev( upper_it );
    }

    double dist_to_lower = std::abs( distance - lower_it->first );
    double dist_to_upper = std::abs( distance - upper_it->first );
    auto   nearest_it    = ( dist_to_lower < dist_to_upper ) ? lower_it : upper_it;

    result.parent_id = nearest_it->second.parent_id;
    result.s         = distance;
  }

  return result;
}
} // namespace map
} // namespace adore
