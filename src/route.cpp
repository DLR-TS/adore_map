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
#include "adore_map/route.hpp"

namespace adore
{
namespace map
{

void
Route::add_route_section( Border& lane_to_add, const MapPoint& start_point, const MapPoint& end_point, bool reverse = false )

{
  if( lane_to_add.interpolated_points.empty() )
    return;
  std::shared_ptr<RouteSection> next = std::make_shared<RouteSection>();
  next->lane_id                      = lane_to_add.interpolated_points[0].parent_id;

  if( reverse )
  {
    next->end_s   = lane_to_add.interpolated_points.front().s;
    next->start_s = lane_to_add.interpolated_points.back().s;
  }
  else
  {
    next->start_s = lane_to_add.interpolated_points.front().s;
    next->end_s   = lane_to_add.interpolated_points.back().s;
  }
  if( start_point.parent_id == next->lane_id )
    next->start_s = start_point.s;
  if( end_point.parent_id == next->lane_id )
    next->end_s = end_point.s;

  lane_to_sections[next->lane_id] = next;
  sections.push_back( next );
}

double
Route::get_length() const
{
  if( center_lane.empty() )
  {
    return 0.0;
  }
  auto center_lane_iter = center_lane.end();
  center_lane_iter--;
  return center_lane_iter->first;
}

std::deque<MapPoint>
Route::get_shortened_route( double start_s, double desired_length ) const
{
  std::deque<MapPoint> result;

  auto center_lane_iter = center_lane.lower_bound( start_s );
  auto upper_bound      = center_lane.upper_bound( start_s + desired_length );

  while( center_lane_iter != center_lane.end() && center_lane_iter != upper_bound )
  {
    result.push_back( center_lane_iter->second );
    center_lane_iter++;
  }
  return result;
}

MapPoint
Route::get_map_point_at_s( double distance ) const
{
  return interpolate_at_s<MapPoint>( distance );
}

math::Pose2d
Route::get_pose_at_s( double distance ) const
{
  return interpolate_at_s<math::Pose2d>( distance );
}

void
Route::initialize_center_lane()
{
  center_lane.clear();
  if( !map )
  {
    return;
  }

  double s = 0.0;

  // Go through each RouteSection, gather center points from that lane in [start_s, end_s]
  for( auto& section : sections )
  {

    section->route_s = s;
    s_to_sections[s] = section;

    auto lane_it = map->lanes.find( section->lane_id );
    if( lane_it == map->lanes.end() )
      continue;

    std::shared_ptr<Lane> lane_ptr = lane_it->second;
    if( !lane_ptr )
      continue;

    const auto& cpoints = lane_ptr->borders.center.interpolated_points;
    bool        reverse = section->end_s < section->start_s;
    double      start_s = reverse ? section->end_s : section->start_s;
    double      end_s   = reverse ? section->start_s : section->end_s;

    for( size_t i = 0; i < cpoints.size(); ++i )
    {
      auto point = cpoints[reverse ? cpoints.size() - i - 1 : i];
      // If reversed, local_s should go 0...seg_length in the same direction
      double local_s = reverse ? ( end_s - point.s ) : ( point.s - start_s );
      if( point.s >= start_s && point.s <= end_s )
      {
        center_lane[s + local_s] = point;
      }
    }
    s = get_length();
  }

  // filter points that are too close to each other
  // loop over center_lane keys and remove points that are too close in s
  for( auto it = center_lane.begin(); it != center_lane.end(); )
  {
    auto next_it = std::next( it );
    if( next_it != center_lane.end() && std::fabs( next_it->first - it->first ) < 0.5 )
    {
      center_lane.erase( it );
      it = next_it;
    }
    else
    {
      ++it;
    }
  }
}

} // namespace map
} // namespace adore
