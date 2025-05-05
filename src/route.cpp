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

} // namespace map
} // namespace adore
