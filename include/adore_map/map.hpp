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
#include <cmath>
#include <stdlib.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "adore_map/border.hpp"
#include "adore_map/lane.hpp"
#include "adore_map/quadtree.hpp"
#include "adore_map/r2s_parser.h"
#include "adore_map/road_graph.hpp"
#include "adore_math/distance.h"

namespace adore
{
namespace map
{


// The Map class definition
class Map
{
public:


  Map() {};
  Map( const std::string& map_file_location );

  Quadtree<MapPoint>                      quadtree;
  RoadGraph                               lane_graph;
  std::map<size_t, Road>                  roads;
  std::map<size_t, std::shared_ptr<Lane>> lanes;

  double get_lane_speed_limit( size_t lane_id ) const;

  template<typename CenterPoint>
  Map
  get_submap( const CenterPoint& center, double width, double height ) const
  {
    Map submap;

    // Define the query boundary based on center, width, and height
    Quadtree<MapPoint>::Boundary query_boundary;
    query_boundary.x_min = center.x - width / 2.0;
    query_boundary.x_max = center.x + width / 2.0;
    query_boundary.y_min = center.y - height / 2.0;
    query_boundary.y_max = center.y + height / 2.0;

    // Set up the quadtree boundaries for the submap
    submap.quadtree.boundary = query_boundary;
    submap.quadtree.capacity = this->quadtree.capacity; // Copy capacity

    // Query the quadtree to find all points within the boundary
    std::vector<MapPoint> found_points;
    this->quadtree.query( query_boundary, found_points );


    // Collect unique lane IDs from the found points
    std::unordered_set<size_t> unique_lane_ids;
    for( const auto& point : found_points )
    {
      unique_lane_ids.insert( point.parent_id );
    }

    // Copy the lanes into the submap
    for( const auto& lane_id : unique_lane_ids )
    {
      auto it = this->lanes.find( lane_id );
      if( it != this->lanes.end() )
      {

        // Deep copy the Lane
        std::shared_ptr<Lane> copied_lane = std::make_shared<Lane>( *it->second );
        submap.lanes[lane_id]             = copied_lane;

        // Insert all MapPoints from the lane's borders into the submap's quadtree
        const Borders& borders = copied_lane->borders;

        for( const auto& point : borders.center.interpolated_points )
        {
          submap.quadtree.insert( point );
        }


        // Copy associated roads
        auto road_it = this->roads.find( it->second->road_id );
        if( road_it != this->roads.end() )
        {
          // Check if the road is already copied
          if( submap.roads.find( road_it->first ) == submap.roads.end() )
          {
            Road copied_road = road_it->second;
            // Clear the lanes in the copied road and add the copied lane
            copied_road.lanes.clear();
            copied_road.lanes.insert( copied_lane );
            submap.roads[road_it->first] = copied_road;
          }
          else
          {
            // Add the lane to the existing road in the submap
            submap.roads[road_it->first].lanes.insert( copied_lane );
          }
        }
      }
    }

    submap.lane_graph = lane_graph.create_subgraph( unique_lane_ids );

    return submap;
  }

  template<typename Point>
  bool
  is_point_on_road( const Point& point )
  {
    double min_dist   = std::numeric_limits<double>::max();
    auto   near_point = quadtree.get_nearest_point( point, min_dist );

    if( !near_point )
      return false;


    if( lanes.count( near_point->parent_id ) == 0 )
    {
      std::cerr << "is_point_on_road failed to get width from lane - nearest point not in lanes" << std::endl;
      return false;
    }

    double width = lanes[near_point->parent_id]->get_width( near_point->s );
    if( min_dist < width / 2 )
    {
      return true;
    }
    return false;
  }

private:
};

} // namespace map
} // namespace adore
