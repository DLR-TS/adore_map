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
#include "adore_map/road_graph.hpp"

namespace adore
{
namespace map
{


bool
RoadGraph::add_connection( Connection connection )
{
  to_successors[connection.from_id].insert( connection.to_id );

  to_predecessors[connection.to_id].insert( connection.from_id );

  all_connections.insert( connection );

  return true;
}

std::deque<LaneID>
RoadGraph::get_best_path( LaneID from, LaneID to ) const
{
  // Priority queue to store (accumulated cost, LaneID)
  std::priority_queue<std::pair<double, LaneID>, std::vector<std::pair<double, LaneID>>, std::greater<>> pq;

  // Maps to store the shortest path to each lane and previous lanes (for reconstructing the path)
  std::unordered_map<LaneID, double> shortest_paths;
  std::unordered_map<LaneID, LaneID> previous_roads;

  // Set of visited lanes
  std::unordered_set<LaneID> visited;

  // Initialize the priority queue with the start point
  pq.push( { 0.0, from } );
  shortest_paths[from] = 0.0;

  while( !pq.empty() )
  {
    auto [current_cost, current_road] = pq.top();
    pq.pop();

    // Skip if the lane has already been visited
    if( visited.find( current_road ) != visited.end() )
    {
      continue;
    }
    visited.insert( current_road );

    // If we've reached the destination, reconstruct the path
    if( current_road == to )
    {
      return reconstruct_path( from, to, previous_roads );
    }

    // Explore successors (neighbors)
    if( to_successors.count( current_road ) == 0 )
      continue;
    for( const auto& successor : to_successors.at( current_road ) )
    {
      // Find the connection between current_road and successor
      auto connection = find_connection( current_road, successor );
      if( connection )
      {
        // std::cerr << connection.value() << std::endl;
        double new_cost = current_cost + connection->weight;

        // If this path is shorter, update it
        if( shortest_paths.find( successor ) == shortest_paths.end() || new_cost < shortest_paths[successor] )
        {
          shortest_paths[successor] = new_cost;
          previous_roads[successor] = current_road;
          pq.push( { new_cost, successor } );
        }
      }
    }
  }
  std::cerr << "failed to find route to end" << std::endl;
  // If no path was found, return an empty vector
  return {};
}

std::deque<LaneID>
RoadGraph::reconstruct_path( LaneID from, LaneID to, const std::unordered_map<LaneID, LaneID>& previous_roads ) const
{
  std::deque<LaneID> path;
  LaneID             current = to;

  while( !( current == from ) )
  {
    path.push_front( current );
    current = previous_roads.at( current );
  }
  path.push_front( from );

  return path;
}

std::optional<Connection>
RoadGraph::find_connection( LaneID from_id, LaneID to_id ) const
{
  Connection query;
  query.from_id = from_id;
  query.to_id   = to_id;
  auto it       = all_connections.find( query );
  if( it != all_connections.end() )
  {
    return *it;
  }
  return std::nullopt;
}

} // namespace map
} // namespace adore
