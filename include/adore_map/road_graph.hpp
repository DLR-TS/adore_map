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
#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "adore_map/lane.hpp"

namespace adore
{
namespace map
{

using LaneID = size_t;

enum ConnectionType
{
  END_TO_START,
  END_TO_END,
  START_TO_START,
  START_TO_END
};

struct Connection
{
  LaneID         from_id;
  LaneID         to_id;
  double         weight;
  ConnectionType connection_type;

  // Define equality operator for Connection
  bool
  operator==( const Connection& other ) const
  {
    return ( from_id == other.from_id ) && ( to_id == other.to_id );
  }

  Connection
  inverse()
  {
    Connection inverse;
    inverse.from_id = this->to_id;
    inverse.to_id   = this->from_id;
    if( this->connection_type == END_TO_START )
      inverse.connection_type = START_TO_END;
    else if( this->connection_type == START_TO_END )
      inverse.connection_type = END_TO_START;
    else
      inverse.connection_type = this->connection_type;
    return inverse;
  }

  // Define the << operator for Connection
  friend std::ostream&
  operator<<( std::ostream& os, const Connection& connection )
  {
    os << "from: " << connection.from_id << ", to: " << connection.to_id << ", weight: " << connection.weight;
    return os;
  }
};

// Hash function for Connection
struct ConnectionHasher
{
  std::size_t
  operator()( const Connection& connection ) const
  {
    // Use std::hash for size_t and double, combine the hashes
    std::size_t from_hash = std::hash<size_t>()( connection.from_id );
    std::size_t to_hash   = std::hash<size_t>()( connection.to_id );

    // Combine the hashes
    return from_hash ^ ( to_hash << 1 );
  }
};

struct RoadGraph
{
  RoadGraph() {};
  std::unordered_map<LaneID, std::unordered_set<LaneID>> to_successors;
  std::unordered_map<LaneID, std::unordered_set<LaneID>> to_predecessors;
  std::unordered_set<Connection, ConnectionHasher>       all_connections;

  // Adds a connection between two lanes
  bool add_connection( Connection connection );

  // Finds the best path from one lane to another using Dijkstra
  std::deque<LaneID> get_best_path( LaneID from, LaneID to ) const;

  // Helper function to reconstruct the path from `from` to `to`
  std::deque<LaneID> reconstruct_path( LaneID from, LaneID to, const std::unordered_map<LaneID, LaneID>& previous_roads ) const;

  // Helper function to find the connection between two lanes
  std::optional<Connection> find_connection( LaneID from_id, LaneID to_id ) const;

  RoadGraph
  create_subgraph( const std::unordered_set<LaneID>& valid_lane_ids ) const
  {
    RoadGraph subgraph;

    for( const auto& connection : this->all_connections )
    {
      if( valid_lane_ids.find( connection.from_id ) != valid_lane_ids.end()
          && valid_lane_ids.find( connection.to_id ) != valid_lane_ids.end() )
      {
        subgraph.add_connection( connection );
      }
    }

    return subgraph;
  }
};

} // namespace map
} // namespace adore
