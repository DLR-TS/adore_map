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

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "adore_map/border.hpp"
#include "adore_map/map_point.hpp"
#include "adore_map/quadtree.hpp"
#include "adore_math/distance.h"

namespace adore
{
namespace map
{
enum LaneMaterial
{
  asphalt,
  concrete,
  pavement,
  cobble,
  vegetation,
  soil,
  gravel
};

enum LaneType
{
  driving,
  parking,
  restricted,
  none,
  sidewalk,
  biking,
  shoulder,
  tram,
  bus
};

enum RoadCategory
{
  unknown,
  rural,
  motorway,
  town,
  low_speed,
  pedestrian,
  bicycle
};

// German speed limits (converted to meters per second)
// Note: 1 km/h = 0.27778 m/s
constexpr double DRIVING_SPEED_LIMIT_RURAL     = 100.0 * 0.27778; // 100 km/h (27.78 m/s)
constexpr double DRIVING_SPEED_LIMIT_MOTORWAY  = 130.0 * 0.27778; // 130 km/h (36.11 m/s)
constexpr double DRIVING_SPEED_LIMIT_TOWN      = 50.0 * 0.27778;  // 50 km/h (13.89 m/s)
constexpr double DRIVING_SPEED_LIMIT_LOW_SPEED = 30.0 * 0.27778;  // 30 km/h (8.33 m/s)
constexpr double PARKING_SPEED_LIMIT           = 5.0 * 0.27778;   // 5 km/h (1.39 m/s)
constexpr double RESTRICTED_SPEED_LIMIT        = 10.0 * 0.27778;  // 10 km/h (2.78 m/s)
constexpr double BIKING_SPEED_LIMIT            = 25.0 * 0.27778;  // 25 km/h (6.94 m/s)
constexpr double PEDESTRIAN_SPEED_LIMIT        = 5.0 * 0.27778;   // 5 km/h (1.39 m/s)

struct Lane
{
  double       length;
  Borders      borders;
  size_t       id;
  size_t       road_id;
  LaneType     type;
  LaneMaterial material;
  bool         left_of_reference = false;
  double       speed_limit       = 5.0; // Default to 5 m/s

  // Method to calculate the width of the lane at a given s coordinate
  double get_width( double s ) const;

  // Set material based on string input
  void set_material( const std::string& material_str );

  // Set type based on string input and adjust speed limit accordingly
  void set_type( const std::string& type_str, const RoadCategory& road_category );

  Lane() = default;

  Lane( const Border& inner, const Border& outer, size_t id, size_t road_id, bool left_of_reference );

  double get_speed_limit() const;
};

struct Road
{
  std::string                               name;
  std::unordered_set<std::shared_ptr<Lane>> lanes;
  bool                                      one_way = false;
  size_t                                    id;
  RoadCategory                              category;

  void set_category( const std::string& road_category_string );

  Road() = default;

  Road( const std::string& name, size_t id, const std::string& road_category_string, bool one_way ) :
    name( name ),
    lanes(), // Ensure all members are initialized in declaration order
    one_way( one_way ),
    id( id ),
    category( RoadCategory::unknown ) // Provide default category value
  {
    set_category( road_category_string );
  }
};


} // namespace map
} // namespace adore
