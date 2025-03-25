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
#include "adore_map/lane.hpp"

namespace adore

{
namespace map
{


double
Lane::get_width( double s ) const
{
  if( borders.inner.interpolated_points.empty() || borders.outer.interpolated_points.empty() )
    return 0.0;

  adore::map::MapPoint inner_point = borders.inner.get_interpolated_point( s );
  adore::map::MapPoint outer_point = borders.outer.get_interpolated_point( s );

  return adore::math::distance_2d( inner_point, outer_point );
}

// Set material based on string input
void
Lane::set_material( const std::string &material_str )
{
  static const std::unordered_map<std::string, LaneMaterial> material_map = {
    {    "asphalt",    LaneMaterial::asphalt },
    {   "concrete",   LaneMaterial::concrete },
    {   "pavement",   LaneMaterial::pavement },
    {     "cobble",     LaneMaterial::cobble },
    { "vegetation", LaneMaterial::vegetation },
    {       "soil",       LaneMaterial::soil },
    {     "gravel",     LaneMaterial::gravel }
  };

  auto it = material_map.find( material_str );
  if( it != material_map.end() )
  {
    material = it->second;
  }
  else
  {
    material = LaneMaterial::asphalt;
    // std::cerr << "unexpected material: " << material_str << std::endl;
  }
}

void
Lane::set_type( const std::string &type_str, const RoadCategory &road_category )
{
  static const std::unordered_map<std::string, LaneType> type_map = {
    {    "driving",    LaneType::driving },
    {    "parking",    LaneType::parking },
    { "restricted", LaneType::restricted },
    {       "none",       LaneType::none },
    {   "sidewalk",   LaneType::sidewalk },
    {    "walking",   LaneType::sidewalk }, // both variants found in map...
    {     "biking",     LaneType::biking },
    {    "Bicycle",     LaneType::biking }, // both variants found in map...
    {   "shoulder",   LaneType::shoulder },
    {        "bus",        LaneType::bus },
    {       "tram",       LaneType::tram }
  };

  auto it = type_map.find( type_str );
  if( it != type_map.end() )
  {
    type = it->second;
  }
  else
  {
    // std::cerr << "unexpected lane type: " << type_str << std::endl;
    type = LaneType::none;
  }
  // Set speed limit based on road category and lane type
  switch( type )
  {
    case LaneType::driving:
      switch( road_category )
      {
        case RoadCategory::rural:
          speed_limit = DRIVING_SPEED_LIMIT_RURAL;
          break;
        case RoadCategory::motorway:
          speed_limit = DRIVING_SPEED_LIMIT_MOTORWAY;
          break;
        case RoadCategory::town:
          speed_limit = DRIVING_SPEED_LIMIT_TOWN;
          break;
        case RoadCategory::low_speed:
          speed_limit = DRIVING_SPEED_LIMIT_LOW_SPEED;
          break;
        default:
          speed_limit = DRIVING_SPEED_LIMIT_RURAL; // Default to rural speed if unknown
          break;
      }
      break;
    case LaneType::parking:
      speed_limit = PARKING_SPEED_LIMIT;
      break;
    case LaneType::restricted:
      speed_limit = RESTRICTED_SPEED_LIMIT;
      break;
    case LaneType::sidewalk:
      speed_limit = PEDESTRIAN_SPEED_LIMIT;
      break;
    case LaneType::shoulder:
      speed_limit = PEDESTRIAN_SPEED_LIMIT;
      break;
    case LaneType::bus:
      speed_limit = PEDESTRIAN_SPEED_LIMIT;
      break;
    case LaneType::biking:
      speed_limit = BIKING_SPEED_LIMIT;
      break;
    case LaneType::tram:
      speed_limit = DRIVING_SPEED_LIMIT_TOWN;
      break;
    default:
      speed_limit = 2.0; // Default speed for undefined types
      break;
  }
}

Lane::Lane( const Border &left, const Border &right, size_t id_, size_t road_id_, bool left_of_reference_ )
{
  Borders lane_borders;
  left_of_reference = left_of_reference_;

  lane_borders.inner = left_of_reference ? right : left;
  lane_borders.outer = left_of_reference ? left : right;

  interpolate_borders( lane_borders, 0.5 );
  process_center( lane_borders );

  borders = lane_borders;
  id      = id_;
  set_parent_id( borders, id_ );

  road_id = road_id_;
  length  = left.points.back().s - left.points.front().s;
  if( length < 0 )
    std::cerr << "negative length lane..." << std::endl;
}

double
Lane::get_speed_limit() const
{
  return speed_limit;
}

void
Road::set_category( const std::string &road_category_str )
{
  static const std::unordered_map<std::string, RoadCategory> category_map = {
    {    "unknown",    RoadCategory::unknown },
    {      "rural",      RoadCategory::rural },
    {   "motorway",   RoadCategory::motorway },
    {       "town",       RoadCategory::town },
    {  "low_speed",  RoadCategory::low_speed },
    { "pedestrian", RoadCategory::pedestrian },
    {    "bicycle",    RoadCategory::bicycle }
  };

  auto it = category_map.find( road_category_str );
  if( it != category_map.end() )
  {
    category = it->second;
  }
  else
  {
    // std::cerr << "unexpected category: " << road_category_str << std::endl;
    category = RoadCategory::low_speed;
  }
}

} // namespace map
} // namespace adore
