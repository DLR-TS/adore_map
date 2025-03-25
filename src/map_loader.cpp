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
#include "adore_map/map_loader.hpp"

namespace adore
{
namespace map
{
Map
MapLoader::load_from_file( const std::string& map_file_location, bool ignore_non_driving )
{
  // Extract file extension
  std::string::size_type dot_pos = map_file_location.find_last_of( '.' );
  if( dot_pos == std::string::npos )
  {
    throw std::invalid_argument( "File has no extension: " + map_file_location );
  }

  // Convert the file extension to lowercase
  std::string extension = map_file_location.substr( dot_pos + 1 );
  std::transform( extension.begin(), extension.end(), extension.begin(), []( unsigned char c ) { return std::tolower( c ); } );

  // Decide based on the file extension
  if( extension == "xodr" )
  {
    return load_from_xodr_file( map_file_location, ignore_non_driving );
  }

  if( extension == "r2sr" )
  {
    return load_from_r2s_file( map_file_location, ignore_non_driving );
  }

  throw std::invalid_argument( "Unsupported file extension: " + extension );
}

size_t
MapLoader::generate_lane_id()
{
  static size_t lane_id_counter = 0;
  return ++lane_id_counter;
}

Map
MapLoader::load_from_r2s_file( const std::string& map_file_location, bool /*ignore_non_driving*/ )
{
  Map map;

  auto border_data_r2sr = adore::r2s::load_border_data_from_r2sr_file( map_file_location );
  auto border_data_r2sl = adore::r2s::load_border_data_from_r2sl_file( map_file_location );

  create_from_r2s( map, border_data_r2sr, border_data_r2sl );

  return map;
}

void
MapLoader::create_from_r2s( Map& map, const std::vector<r2s::BorderDataR2SR>& standard_lines,
                            const std::vector<r2s::BorderDataR2SL>& lane_boundaries )
{
  set_quadtree_bounds( map, standard_lines, lane_boundaries );

  Bordermap                                                     refline_to_border;
  std::unordered_map<int, std::shared_ptr<r2s::BorderDataR2SL>> id_to_border;

  for( const auto& boundary : lane_boundaries )
  {
    if( boundary.linetype != "driving" )
      continue;
    auto boundary_pointer = std::make_shared<r2s::BorderDataR2SL>( boundary );
    refline_to_border[boundary.parent_id].push_back( boundary_pointer );
    id_to_border[boundary.id] = boundary_pointer;
  }

  for( const auto& r2s_ref_line : standard_lines )
  {
    // Create a Road object for each reference line
    Road                road             = Road( r2s_ref_line.streetname, r2s_ref_line.id, r2s_ref_line.category, r2s_ref_line.oneway );
    Border              reference_line   = create_reference_line( r2s_ref_line );
    std::vector<Border> relevant_borders = process_relevant_borders( refline_to_border, r2s_ref_line, reference_line );
    std::vector<double> s_positions      = collect_s_positions( reference_line, relevant_borders );

    if( s_positions.size() < 2 )
      continue;
    // Create lanes in each interval
    for( auto s_iter = s_positions.begin(); s_iter != s_positions.end() - 1; ++s_iter )
    {
      std::vector<BorderWithOffset> clipped_borders = get_clipped_borders( relevant_borders, reference_line, *s_iter, *( s_iter + 1 ) );
      for( size_t i = 1; i < clipped_borders.size(); ++i )
      {
        make_lane( clipped_borders[i - 1], clipped_borders[i], road, map, id_to_border );
      }
    }
    map.roads[r2s_ref_line.id] = road;
  }

  map.lane_graph = infer_graph_from_proximity_of_lanes( map, LANE_CONNECTION_DIST );
}

void
MapLoader::make_lane( const BorderWithOffset& left_border, const BorderWithOffset& right_border, Road& road, Map& map,
                      const std::unordered_map<int, std::shared_ptr<r2s::BorderDataR2SL>>& id_to_border )
{
  bool left_of_reference = left_border.lateral_offset < 0.0;
  auto lane_ptr          = std::make_shared<Lane>( left_border.clipped_border, right_border.clipped_border, generate_lane_id(), road.id,
                                                   left_of_reference );

  int boundary_id = left_of_reference ? left_border.clipped_border.points.front().parent_id
                                      : right_border.clipped_border.points.front().parent_id;

  if( id_to_border.find( boundary_id ) != id_to_border.end() )
  {
    auto boundary = id_to_border.at( boundary_id );
    lane_ptr->set_material( boundary->material );
    lane_ptr->set_type( boundary->linetype, road.category );
  }

  map.lanes[lane_ptr->id] = lane_ptr;
  road.lanes.insert( lane_ptr );

  for( const auto& p : lane_ptr->borders.center.interpolated_points )
  {
    map.quadtree.insert( p );
  }
}

std::vector<Border>
MapLoader::process_relevant_borders( Bordermap& refline_to_border, const adore::r2s::BorderDataR2SR& r2s_ref_line, Border& reference_line )
{
  std::vector<Border> relevant_borders;
  auto                relevant_boundaries = refline_to_border.find( r2s_ref_line.id );
  if( relevant_boundaries != refline_to_border.end() )
  {
    for( const auto& l_border : refline_to_border[r2s_ref_line.id] )
    {
      // Create the border from R2S data
      Border border;
      for( size_t i = 0; i < l_border->x.size(); ++i )
      {
        border.points.emplace_back( l_border->x[i], l_border->y[i], l_border->id );
      }
      border.reparameterize_based_on_reference( reference_line );
      relevant_borders.push_back( std::move( border ) );
    }
  }
  return relevant_borders;
}

Border
MapLoader::create_reference_line( const adore::r2s::BorderDataR2SR& r2s_ref_line )
{
  Border reference_line;
  for( size_t i = 0; i < r2s_ref_line.x.size(); ++i )
  {
    reference_line.points.emplace_back( r2s_ref_line.x[i], r2s_ref_line.y[i], r2s_ref_line.id );
  }
  reference_line.initialize_spline();
  reference_line.compute_s_values();
  reference_line.compute_length();
  return reference_line;
}

std::vector<double>
MapLoader::collect_s_positions( Border& reference_line, const std::vector<Border>& borders )
{
  std::vector<double> s_positions = { 0.0, reference_line.get_length() };

  for( const auto& border : borders )
  {
    s_positions.push_back( border.points.front().s );
    s_positions.push_back( border.points.back().s );
  }

  // Sort and remove duplicate values within MIN_LANE_LENGTH tolerance
  std::sort( s_positions.begin(), s_positions.end() );

  auto it = std::unique( s_positions.begin(), s_positions.end(),
                         []( double a, double b ) { return std::abs( a - b ) <= MIN_LANE_LENGTH; } );
  s_positions.erase( it, s_positions.end() );

  return s_positions;
}

std::vector<BorderWithOffset>
MapLoader::get_clipped_borders( const std::vector<Border>& borders, const Border& reference_line, double s_start, double s_end )
{
  std::vector<BorderWithOffset> borders_with_offsets;

  Border ref_line_clipped = reference_line.make_clipped( s_start, s_end );
  if( ref_line_clipped.points.size() < 2 )
    return borders_with_offsets;
  borders_with_offsets.push_back( { ref_line_clipped, 0.0 } );

  for( const auto& border : borders )
  {
    if( border.points.back().s <= s_start || border.points.front().s >= s_end )
      continue;
    Border clipped_border = border.make_clipped( s_start, s_end );
    if( clipped_border.points.size() < 2 )
      continue;

    MapPoint border_point   = clipped_border.get_interpolated_point( ( s_start + s_end ) / 2.0 );
    double   lateral_offset = compute_lateral_offset( ref_line_clipped, border_point );
    borders_with_offsets.push_back( { clipped_border, lateral_offset } );
  }
  std::sort( borders_with_offsets.begin(), borders_with_offsets.end(),
             []( const BorderWithOffset& a, const BorderWithOffset& b ) { return a.lateral_offset < b.lateral_offset; } );

  return borders_with_offsets;
}

double
MapLoader::compute_lateral_offset( const Border& reference_line, const MapPoint& target_point )
{
  // Compute direction vector of reference line at ref_point
  MapPoint ref_point = reference_line.get_interpolated_point( ( reference_line.points.front().s + reference_line.points.back().s ) / 2.0 );
  double   s         = ref_point.s;
  const double delta_s        = 0.01;
  MapPoint     ref_point_next = reference_line.get_interpolated_point( s + delta_s );

  double dx        = ref_point_next.x - ref_point.x;
  double dy        = ref_point_next.y - ref_point.y;
  double magnitude = std::hypot( dx, dy );

  if( magnitude == 0.0 )
  {
    return 0.0; // Avoid division by zero
  }

  // Normalized normal vector
  double nx = -dy / magnitude;
  double ny = dx / magnitude;

  // Vector from ref_point to target_point
  double tx = target_point.x - ref_point.x;
  double ty = target_point.y - ref_point.y;

  // Compute dot product (lateral offset)
  return -( tx * nx + ty * ny );
}

RoadGraph
MapLoader::infer_graph_from_proximity_of_lanes( Map& map, double proximity_threshold )
{
  RoadGraph lane_graph;
  // Iterate through all lanes
  for( const auto& [lane_id, lane_ptr] : map.lanes )
  {
    const auto& lane = *lane_ptr;
    if( lane.borders.center.interpolated_points.empty() )
      continue;

    // Query the quadtree for points near the start and end of the current lane
    std::vector<MapPoint> nearby_points;
    map.quadtree.query_range( lane.borders.center.interpolated_points.front(), proximity_threshold, nearby_points );
    map.quadtree.query_range( lane.borders.center.interpolated_points.back(), proximity_threshold, nearby_points );

    // Remove duplicates in the nearby points list (as we query both start and end)
    std::sort( nearby_points.begin(), nearby_points.end(),
               []( const MapPoint& a, const MapPoint& b ) { return a.parent_id < b.parent_id; } );

    nearby_points.erase( std::unique( nearby_points.begin(), nearby_points.end() ), nearby_points.end() );

    // Check connections for nearby lanes
    for( const auto& point : nearby_points )
    {
      if( point.parent_id == lane_id || map.lanes.find( point.parent_id ) == map.lanes.end() )
        continue; // Avoid self-connections

      const Lane& other_lane = *map.lanes.at( point.parent_id );

      auto [min_distance, connection_type] = calculate_lane_distance( lane, other_lane );

      // Only add connections if within proximity threshold
      if( min_distance <= proximity_threshold )
      {
        // Prohibit going in opposite direction to other lane
        if( connection_type == END_TO_START && !( !other_lane.left_of_reference && !lane.left_of_reference ) )
          continue;
        if( connection_type == START_TO_END && !( other_lane.left_of_reference && lane.left_of_reference ) )
          continue;
        if( connection_type == START_TO_START && !( !other_lane.left_of_reference && lane.left_of_reference ) )
          continue;
        if( connection_type == END_TO_END && !( other_lane.left_of_reference && !lane.left_of_reference ) )
          continue;
        Connection connection;
        connection.from_id         = lane.id;
        connection.to_id           = other_lane.id;
        connection.weight          = lane.length;
        connection.connection_type = connection_type;
        lane_graph.add_connection( connection );
      }
    }
  }

  return lane_graph;
}

std::pair<double, ConnectionType>
MapLoader::calculate_lane_distance( const Lane& from_lane, const Lane& to_lane )
{
  const auto& from_points = from_lane.borders.center.interpolated_points;
  const auto& to_points   = to_lane.borders.center.interpolated_points;

  if( from_points.empty() || to_points.empty() )
  {
    return { std::numeric_limits<double>::max(), END_TO_START };
  }

  std::array<std::pair<double, ConnectionType>, 4> distances = {
    std::make_pair( adore::math::distance_2d( from_points.front(), to_points.front() ), START_TO_START ),
    std::make_pair( adore::math::distance_2d( from_points.front(), to_points.back() ), START_TO_END ),
    std::make_pair( adore::math::distance_2d( from_points.back(), to_points.front() ), END_TO_START ),
    std::make_pair( adore::math::distance_2d( from_points.back(), to_points.back() ), END_TO_END )
  };

  auto min_it = std::min_element( distances.begin(), distances.end(), []( const auto& a, const auto& b ) { return a.first < b.first; } );

  return *min_it;
}

void
MapLoader::set_quadtree_bounds( Map& map, const std::vector<adore::r2s::BorderDataR2SR>& standard_lines,
                                const std::vector<adore::r2s::BorderDataR2SL>& lane_boundaries )
{
  // Initialize min/max values to extreme bounds
  auto [x_min, x_max] = std::make_pair( std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest() );
  auto [y_min, y_max] = std::make_pair( std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest() );

  // Lambda to update min/max for a given range
  auto update_min_max = [&]( const auto& x_range, const auto& y_range ) {
    x_min = std::min( x_min, *std::min_element( x_range.begin(), x_range.end() ) );
    x_max = std::max( x_max, *std::max_element( x_range.begin(), x_range.end() ) );
    y_min = std::min( y_min, *std::min_element( y_range.begin(), y_range.end() ) );
    y_max = std::max( y_max, *std::max_element( y_range.begin(), y_range.end() ) );
  };

  // Update bounds using
  for( const auto& line : standard_lines )
    update_min_max( line.x, line.y );
  for( const auto& line : lane_boundaries )
    update_min_max( line.x, line.y );

  // Apply margin and set boundaries
  map.quadtree.boundary.x_min = x_min - 100;
  map.quadtree.boundary.x_max = x_max + 100;
  map.quadtree.boundary.y_min = y_min - 100;
  map.quadtree.boundary.y_max = y_max + 100;
}

void
MapLoader::set_quadtree_bounds( Map& map, const odr::OpenDriveMap& xodr_map )
{
  // Initialize min/max values to extreme bounds
  auto [x_min, x_max] = std::make_pair( std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest() );
  auto [y_min, y_max] = std::make_pair( std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest() );

  for( const auto& [id, road] : xodr_map.id_to_road )
  {
    auto start_pt = road.get_surface_pt( 0, 0 );           // this is start of road point as vec3D
    auto end_pt   = road.get_surface_pt( road.length, 0 ); // this is end of road as vec3D
    x_min         = std::min( x_min, std::min( start_pt[0], end_pt[0] ) );
    x_max         = std::max( x_max, std::min( start_pt[0], end_pt[0] ) );
    y_min         = std::min( y_min, std::min( start_pt[1], end_pt[1] ) );
    y_max         = std::max( y_max, std::min( start_pt[1], end_pt[1] ) );
  }
  // Apply margin and set boundaries
  map.quadtree.boundary.x_min = x_min - 100;
  map.quadtree.boundary.x_max = x_max + 100;
  map.quadtree.boundary.y_min = y_min - 100;
  map.quadtree.boundary.y_max = y_max + 100;
}

Map
MapLoader::load_from_xodr_file( const std::string& filename, bool ignore_non_driving )
{
  Map               adore_road_map;
  odr::OpenDriveMap xodr_map( filename );
  set_quadtree_bounds( adore_road_map, xodr_map );
  std::unordered_map<odr::LaneKey, size_t> lane_mapping;

  size_t lane_id_counter = 0;
  size_t road_id_counter = 0;
  // Lambda functions for generating unique IDs
  auto generate_lane_id = [&lane_id_counter]() -> size_t { return ++lane_id_counter; };
  auto generate_road_id = [&road_id_counter]() -> size_t { return ++road_id_counter; };

  const double mesh_eps = 1.0;

  for( const auto& [xodr_road_id, xodr_road] : xodr_map.id_to_road )
  {
    for( const auto& [lanesec_s, lanesec] : xodr_road.s_to_lanesection )
    {
      auto road_type = xodr_road.s_to_type.find( lanesec_s ) == xodr_road.s_to_type.end() ? "town" : xodr_road.s_to_type.at( lanesec_s );
      Road adore_road( xodr_road_id + " s: " + std::to_string( lanesec_s ), generate_road_id(), road_type, xodr_road.left_hand_traffic );

      for( const auto& [lane_id, lane] : lanesec.id_to_lane )
      {
        if( lane.id == 0 || ( ignore_non_driving && lane.type != "driving" ) )
          continue;

        auto lane_mesh = xodr_road.get_lane_mesh( lane, mesh_eps );
        generate_lane_id();

        auto [inner_border, outer_border] = xodr_mesh_to_borders( lane_mesh, lane_id, lanesec_s );

        std::shared_ptr<Lane> adore_lane_ptr = std::make_shared<Lane>( inner_border, outer_border, lane_id_counter, adore_road.id,
                                                                       lane.id > 0 );

        adore_lane_ptr->set_type( lane.type, adore_road.category );

        lane_mapping[lane.key] = adore_lane_ptr->id;

        adore_road.lanes.insert( adore_lane_ptr );

        // TODO set road/lane categories

        for( const auto& p : adore_lane_ptr->borders.center.interpolated_points )
        {
          adore_road_map.quadtree.insert( p );
        }
        adore_road_map.lanes[adore_lane_ptr->id] = adore_lane_ptr;
      }
      adore_road_map.roads[adore_road.id] = adore_road;
    }
  }

  auto xodr_routing_graph = xodr_map.get_routing_graph();

  for( const auto& edge : xodr_routing_graph.edges )
  {
    if( edge.from.lane_id == 0 || edge.to.lane_id == 0 )
      continue;
    if( lane_mapping.find( edge.from ) == lane_mapping.end() || lane_mapping.find( edge.to ) == lane_mapping.end() )
      continue;
    Connection connection;
    connection.from_id         = lane_mapping.at( edge.from );
    connection.to_id           = lane_mapping.at( edge.to );
    connection.weight          = edge.weight;
    connection.connection_type = END_TO_START;
    adore_road_map.lane_graph.add_connection( connection );
  }
  return adore_road_map;
}

std::pair<Border, Border>
MapLoader::xodr_mesh_to_borders( const odr::Mesh3D& mesh, size_t lane_id, double s0 )
{
  Border inner;
  Border outer;
  for( size_t i = 0; i < mesh.vertices.size(); ++i )
  {
    MapPoint map_point( mesh.vertices[i][0], mesh.vertices[i][1], lane_id );
    map_point.s = mesh.st_coordinates[i][0] - s0;
    if( i % 2 == 0 )
      outer.points.push_back( map_point );
    else
      inner.points.push_back( map_point );
  }
  return { inner, outer };
};

} // namespace map
} // namespace adore
