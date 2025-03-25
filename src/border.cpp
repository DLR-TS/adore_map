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
#include "adore_map/border.hpp"

namespace adore
{
namespace map
{

// Implement operator<< for Border
std::ostream&
operator<<( std::ostream& os, Border& border )
{
  os << "Border Points:\n";
  for( const auto& point : border.points )
  {
    os << point << "\n";
  }

  os << "Interpolated Points:\n";
  for( const auto& point : border.interpolated_points )
  {
    os << point << "\n";
  }

  return os;
}

double
Border::get_length() const
{
  return length;
}

double
Border::compute_length()
{
  length = 0.0;
  for( size_t i = 0; i < points.size() - 1; i++ )
  {
    length += adore::math::distance_2d( points[i], points[i + 1] );
  }
  return length;
}

void
Border::preprocess_points_for_spline( double angle_threshold )
{
  if( points.size() < 3 )
  {
    // Not enough points to have a sharp turn
    return;
  }

  std::vector<MapPoint> new_points;
  new_points.push_back( points[0] ); // Add the first point


  for( size_t i = 1; i < points.size() - 1; ++i )
  {
    const MapPoint& prev = points[i - 1];
    const MapPoint& curr = points[i];
    const MapPoint& next = points[i + 1];

    // Vectors for the segments
    double vx1 = curr.x - prev.x;
    double vy1 = curr.y - prev.y;
    double vx2 = next.x - curr.x;
    double vy2 = next.y - curr.y;

    // Calculate the angle between the segments
    double dot_product = vx1 * vx2 + vy1 * vy2;
    double mag1        = std::hypot( vx1, vy1 );
    double mag2        = std::hypot( vx2, vy2 );

    if( mag1 == 0.0 || mag2 == 0.0 )
    {
      new_points.push_back( curr );
      continue; // Skip if segment length is zero
    }

    double cos_theta = dot_product / ( mag1 * mag2 );
    cos_theta        = std::clamp( cos_theta, -1.0, 1.0 ); // Clamp to valid range
    double angle     = std::acos( cos_theta );

    if( angle >= angle_threshold )
    {
      // Sharp turn detected

      // Distance to offset along the segments
      const double offset_distance = std::min( mag1, mag2 ) * 0.1; // Adjust factor as needed

      // Calculate unit vectors for incoming and outgoing segments
      double ux1 = vx1 / mag1;
      double uy1 = vy1 / mag1;
      double ux2 = vx2 / mag2;
      double uy2 = vy2 / mag2;

      // Create a point slightly before the current point along the incoming segment
      MapPoint before_turn;
      before_turn.x         = curr.x - offset_distance * ux1;
      before_turn.y         = curr.y - offset_distance * uy1;
      before_turn.parent_id = curr.parent_id;
      before_turn.s         = curr.s - offset_distance; // Adjust s if necessary

      // Create a point slightly after the current point along the outgoing segment
      MapPoint after_turn;
      after_turn.x         = curr.x + offset_distance * ux2;
      after_turn.y         = curr.y + offset_distance * uy2;
      after_turn.parent_id = curr.parent_id;
      after_turn.s         = curr.s + offset_distance; // Adjust s if necessary

      // Add the before_turn point
      new_points.push_back( before_turn );

      // Add the current point
      new_points.push_back( curr );

      // Add the after_turn point
      new_points.push_back( after_turn );
    }
    else
    {
      // No sharp turn, add the current point
      new_points.push_back( curr );
    }
  }

  new_points.push_back( points.back() ); // Add the last point

  // Replace the original points with the new ones
  points = std::move( new_points );
}

void
Border::initialize_spline()
{
  if( points.size() >= 2 )
  {
    constexpr static double MAX_ANGLE = 0.1;
    preprocess_points_for_spline( MAX_ANGLE );
    spline = BorderSpline( points );
  }
  else
  {
    spline.reset(); // Ensure spline is empty if not initialized
  }
}

double
Border::find_nearest_s( const MapPoint& point )
{
  if( !spline )
  {
    throw std::runtime_error( "Reference line spline is not initialized." );
  }

  // Find initial guess by sampling
  const int num_samples  = 5;
  double    min_distance = std::numeric_limits<double>::max();
  double    best_s       = 0.0;

  for( int i = 0; i <= num_samples; ++i )
  {
    double   s_sample     = get_length() * i / num_samples;
    MapPoint spline_point = spline->get_point_at_s( s_sample );
    double   distance     = adore::math::squared_distance_2d( spline_point, point );
    if( distance < min_distance )
    {
      min_distance = distance;
      best_s       = s_sample;
    }
  }

  double s = best_s;

  // Newton-Raphson parameters
  const double tol            = 1e-3;
  const int    max_iterations = 100;
  int          iteration      = 0;

  // gradient descent to find closest point
  while( iteration < max_iterations )
  {
    MapPoint spline_point = spline->get_point_at_s( s );
    double   x_s          = spline_point.x;
    double   y_s          = spline_point.y;

    double dx_ds  = spline->get_x_derivative_at_s( s );
    double dy_ds  = spline->get_y_derivative_at_s( s );
    double ddx_ds = spline->get_x_second_derivative_at_s( s );
    double ddy_ds = spline->get_y_second_derivative_at_s( s );

    double F       = ( x_s - point.x ) * dx_ds + ( y_s - point.y ) * dy_ds;
    double F_prime = dx_ds * dx_ds + ( x_s - point.x ) * ddx_ds + dy_ds * dy_ds + ( y_s - point.y ) * ddy_ds;

    if( std::abs( F_prime ) < 1e-12 )
    {
      break;
    }

    double s_new = s - F / F_prime;

    // Clamp s to valid range
    s_new = std::clamp( s_new, 0.0, get_length() );

    if( std::abs( s_new - s ) < tol )
    {
      s = s_new;
      break;
    }

    s = s_new;
    ++iteration;
  }

  return s;
}

// Reparameterize the border so that its s values correspond to the reference line's s values
void
Border::reparameterize_based_on_reference( Border& reference_line )
{
  double THRESHOLD = 5.0;

  // Compute s-values for the first and last points
  points.front().s = reference_line.find_nearest_s( points.front() );
  points.back().s  = reference_line.find_nearest_s( points.back() );


  // Reverse points if the back's s-value is less than the front's
  if( points.back().s < points.front().s )
    std::reverse( points.begin(), points.end() );

  if( points.front().s < THRESHOLD )
    points.front().s = 0.0;
  if( reference_line.get_length() - points.back().s < THRESHOLD )
    points.back().s = reference_line.get_length();

  // Compute the total length of the border
  double total_length = compute_length(); // Computes and sets the 'length' member variable

  // Compute the s-length between the first and last points on the reference line
  double s_length = points.back().s - points.front().s;

  // Compute cumulative lengths and interpolate s-values for intermediate points
  double cumulative_length = 0.0;

  for( size_t i = 1; i < points.size(); ++i )
  {
    // Compute the segment length between consecutive points
    double segment_length  = adore::math::distance_2d( points[i - 1], points[i] );
    cumulative_length     += segment_length;

    // Compute the ratio of cumulative length to total length
    double length_ratio = cumulative_length / total_length;

    // Interpolate the s-value based on the length ratio
    points[i].s = points.front().s + length_ratio * s_length;
  }

  // Sort points based on new s-values
  std::sort( points.begin(), points.end(), []( const MapPoint& a, const MapPoint& b ) { return a.s < b.s; } );
}

Border
Border::make_clipped( double s_start, double s_end ) const
{
  // Ensure s_start and s_end are within the border's range
  double border_s_start = points.front().s;
  double border_s_end   = points.back().s;

  double clipped_s_start = std::max( s_start, border_s_start );
  double clipped_s_end   = std::min( s_end, border_s_end );

  // If there's no overlap or the interval is too small, return an empty border
  if( clipped_s_start >= clipped_s_end - 1e-6 )
    return Border();

  // Create a new border
  Border clipped_border;

  // Insert the start point
  MapPoint start_point = get_interpolated_point( clipped_s_start );
  start_point.s        = clipped_s_start;
  clipped_border.points.push_back( start_point );

  // Efficiently find points within [clipped_s_start, clipped_s_end]
  auto lower = std::lower_bound( points.begin(), points.end(), clipped_s_start,
                                 []( const MapPoint& point, double s ) { return point.s < s; } );

  auto upper = std::upper_bound( points.begin(), points.end(), clipped_s_end,
                                 []( double s, const MapPoint& point ) { return s < point.s; } );

  // Insert points in the range [lower, upper)
  for( auto it = lower; it != upper; ++it )
  {
    clipped_border.points.push_back( *it );
  }

  // Insert the end point
  MapPoint end_point = get_interpolated_point( clipped_s_end );
  end_point.s        = clipped_s_end;
  clipped_border.points.push_back( end_point );

  // Initialize spline if enough points
  if( clipped_border.points.size() >= 2 )
  {
    clipped_border.initialize_spline();
  }
  else
  {
    // Return an empty border if not enough points
    return Border();
  }

  return clipped_border;
}

MapPoint
Border::get_interpolated_point( double s ) const
{
  adore::map::MapPoint interpolated_point;

  if( spline )
  {
    interpolated_point           = spline->get_point_at_s( s );
    interpolated_point.parent_id = points[0].parent_id;
  }
  else
  {

    // Linear interpolation using the original points
    if( points.empty() && interpolated_points.empty() )
      throw std::invalid_argument( "Border is empty." );

    auto& interpolation_points = ( points.size() > 0 ) ? points : interpolated_points;

    if( interpolation_points.size() == 1 )
      return interpolated_points[0];

    if( s <= interpolation_points.front().s )
      return interpolation_points.front();
    if( s >= interpolation_points.back().s )
      return interpolation_points.back();

    // Find the two interpolation_points between which s lies
    for( size_t i = 1; i < interpolation_points.size(); ++i )
    {
      if( s < interpolation_points[i].s )
      {
        const MapPoint& p1 = interpolation_points[i - 1];
        const MapPoint& p2 = interpolation_points[i];

        // Compute the interpolation factor (t) in [0, 1]
        double t = ( s - p1.s ) / ( p2.s - p1.s );

        // Interpolate between p1 and p2
        interpolated_point.x         = p1.x + t * ( p2.x - p1.x );
        interpolated_point.y         = p1.y + t * ( p2.y - p1.y );
        interpolated_point.s         = s;
        interpolated_point.parent_id = p1.parent_id; // Assuming parent_id is taken from the first point

        return interpolated_point;
      }
    }
    // If s is beyond the total length due to numerical errors
    return points.back();
  }

  interpolated_point.s = s;

  return interpolated_point;
}

void
Border::interpolate_border( const std::vector<double>& s_values )
{
  interpolated_points.clear();
  interpolated_points.reserve( s_values.size() );

  for( const auto& s : s_values )
  {
    try
    {
      auto point = get_interpolated_point( s );
      interpolated_points.push_back( point );
    }
    catch( const std::exception& e )
    {
      std::cerr << "Interpolation error at s=" << s << ": " << e.what() << std::endl;
    }
  }
}

void
process_center( Borders& borders )
{
  if( borders.inner.interpolated_points.size() != borders.outer.interpolated_points.size() )
    throw std::invalid_argument( "Borders need equal size inner and outer to process center." );

  borders.center.interpolated_points.clear();
  borders.center.interpolated_points.reserve( borders.inner.interpolated_points.size() );

  for( size_t i = 0; i < borders.inner.interpolated_points.size(); ++i )
  {
    const MapPoint& inner_point = borders.inner.interpolated_points[i];
    const MapPoint& outer_point = borders.outer.interpolated_points[i];

    double cx = ( inner_point.x + outer_point.x ) / 2.0;
    double cy = ( inner_point.y + outer_point.y ) / 2.0;

    MapPoint center_point( cx, cy, outer_point.parent_id );
    center_point.s = inner_point.s; // s values are aligned

    borders.center.interpolated_points.push_back( center_point );
  }
}

void
interpolate_borders( Borders& borders, double spacing_s )
{
  if( borders.inner.points.size() < 2 || borders.outer.points.size() < 2 )
  {
    // Not enough points to compute centerline
    return;
  }

  borders.inner.initialize_spline();
  borders.outer.initialize_spline();

  double inner_length = borders.inner.compute_length();
  double outer_length = borders.outer.compute_length();

  // Determine the number of samples based on the longer border
  double max_length  = std::max( inner_length, outer_length );
  size_t num_samples = static_cast<size_t>( max_length / spacing_s ) + 1;

  if( num_samples < 2 )
    num_samples = 2;

  // Create normalized parameter t_values ranging from 0 to 1
  std::vector<double> t_values( num_samples );
  for( size_t i = 0; i < num_samples; ++i )
  {
    t_values[i] = static_cast<double>( i ) / ( num_samples - 1 );
  }

  // Compute s values for inner and outer borders
  std::vector<double> s_inner_values( num_samples );
  std::vector<double> s_outer_values( num_samples );
  std::vector<double> s_values( num_samples );
  for( size_t i = 0; i < num_samples; ++i )
  {
    s_inner_values[i] = t_values[i] * inner_length;
    s_outer_values[i] = t_values[i] * outer_length;
    s_values[i]       = t_values[i] * max_length;
  }

  // Interpolate borders
  borders.inner.interpolate_border( s_inner_values );
  borders.outer.interpolate_border( s_outer_values );

  // Assign the s values to the interpolated points
  for( size_t i = 0; i < num_samples; ++i )
  {
    borders.inner.interpolated_points[i].s = s_values[i];
    borders.outer.interpolated_points[i].s = s_values[i];
  }
}

void
set_parent_id( Borders& borders, size_t parent_id )
{
  for( auto& p : borders.inner.points )
  {
    p.parent_id = parent_id;
  }
  for( auto& p : borders.outer.points )
  {
    p.parent_id = parent_id;
  }
  for( auto& p : borders.center.points )
  {
    p.parent_id = parent_id;
  }
  for( auto& p : borders.inner.interpolated_points )
  {
    p.parent_id = parent_id;
  }
  for( auto& p : borders.outer.interpolated_points )
  {
    p.parent_id = parent_id;
  }
  for( auto& p : borders.center.interpolated_points )
  {
    p.parent_id = parent_id;
  }
}

} // namespace map
} // namespace adore
