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

#include <algorithm>
#include <optional>
#include <vector>

#include "adore_map/border_spline.hpp"
#include "adore_map/helpers.hpp"
#include "adore_map/map_point.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"

namespace adore
{
namespace map
{
struct Border
{

  // original points for the border
  std::vector<MapPoint> points;

  std::optional<BorderSpline> spline = std::nullopt;

  // interpolated points
  std::vector<MapPoint> interpolated_points;

  double length = 0.0;

  // Get the border length by summing all segment lengths
  double get_length() const;
  double compute_length();

  // inerpolate border at given s values
  void interpolate_border( const std::vector<double>& s_values );
  // initialize spline with points
  void initialize_spline();

  void preprocess_points_for_spline( double angle_threshold_degrees );

  // Get an interpolated point at a given s value
  MapPoint get_interpolated_point( double s ) const;

  double find_nearest_s( const MapPoint& point );

  Border make_clipped( double s_start, double s_send ) const;

  // Reparameterize the border so that its s values correspond to the reference line's s values
  void reparameterize_based_on_reference( Border& reference_line );

  void
  compute_s_values()
  {
    if( points.empty() )
      return;

    points[0].s = 0.0;
    for( size_t i = 1; i < points.size(); ++i )
    {
      double ds   = adore::math::distance_2d( points[i], points[i - 1] );
      points[i].s = points[i - 1].s + ds;
    }
  }

  friend std::ostream& operator<<( std::ostream& os, Border& border );
};

struct Borders

{
  Border inner;
  Border outer;
  Border center;
};

// in place interpolation of borders given a spacing of points
void interpolate_borders( Borders& borders, double spacing_s );

// Calculate center from inner and outer borders
void process_center( Borders& borders );

void set_parent_id( Borders& borders, size_t parent_id );


} // namespace map
} // namespace adore
