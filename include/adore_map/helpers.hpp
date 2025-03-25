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
#include "adore_map/border.hpp"
#include "adore_map/map_point.hpp"

namespace adore
{
namespace map
{

// Remove duplicate points from a vector of MapPoints based on s values
inline static void
remove_duplicate_points( std::vector<MapPoint>& points )
{
  auto last = std::unique( points.begin(), points.end(),
                           []( const MapPoint& a, const MapPoint& b ) { return std::abs( a.s - b.s ) < 1e-6; } );
  points.erase( last, points.end() );
}

} // namespace map
} // namespace adore
