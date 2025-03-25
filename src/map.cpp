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
#include "adore_map/map.hpp"

#include "adore_map/helpers.hpp"

namespace adore
{
namespace map
{

double
Map::get_lane_speed_limit( size_t lane_id ) const
{
  // Use find() to search for the lane_id
  auto it = lanes.find( lane_id );
  if( it != lanes.end() )
  {
    const auto& lane        = it->second;
    double      speed_limit = lane->get_speed_limit();
    return speed_limit;
  }

  return 13.6;
}


} // namespace map
} // namespace adore
