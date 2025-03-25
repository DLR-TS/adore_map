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
#include <vector>

#include "adore_math/point.h"

namespace adore
{
namespace map
{


struct TrafficLight
{
  std::vector<adore::math::Point2d> control_points;

  enum TrafficLightState
  {
    GREEN,
    RED,
    UNKNOWN,
    AMBER
  } state;

  size_t id;
};

} // namespace map
} // namespace adore
