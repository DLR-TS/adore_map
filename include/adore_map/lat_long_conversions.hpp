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
 *    Andrew Koener
 *    Marko Mizdrak
 ********************************************************************************/
#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace adore
{
namespace map
{


// Function to execute a shell command and capture its output
std::string execute_shell_command( const std::string& command );

// Function to convert UTM coordinates to Latitude and Longitude
std::vector<double> convert_utm_to_lat_lon( double utm_x, double utm_y, int utm_zone, const std::string& utm_zone_letter );
std::vector<double> convert_utm_to_lat_lon_python( double utm_x, double utm_y, int utm_zone, const std::string& utm_zone_letter );


// Function to convert Latitude and Longitude UTM coordinates
std::vector<double> convert_lat_lon_to_utm( double lat, double lon );
std::vector<double> convert_lat_lon_to_utm_python( double lat, double lon );
} // namespace map
} // namespace adore
