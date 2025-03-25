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
 *    Mikkel Skov Maarssø
 ********************************************************************************/

#pragma once
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace adore
{
namespace r2s
{

// Data structure definitions for R2SL and R2SR
struct BorderDataR2SL
{
  int                 id;
  int                 parent_id                 = 0;
  int                 datasource_description_id = 0;
  std::string         material;
  std::string         linetype;
  std::vector<double> x;
  std::vector<double> y;
};

struct BorderDataR2SR
{
  int                 id;
  std::string         streetname;
  int                 successor_id              = 0;
  int                 predecessor_id            = 0;
  int                 datasource_description_id = 0;
  std::string         turn;
  std::string         category;
  bool                oneway = false;
  std::string         linetype;
  std::vector<double> x;
  std::vector<double> y;
};

// Utility function for splitting strings by a delimiter
std::vector<std::string> split_line( const std::string& line, char delimiter = ',' );

// Functions to parse and create BorderData objects from attributes
BorderDataR2SL parse_border_data_r2sl( const std::vector<std::string>& attributes );
BorderDataR2SR parse_border_data_r2sr( const std::vector<std::string>& attributes );

// Load functions to parse .r2sl and .r2sr files into BorderData structures
std::vector<BorderDataR2SL> load_border_data_from_r2sl_file( const std::string& file_name );
std::vector<BorderDataR2SR> load_border_data_from_r2sr_file( const std::string& file_name );

// Print utility functions for debugging
void print_string( const std::string& string_to_print );
void print_string_array( const std::vector<std::string>& string_vector_to_print );
void print_border_data_r2sl( const BorderDataR2SL& data_to_print );
void print_border_data_r2sr( const BorderDataR2SR& data_to_print );

// Template functions for converting BorderDataR2SL and BorderDataR2SR into a combined data format
template<typename BorderDataCombined>
std::shared_ptr<BorderDataCombined>
border_data_from_r2sr_data( const BorderDataR2SR& border )
{
  auto border_data                         = std::make_shared<BorderDataCombined>();
  border_data->DataBaseId                  = border.id;
  border_data->parent_id                   = -1;
  border_data->left_neighbor_id            = -1;
  border_data->right_neighbor_id           = -1;
  border_data->TypeDescription.isReference = true;
  border_data->TypeDescription.isOneWay    = border.oneway;
  for( size_t i = 0; i < border.x.size(); ++i )
  {
    border_data->points.emplace_back( border.x[i], border.y[i] );
  }
  return border_data;
}

template<typename BorderDataCombined>
std::shared_ptr<BorderDataCombined>
border_data_from_r2sl_data( const BorderDataR2SL& border )
{
  auto border_data                        = std::make_shared<BorderDataCombined>();
  border_data->DataBaseId                 = border.id + 1000000;
  border_data->parent_id                  = border.parent_id;
  border_data->left_neighbor_id           = -1;
  border_data->right_neighbor_id          = -1;
  border_data->TypeDescription.isDrivable = ( border.linetype == "drivin" || border.linetype == "driving" );
  for( size_t i = 0; i < border.x.size(); ++i )
  {
    border_data->points.emplace_back( border.x[i], border.y[i] );
  }
  return border_data;
}

// Test function for parser
void test_border_set_parser_functions();

} // namespace r2s
} // namespace adore
