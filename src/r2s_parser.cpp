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
 *    Marko Mizdrak
 ********************************************************************************/
#include "adore_map/r2s_parser.h"

namespace adore
{
namespace r2s
{

// Helper function to parse LINESTRING and other fields

std::vector<std::string>
split_fields( const std::string& line )
{
  std::vector<std::string> fields;

  // Regular expression to match ID, LINESTRING, and other fields
  std::regex  re( R"((\d+),\"?LINESTRING \(([^)]+)\)\"?,(.*))" );
  std::smatch match;

  if( std::regex_match( line, match, re ) )
  {
    // Extract ID, LINESTRING, and remaining fields
    fields.push_back( match[1] ); // ID
    fields.push_back( match[2] ); // LINESTRING content (coordinates)

    // Process remaining fields manually by splitting on commas to handle empty fields
    std::string        remaining_fields = match[3];
    std::istringstream ss( remaining_fields );
    std::string        field;

    while( std::getline( ss, field, ',' ) )
    {
      field.erase( std::remove( field.begin(), field.end(), '"' ), field.end() ); // Remove quotes
      fields.push_back( field );
    }
  }
  else
  {
    std::cerr << "Unrecognized line format: " << line << std::endl;
  }
  return fields;
}

// Parse function for BorderDataR2SL with precise LINESTRING handling
BorderDataR2SL
parse_border_data_r2sl( const std::vector<std::string>& fields )
{
  BorderDataR2SL data;
  try
  {
    data.id = std::stoi( fields[0] );

    // Parse LINESTRING coordinates
    std::istringstream coords_stream( fields[1] );
    std::string        coord_pair;
    while( std::getline( coords_stream, coord_pair, ',' ) )
    {
      std::istringstream coord_stream( coord_pair );
      double             x, y;
      coord_stream >> x >> y;
      data.x.push_back( x );
      data.y.push_back( y );
    }

    // Parse remaining fields
    data.linetype                  = fields[fields.size() - 4];
    data.material                  = fields[fields.size() - 3];
    data.datasource_description_id = ( fields[fields.size() - 2] != "NULL" ) ? std::stoi( fields[fields.size() - 2] ) : 0;
    data.parent_id                 = ( fields[fields.size() - 1] != "NULL" ) ? std::stoi( fields[fields.size() - 1] ) : 0;
  }
  catch( const std::exception& e )
  {
    std::cerr << "Error parsing BorderDataR2SL: " << e.what() << std::endl;
  }
  return data;
}

// Similar parse function for BorderDataR2SR
BorderDataR2SR
parse_border_data_r2sr( const std::vector<std::string>& fields )
{
  BorderDataR2SR data;
  try
  {
    data.id = std::stoi( fields[0] );

    // Parse LINESTRING coordinates
    std::istringstream coords_stream( fields[1] );
    std::string        coord_pair;
    while( std::getline( coords_stream, coord_pair, ',' ) )
    {
      std::istringstream coord_stream( coord_pair );
      double             x, y;
      coord_stream >> x >> y;
      data.x.push_back( x );
      data.y.push_back( y );
    }

    // Parse remaining fields
    data.linetype                  = fields[fields.size() - 8];
    data.oneway                    = ( fields[fields.size() - 7] == "true" );
    data.category                  = fields[fields.size() - 6];
    data.turn                      = fields[fields.size() - 5];
    data.datasource_description_id = ( fields[fields.size() - 4] != "NULL" ) ? std::stoi( fields[fields.size() - 4] ) : 0;
    data.predecessor_id            = ( fields[fields.size() - 3] != "NULL" ) ? std::stoi( fields[fields.size() - 3] ) : 0;
    data.successor_id              = ( fields[fields.size() - 2] != "NULL" ) ? std::stoi( fields[fields.size() - 2] ) : 0;
    data.streetname                = fields[fields.size() - 1];
  }
  catch( const std::exception& e )
  {
    std::cerr << "Error parsing BorderDataR2SR: " << e.what() << std::endl;
  }
  return data;
}

// Load functions for R2SL and R2SR, handling line endings
std::vector<BorderDataR2SL>
load_border_data_from_r2sl_file( const std::string& file_name )
{

  std::string                 file_name_as_r2sl = file_name.substr( 0, file_name.size() - 1 ) + "l";
  std::vector<BorderDataR2SL> data_vector;
  std::ifstream               file( file_name_as_r2sl );
  if( !file )
  {
    std::cerr << "Failed to open file: " << file_name_as_r2sl << std::endl;
    return data_vector;
  }

  std::string line;
  std::getline( file, line ); // Skip header line

  while( std::getline( file, line ) )
  {
    if( !line.empty() && line.back() == '\r' )
    {
      line.pop_back(); // Remove carriage return for Windows-style line endings
    }
    auto fields = split_fields( line );
    if( fields.size() > 4 )
    {
      data_vector.push_back( parse_border_data_r2sl( fields ) );
    }
  }
  return data_vector;
}

std::vector<BorderDataR2SR>
load_border_data_from_r2sr_file( const std::string& file_name )
{
  std::vector<BorderDataR2SR> data_vector;
  std::ifstream               file( file_name );
  if( !file )
  {
    std::cerr << "Failed to open file: " << file_name << std::endl;
    return data_vector;
  }

  std::string line;
  std::getline( file, line ); // Skip header line

  while( std::getline( file, line ) )
  {
    if( !line.empty() && line.back() == '\r' )
    {
      line.pop_back(); // Remove carriage return for Windows-style line endings
    }
    auto fields = split_fields( line );
    if( fields.size() > 8 )
    {
      data_vector.push_back( parse_border_data_r2sr( fields ) );
    }
  }
  return data_vector;
}

} // namespace r2s
} // namespace adore
