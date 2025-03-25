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
#include <iostream>
#include <memory>
#include <optional>

#include "adore_math/distance.h"

template<typename Point>
class Quadtree
{
public:

  Quadtree() {};

  // Boundary for this node (a simple square region)
  struct Boundary
  {
    double x_min, x_max, y_min, y_max;

    // Check if a point lies within this boundary
    template<typename QueryPoint>
    bool
    contains( const QueryPoint& point ) const
    {
      return ( point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max );
    }

    // Check if this boundary overlaps with another rectangular boundary
    bool
    intersects( const Boundary& range ) const
    {
      return !( range.x_min > x_max || range.x_max < x_min || range.y_min > y_max || range.y_max < y_min );
    }

    // Calculate the shortest distance from a point to the boundary (used for pruning)
    template<typename QueryPoint>
    double
    distance_to_point( const QueryPoint& point ) const
    {
      double dx = std::max( { x_min - point.x, 0.0, point.x - x_max } );
      double dy = std::max( { y_min - point.y, 0.0, point.y - y_max } );
      return std::sqrt( dx * dx + dy * dy );
    }

    // Check if this boundary intersects with a circle
    bool
    intersects_circle( double center_x, double center_y, double radius ) const
    {
      // Compute the closest point on the rectangle to the circle's center
      double closest_x = std::clamp( center_x, x_min, x_max );
      double closest_y = std::clamp( center_y, y_min, y_max );

      // Compute the distance from the circle's center to this closest point
      double distance = std::hypot( closest_x - center_x, closest_y - center_y );

      // If the distance is less than or equal to the radius, the circle intersects the boundary
      return distance <= radius;
    }
  };

  // Constructor for Quadtree node
  Quadtree( const Boundary& boundary, size_t capacity ) :
    boundary( boundary ),
    capacity( capacity ),
    divided( false )
  {}

  // Insert a point into the quadtree
  bool
  insert( const Point& point )
  {
    if( !boundary.contains( point ) )
    {
      return false; // Point is out of this node's boundary
    }

    if( points.size() < capacity )
    {
      points.push_back( point );
      return true;
    }

    // Need to subdivide and redistribute points
    if( !divided )
    {
      subdivide();
    }

    // Now insert the new point into appropriate child
    return ( northwest->insert( point ) || northeast->insert( point ) || southwest->insert( point ) || southeast->insert( point ) );
  }

  // Query all points within a range
  void
  query( const Boundary& range, std::vector<Point>& found ) const
  {
    if( !boundary.intersects( range ) )
    {
      return; // Range does not intersect this node, return
    }

    for( const auto& point : points )
    {
      if( range.contains( point ) )
      {
        found.push_back( point );
      }
    }

    // Recursively check children if divided
    if( divided )
    {
      northwest->query( range, found );
      northeast->query( range, found );
      southwest->query( range, found );
      southeast->query( range, found );
    }
  }

  // Query all points within a given radius from a center point (circular range)
  template<typename QueryPoint>
  void
  query_range( const QueryPoint& center, double radius, std::vector<Point>& found ) const
  {
    // If the query circle does not intersect this node's boundary, return early
    if( !boundary.intersects_circle( center.x, center.y, radius ) )
    {
      return;
    }

    // Check all points in this node
    for( const auto& point : points )
    {
      double distance = std::hypot( point.x - center.x, point.y - center.y );
      if( distance <= radius )
      {
        found.push_back( point );
      }
    }

    // Recursively check children if divided
    if( divided )
    {
      northwest->query_range( center, radius, found );
      northeast->query_range( center, radius, found );
      southwest->query_range( center, radius, found );
      southeast->query_range( center, radius, found );
    }
  }

  // Find the nearest point to the query point

  template<typename QueryPoint>
  std::optional<Point>
  get_nearest_point(
    const QueryPoint& query_point, double& min_dist,
    // default: accept all points
    const std::function<bool( const Point& )>& filter = []( const Point& ) { return true; } ) const
  {
    std::optional<Point> nearest_point = std::nullopt;

    // Check all points in this node
    for( const auto& point : points )
    {
      // Skip any point that fails the user-supplied filter
      if( !filter( point ) )
      {
        continue;
      }

      double dist = adore::math::distance_2d( point, query_point );
      if( dist < min_dist )
      {
        min_dist      = dist;
        nearest_point = point;
      }
    }

    // Recursively check children if this node is subdivided
    if( divided )
    {
      // Create a list of quadrants with their distances to the query point
      std::vector<std::pair<double, const Quadtree*>> quadrants = {
        { northwest->boundary.distance_to_point( query_point ), northwest.get() },
        { northeast->boundary.distance_to_point( query_point ), northeast.get() },
        { southwest->boundary.distance_to_point( query_point ), southwest.get() },
        { southeast->boundary.distance_to_point( query_point ), southeast.get() }
      };

      // Sort quadrants by distance to the query point
      std::sort( quadrants.begin(), quadrants.end(), []( const auto& a, const auto& b ) { return a.first < b.first; } );

      // Recursively search quadrants that might contain a closer point
      for( const auto& [dist_to_boundary, quadrant] : quadrants )
      {
        // If the quadrant boundary is still within the current min_dist, it might have a closer point
        if( dist_to_boundary < min_dist )
        {
          auto child_nearest = quadrant->get_nearest_point( query_point, min_dist, filter );
          if( child_nearest )
          {
            nearest_point = child_nearest;
          }
        }
        else
        {
          // Prune search if the quadrant is definitely farther than our current best
          break;
        }
      }
    }

    return nearest_point;
  }

  Boundary boundary;
  size_t   capacity = 10;

private:


  std::vector<Point> points;
  bool               divided = false;

  // Children of the quadtree
  std::shared_ptr<Quadtree<Point>> northwest = nullptr;
  std::shared_ptr<Quadtree<Point>> northeast = nullptr;
  std::shared_ptr<Quadtree<Point>> southwest = nullptr;
  std::shared_ptr<Quadtree<Point>> southeast = nullptr;

  // Subdivide the current node into four smaller nodes
  void
  subdivide()
  {
    double x_mid = ( boundary.x_min + boundary.x_max ) / 2;
    double y_mid = ( boundary.y_min + boundary.y_max ) / 2;

    // Create the four child quadrants
    northwest = std::make_shared<Quadtree<Point>>( Boundary{ boundary.x_min, x_mid, y_mid, boundary.y_max }, capacity );
    northeast = std::make_shared<Quadtree<Point>>( Boundary{ x_mid, boundary.x_max, y_mid, boundary.y_max }, capacity );
    southwest = std::make_shared<Quadtree<Point>>( Boundary{ boundary.x_min, x_mid, boundary.y_min, y_mid }, capacity );
    southeast = std::make_shared<Quadtree<Point>>( Boundary{ x_mid, boundary.x_max, boundary.y_min, y_mid }, capacity );

    divided = true;

    // Redistribute points into children
    for( const auto& p : points )
    {
      // Insert point into appropriate child node
      bool inserted = ( northwest->insert( p ) || northeast->insert( p ) || southwest->insert( p ) || southeast->insert( p ) );
      if( !inserted )
      {
        std::cerr << "subdivision problems - point not in any of sub quads" << std::endl;
        // Handle error: point should have been inserted into a child
        // This should not happen if boundaries are correctly defined
      }
    }
    points.clear(); // Clear points from the parent node
  }
};
