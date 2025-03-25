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
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "adore_math/distance.h"

#include "map_point.hpp"

namespace adore
{

namespace map
{
class BorderSpline
{
private:

  std::vector<double> distances_;             // Cumulative distances for spline interpolation
  std::vector<double> a_x_, b_x_, c_x_, d_x_; // Spline coefficients for x
  std::vector<double> a_y_, b_y_, c_y_, d_y_; // Spline coefficients for y

  // Eigen-based tridiagonal system solver for spline coefficients
  Eigen::VectorXd
  solve_tridiagonal_system( const Eigen::MatrixXd& A, const Eigen::VectorXd& rhs ) const
  {
    Eigen::VectorXd result = A.fullPivLu().solve( rhs );
    if( !result.allFinite() )
    {
      throw std::runtime_error( "Solution contains NaNs or Infs, indicating an unstable system." );
    }
    return result;
  }

  // Find the interval index for a given s using binary search
  size_t
  find_interval( double s ) const
  {
    // Ensure s is within bounds
    s = std::clamp( s, distances_.front(), distances_.back() );

    // Use std::lower_bound to perform binary search
    auto   it = std::lower_bound( distances_.begin(), distances_.end(), s );
    size_t i  = std::distance( distances_.begin(), it );

    if( i == 0 )
      return 0;
    if( i >= distances_.size() )
      return distances_.size() - 2;
    return i - 1;
  }

  // Evaluate cubic polynomial using Horner's method
  double
  evaluate_cubic( double a, double b, double c, double d, double ds ) const
  {
    // Evaluate a + ds*(b + ds*(c + ds*d))
    return a + ds * ( b + ds * ( c + ds * d ) );
  }

public:

  BorderSpline() = default;

  // Initialize spline from points
  BorderSpline( const std::vector<MapPoint>& points )
  {
    initialize( points );
  }

  void
  initialize( const std::vector<MapPoint>& points )
  {
    if( points.size() < 2 )
    {
      throw std::invalid_argument( "Insufficient points for spline calculation." );
    }

    // Compute distances while skipping duplicate points and collect unique x and y values
    distances_.clear();
    std::vector<double> x_values, y_values;
    distances_.push_back( 0.0 );
    x_values.push_back( points[0].x );
    y_values.push_back( points[0].y );

    for( size_t i = 1; i < points.size(); ++i )
    {
      double distance = adore::math::distance_2d( points[i - 1], points[i] );

      // Skip duplicate points if distance is zero
      if( distance == 0.0 )
      {
        continue;
      }

      distances_.push_back( distances_.back() + distance );
      x_values.push_back( points[i].x );
      y_values.push_back( points[i].y );
    }

    // Ensure we have enough unique points after removing duplicates
    if( distances_.size() < 2 )
    {
      throw std::runtime_error( "Insufficient unique points for spline calculation." );
    }

    // Compute spline coefficients for x and y based on unique points
    compute_spline_coefficients( x_values, a_x_, b_x_, c_x_, d_x_ );
    compute_spline_coefficients( y_values, a_y_, b_y_, c_y_, d_y_ );
  }

  void
  compute_spline_coefficients( const std::vector<double>& values, std::vector<double>& a, std::vector<double>& b, std::vector<double>& c,
                               std::vector<double>& d )
  {
    size_t n = values.size() - 1;
    a.resize( n );
    b.resize( n );
    c.resize( n + 1 );
    d.resize( n );

    // Create tridiagonal matrix A and right-hand side vector rhs
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero( n + 1, n + 1 );
    Eigen::VectorXd rhs( n + 1 );

    // Fill the interior points for the tridiagonal system
    for( size_t i = 1; i < n; ++i )
    {
      double h1     = distances_[i] - distances_[i - 1];
      double h2     = distances_[i + 1] - distances_[i];
      A( i, i - 1 ) = h1;
      A( i, i )     = 2.0 * ( h1 + h2 );
      A( i, i + 1 ) = h2;
      rhs( i )      = 3.0 * ( ( values[i + 1] - values[i] ) / h2 - ( values[i] - values[i - 1] ) / h1 );
    }

    // Natural boundary conditions: Set the first and last row of A
    A( 0, 0 ) = 1.0;
    A( n, n ) = 1.0;
    rhs( 0 )  = 0.0;
    rhs( n )  = 0.0;

    // Solve for c coefficients
    Eigen::VectorXd c_vec = solve_tridiagonal_system( A, rhs );
    for( size_t i = 0; i <= n; ++i )
    {
      c[i] = c_vec[i];
    }

    // Calculate b and d coefficients based on c values
    for( size_t i = 0; i < n; ++i )
    {
      double h = distances_[i + 1] - distances_[i];
      d[i]     = ( c[i + 1] - c[i] ) / ( 3.0 * h );
      b[i]     = ( values[i + 1] - values[i] ) / h - ( 2.0 * c[i] + c[i + 1] ) * h / 3.0;
      a[i]     = values[i];
    }
  }

  // Method to get interpolated point at a given distance s
  MapPoint
  get_point_at_s( double s ) const
  {
    size_t i = find_interval( s );

    double ds = s - distances_[i];
    double x  = evaluate_cubic( a_x_[i], b_x_[i], c_x_[i], d_x_[i], ds );
    double y  = evaluate_cubic( a_y_[i], b_y_[i], c_y_[i], d_y_[i], ds );

    return MapPoint( x, y, 0 );
  }

  // Method to compute the first derivative of x at s
  double
  get_x_derivative_at_s( double s ) const
  {
    size_t i  = find_interval( s );
    double ds = s - distances_[i];
    // x'(s) = b + 2c ds + 3d ds^2
    return b_x_[i] + ds * ( 2.0 * c_x_[i] + 3.0 * d_x_[i] * ds );
  }

  // Method to compute the first derivative of y at s
  double
  get_y_derivative_at_s( double s ) const
  {
    size_t i  = find_interval( s );
    double ds = s - distances_[i];
    // y'(s) = b + 2c ds + 3d ds^2
    return b_y_[i] + ds * ( 2.0 * c_y_[i] + 3.0 * d_y_[i] * ds );
  }

  // Method to compute the second derivative of x at s
  double
  get_x_second_derivative_at_s( double s ) const
  {
    size_t i  = find_interval( s );
    double ds = s - distances_[i];
    // x''(s) = 2c + 6d ds
    return 2.0 * c_x_[i] + 6.0 * d_x_[i] * ds;
  }

  // Method to compute the second derivative of y at s
  double
  get_y_second_derivative_at_s( double s ) const
  {
    size_t i  = find_interval( s );
    double ds = s - distances_[i];
    // y''(s) = 2c + 6d ds
    return 2.0 * c_y_[i] + 6.0 * d_y_[i] * ds;
  }

  // Method to interpolate multiple points at once
  std::vector<MapPoint>
  get_points_at_s_values( const std::vector<double>& s_values ) const
  {
    std::vector<MapPoint> points;
    points.reserve( s_values.size() );
    for( double s : s_values )
    {
      points.push_back( get_point_at_s( s ) );
    }
    return points;
  }

  // Get the total length of the spline
  double
  get_total_length() const
  {
    return distances_.back();
  }
};


} // namespace map
} // namespace adore
