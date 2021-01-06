/******************************************************************************/
/*
  File:   closestpair.cpp
  Author: Braxton DeHate
  Brief:
    This file contains the implementation for a solution to the "Closest Pair"
    problem in O(nlogn) time using the divide and conquer method.
*/
/******************************************************************************/
#include "closestpair.h"
#include <algorithm> // sort, min
#include <limits> // numeric_limits<float>::max
#include <cmath> // sqrt
#include <iostream> // ostream, istream
#include <vector> // vector
#include <stdexcept> // runtime_error

//////////////// Prototypes ////////////////////////////////////////////////////

float closestPairDAC ( std::vector< Point >::const_iterator begin,
  std::vector<Point>::const_iterator end);

std::vector<Point>::const_iterator divide(
  std::vector< Point >::const_iterator begin,
  std::vector<Point>::const_iterator end,
  bool onY);

////////////////////////////////////////////////////////////////////////////////

// Public facing closest pair implementation.
// Param - points: Vector of points with which to find the closest pair.
// Return: Distance between the closest two points in the vector.
//  If function is called with 0 or 1 points, it returns float max.
float closestPair ( std::vector< Point > const& points )
{
	int size = points.size();

	if (size <= 1)
    return std::numeric_limits<float>::max();

  std::vector<Point> newPoints(points);

  // Sort points in order of:
  //  1) y position, ascending
  //  2) x position, ascending
  std::sort(newPoints.begin(), newPoints.end(),
    [](Point const& p1, Point const& p2)
  {
    if (p1.y < p2.y) return true;
    if (p1.y > p2.y) return false;
    if (p1.x < p2.x) return true;
    return false;
  });

  // Call recursive divide and conquer closest pair solver
	return closestPairDAC( newPoints.begin(), newPoints.end() );
}

// Returns an iterator to the item following an appropriate divide in.
// Finds a roughly halfway division point where the elements before and after
// the division point differ on the specified axis.
// Param - begin, end: Iterators specifying the range on which to operate.
// Param - onY: True will make sure there is no overlap on the Y axis. False
//  will use X.
// Return: Iterator to the item following an appropriate divide in.
std::vector<Point>::const_iterator divide(
  std::vector< Point >::const_iterator begin,
  std::vector<Point>::const_iterator end,
  bool onY)
{
  int size = end - begin;

  // Get midpoint of array as iterator
  auto midpoint = begin + size / 2;
  auto division = midpoint;
  while (division != end)
  {
    // Check if the item preceding division is different.
    if (onY ?
      division->y != (division - 1)->y :
      division->x != (division - 1)->x)
    {
      return division;
    }
    ++division;
  }
  // Failed to find division by searching upwards. Search downwards next.
  division = midpoint;
  while (division != begin)
  {
    // Check if the item preceding division is different.
    if (onY ?
      division->y != (division - 1)->y :
      division->x != (division - 1)->x)
    {
      return division;
    }
    --division;
  }
  return end;
}

// Helper function which returns the distance between two Points.
// Param - p1, p2: Points to compute the distance between.
// Return: Distance between the input points.
float distance(Point const& p1, Point const& p2)
{
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
    (p1.y - p2.y) * (p1.y - p2.y));
}

// Divide and conquer implementation of a solution to the closest pair problem.
// Separate from the public implementation in order to not recurse over the
// setup code.
// Param - begin: Iterator to the first element to consider.
// Param - end: Iterator to one past the last element to consider.
float closestPairDAC (std::vector< Point >::const_iterator begin,
  std::vector<Point>::const_iterator end)
{
	int size = end - begin;

	if (size <= 1) return std::numeric_limits<float>::max();

  bool onY = true;
  auto division = divide(begin, end, true);

  // Case: Could not divide point set on the Y axis.
  if(division == end)
  {
    // Try again on the X axis.
    division = divide(begin, end, false);
    // If it could not be divided on x or y, the set has only identical points.
    if(division == end)
    {
      return 0.0f;
    }
    onY = false;
  }

  // Operate on left and right divisions recursively
  float leftDist = closestPairDAC(begin, division);
  float rightDist = closestPairDAC(division, end);
  float minDist = std::min(leftDist, rightDist);

  // Check if the closest pair is between the two halves.
  for(auto left_iter = begin; left_iter != division; ++left_iter)
  {
    // The speed increase of divide and conquer comes from the early exit here,
    // We generally only have to compare points very close to the division.
    if (onY && (division->y - left_iter->y) > minDist)
      continue;
    if (!onY && (division->x - left_iter->x) > minDist)
      continue;

    for (auto right_iter = division; right_iter != end; ++right_iter)
    {
      float dist = distance(*left_iter, *right_iter);
      if (minDist > dist)
        minDist = dist;
    }
  }

	return minDist;
}

