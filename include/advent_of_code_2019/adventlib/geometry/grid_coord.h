#pragma once

// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>

namespace adventlib
{
namespace geometry
{

/// A 2D coordinate that falls on an integer grid, including an X and Y value
class GridCoord
{
public:
    /// Default constructs a Coord2D with 0 X and Y values.
    GridCoord();

    /// Constructs a Coord2D with given X & Y values
    GridCoord(int x, int y);

    /// Destructs this Coord2D object
    ~GridCoord();

    /// Gets the distance between two GridCoords
    static int distanceBetween(const GridCoord *coord1, const GridCoord *coord2);

    /// The X value of this 2D coordinate
    int x;

    /// The Y value of this 2D coordinate
    int y;
};

} // namespace geometry
} // namespace adventlib