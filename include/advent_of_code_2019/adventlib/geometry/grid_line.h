#pragma once

// System includes
#include <stdexcept>

// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>

// Library Includes
#include <advent_of_code_2019/adventlib/geometry/grid_coord.h>

namespace adventlib
{
namespace geometry
{

/// A 2D Line segment that occupies an XY grid. Values are whole integers and lines may only be vertical or horizontal (no diagonal lines).
class GridLine
{
public:
    enum GridLineAlignment
    {
        HORIZONTAL,
        VERTICAL
    };

    /// Constructs a GridLine out of a start and end coordinate. Throws an exception if coordinates are invalid (form a diagonal line)
    GridLine(GridCoord start_coord, GridCoord end_coord);

    ~GridLine();

    /// Returns the start coordinate of this GridLine
    GridCoord getStartCoord() const { return start_coord_; }

    /// Returns the end coordinate of thie GridLine
    GridCoord getEndCoord() const { return end_coord_; }

    /// Get the lower value coordinate of this GridLine (based on if the line is horizontal or vertical)
    GridCoord getLowerCoord() const;

    /// Get the higher value coordinate of this GridLine (based on if the line is horizontal or vertical)
    GridCoord getHigherCoord() const;

    /// Gets the alignment (vertical or horizontal) of this GridLine
    GridLineAlignment getAlignment() const { return alignment_; }

    /// Returns the length of the line segment
    int getLength();

    /// Finds an intersection between two GridLines. Populates the out_coord param with the coordinate of the intersection (if found). Returns true on found intersection.
    static bool getIntersection(const GridLine *line1, const GridLine *line2, GridCoord *out_coord);

private:
    /// The start coordinate of this line segment
    const GridCoord start_coord_;
    /// The end coordinate of this line segment
    const GridCoord end_coord_;
    /// Specifies the alignment (horz or vert) of this GridLine.
    GridLineAlignment alignment_;
};

} // namespace geometry
} // namespace adventlib