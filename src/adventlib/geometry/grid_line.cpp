#include <advent_of_code_2019/adventlib/geometry/grid_line.h>

namespace adventlib
{
namespace geometry
{

GridLine::GridLine(GridCoord start_coord, GridCoord end_coord)
: start_coord_(start_coord), end_coord_(end_coord)
{
    if (start_coord.x == end_coord_.x)
        alignment_ = GridLineAlignment::VERTICAL;
    else if (start_coord_.y == end_coord_.y)
        alignment_ = GridLineAlignment::HORIZONTAL;
    else
        throw std::invalid_argument("GridLine: Start and End coordinates do not form a Vertical or Horizontal line.");
}

GridLine::~GridLine()
{
    
}

int GridLine::getLength()
{
    return GridCoord::distanceBetween(&start_coord_, &end_coord_);
}

GridCoord GridLine::getLowerCoord() const
{
    if (alignment_ == GridLineAlignment::HORIZONTAL)
    {
        return (start_coord_.x < end_coord_.x) ? start_coord_ : end_coord_;
    }
    else
    {
        return (start_coord_.y < end_coord_.y) ? start_coord_ : end_coord_;
    }
}

GridCoord GridLine::getHigherCoord() const
{
    if (alignment_ == GridLineAlignment::HORIZONTAL)
    {
        return (start_coord_.x > end_coord_.x) ? start_coord_ : end_coord_;
    }
    else
    {
        return (start_coord_.y > end_coord_.y) ? start_coord_ : end_coord_;
    }
}

bool GridLine::getIntersection(const GridLine *line1, const GridLine *line2, GridCoord *out_coord)
{
    // If these lines are parallel
    if (line1->getAlignment() == line2->getAlignment())
        return false;

    // Identify horizontal and vertical lines
    const GridLine *vert_line;
    const GridLine *horz_line;
    if (line1->getAlignment() == GridLineAlignment::HORIZONTAL)
    {
        horz_line = line1;
        vert_line = line2;
    }
    else
    {
        horz_line = line2;
        vert_line = line1;
    }

    // Check for intersection
    if (horz_line->getStartCoord().y < vert_line->getHigherCoord().y &&
        horz_line->getStartCoord().y > vert_line->getLowerCoord().y &&
        vert_line->getStartCoord().x < horz_line->getHigherCoord().x &&
        vert_line->getStartCoord().x > horz_line->getLowerCoord().x)
    {
        out_coord->x = vert_line->getStartCoord().x;
        out_coord->y = horz_line->getStartCoord().y;
        return true;
    }
    return false;
}

} // namespace geometry
} // namespace adventlib