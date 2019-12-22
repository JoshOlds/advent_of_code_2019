#include <advent_of_code_2019/adventlib/geometry/grid_coord.h>

namespace adventlib
{
namespace geometry
{

GridCoord::GridCoord()
{
    x = 0;
    y = 0;
}

GridCoord::GridCoord(int x, int y)
{
    this->x = x;
    this->y = y;
}

int GridCoord::distanceBetween(const GridCoord *coord1, const GridCoord *coord2)
{
   int x = abs(coord1->x - coord2->x); 
   int y = abs(coord1->y - coord2->y);
   return x + y;
}

GridCoord::~GridCoord()
{
    
}

} // namespace geometry
} // namespace adventlib  