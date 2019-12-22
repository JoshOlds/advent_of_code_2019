#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <advent_of_code_2019/adventlib/input_helper.h>
#include <advent_of_code_2019/adventlib/geometry/grid_line.h>

using namespace std;
using namespace adventlib;
using namespace geometry;

void wireFromCommands(const vector<string> *commands, vector<GridLine> *out_wire)
{
    GridCoord last_coord;
    for (string s : *commands)
    {
        int dist;
        switch (s.at(0))
        {
        case 'U':
            dist = stoi(s.substr(1));
            out_wire->push_back(GridLine{last_coord, GridCoord{last_coord.x, last_coord.y + dist}});
            last_coord = out_wire->back().getEndCoord();
            break;
        case 'D':
            dist = stoi(s.substr(1));
            out_wire->push_back(GridLine{last_coord, GridCoord{last_coord.x, last_coord.y - dist}});
            last_coord = out_wire->back().getEndCoord();
            break;
        case 'L':
            dist = stoi(s.substr(1));
            out_wire->push_back(GridLine{last_coord, GridCoord{last_coord.x - dist, last_coord.y}});
            last_coord = out_wire->back().getEndCoord();
            break;
        case 'R':
            dist = stoi(s.substr(1));
            out_wire->push_back(GridLine{last_coord, GridCoord{last_coord.x + dist, last_coord.y}});
            last_coord = out_wire->back().getEndCoord();
            break;
        }
    }
}

int main(int argc, char **argv)
{
    std::cout << "Advent of Code 2019 - Day 3 Part 1" << std::endl;
    ros::init(argc, argv, "day3");

    InputHelper input_helper1{"puzzle_inputs/day3_wire1.txt"};
    InputHelper input_helper2{"puzzle_inputs/day3_wire2.txt"};

    // Get the input data lines as a vector
    vector<string> input_vector1 = input_helper1.getInputVector();
    vector<string> input_vector2 = input_helper2.getInputVector();

    // Get vector of commands from comma delimited string
    vector<string> wire_1_commands = input_helper1.parseDelimitedStringToVector(input_vector1.at(0), ",");
    vector<string> wire_2_commands = input_helper2.parseDelimitedStringToVector(input_vector2.at(0), ",");

    // Create the wires our of GridLines
    vector<GridLine> wire1, wire2;
    wireFromCommands(&wire_1_commands, &wire1);
    wireFromCommands(&wire_2_commands, &wire2);

    // Search for intersections
    vector<GridCoord> intersections;
    for (auto line1 : wire1)
    {
        for (auto line2 : wire2)
        {
            GridCoord intersect;
            if (GridLine::getIntersection(&line1, &line2, &intersect))
                intersections.push_back(intersect);
        }
    }

    // Find the closest intersection
    int closest = 99999999;
    GridCoord origin;
    for (auto coord : intersections)
    {
        if (GridCoord::distanceBetween(&origin, &coord) < closest)
            closest = GridCoord::distanceBetween(&origin, &coord);
    }

    cout << "Closest intersection is at distance: " << closest << endl;

    // PART 2 -------------------------------------------------------
    std::cout << "Advent of Code 2019 - Day 3 Part 2" << std::endl;

    // Store intersections with their length
    vector<pair<GridCoord, int>> intersections_with_lengths;
    int len1 = 0;
    int len2 = 0;
    for (auto line1 : wire1)
    {
        for (auto line2 : wire2)
        {
            GridCoord intersect;
            if (GridLine::getIntersection(&line1, &line2, &intersect))
            {
                GridCoord temp1, temp2;
                temp1 = line1.getStartCoord();
                temp2 = line2.getStartCoord();
                int dist1 = GridCoord::distanceBetween(&temp1, &intersect);
                int dist2 = GridCoord::distanceBetween(&temp2, &intersect);
                intersections_with_lengths.push_back({pair<GridCoord, int>{intersect, len1 + len2 + dist1 + dist2}});
            }

            len2 += line2.getLength();
        }
        len1 += line1.getLength();
        len2 = 0;
    }

    // Find the shortest overall length
    int shortest = 99999999;
    for (auto i : intersections_with_lengths)
    {
        if (i.second < shortest)
            shortest = i.second;
    }

    cout << "Shortest length intersection is at distance: " << shortest << endl;
}