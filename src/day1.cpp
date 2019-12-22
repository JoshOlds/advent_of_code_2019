#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <advent_of_code_2019/adventlib/input_helper.h>

using namespace adventlib;

int main(int argc, char **argv)
{
    std::cout << "Advent of Code 2019 - Day 1 Part 1" << std::endl;

    InputHelper input{"puzzle_inputs/day1.txt"};

    // Get the puzzle input as ints
    std::vector<int> module_masses = input.convertStringVectorToIntVector(input.getInputVector());

    // Calculate the total mass
    int total_fuel_required = 0;
    for(auto mass : module_masses)
    {
        total_fuel_required += (mass / 3) - 2;
    }
    std::cout << "Total fuel required (Part 1): " << total_fuel_required << std::endl;

    // Calculate again, but recursively calculate the fuel required for the mass of the fuel
    total_fuel_required = 0;
    for(auto mass : module_masses)
    {
        int temp_mass = mass;
        while(temp_mass > 0)
        {
            int fuel_required = (temp_mass / 3) - 2;
            if(fuel_required > 0) 
                total_fuel_required += fuel_required;
            temp_mass = fuel_required;
        }
    }
    std::cout << "Total fuel required, including fuel for fuel mass (Part 2): " << total_fuel_required << std::endl;

}