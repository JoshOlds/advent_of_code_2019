#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <advent_of_code_2019/adventlib/input_helper.h>
#include <advent_of_code_2019/adventlib/int_code_cpu/int_code_cpu.h>

using namespace std;
using namespace adventlib;
using namespace int_code_cpu;

int main(int argc, char **argv)
{
    std::cout << "Advent of Code 2019 - Day 5 Part 1 & 2" << std::endl;
    ros::init(argc, argv, "day5");

    InputHelper input_helper{"puzzle_inputs/day5.txt"};

    // Get the input data lines as a vector
    vector<string> input_vector = input_helper.getInputVector();

    // Parse the delimited string
    vector<string> codes = input_helper.parseDelimitedStringToVector(input_vector.at(0), ",");
    vector<int> int_codes = input_helper.convertStringVectorToIntVector(codes);

    // Set up IntCodeCPU
    IntCodeCPU cpu{int_codes};

    cpu.run();
}