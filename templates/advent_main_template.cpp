#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <advent_of_code_2019/adventlib/input_helper.h>
#include <advent_of_code_2019/adventlib/int_code_cpu.h>

using namespace std;
using namespace adventlib;

int main(int argc, char **argv)
{
    std::cout << "Advent of Code 2019 - Day XX Part Y" << std::endl;
    ros::init(argc, argv, "dayX");
    
    InputHelper input_helper{"puzzle_inputs/dayX.txt"};

    // Get the input data lines as a vector
    vector<string> input_vector = input_helper.getInputVector();

}