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
    std::cout << "Advent of Code 2019 - Day 2 Part 1" << std::endl;
    ros::init(argc, argv, "day2");

    InputHelper input_helper{"puzzle_inputs/day2.txt"};

    // Get the input data lines as a vector
    vector<string> input_vector = input_helper.getInputVector();

    // Parse the delimited string
    vector<string> codes = input_helper.parseDelimitedStringToVector(input_vector.at(0), ",");
    vector<int> int_codes = input_helper.convertStringVectorToIntVector(codes);

    // Set up IntCodeCPU
    IntCodeCPU cpu{int_codes};

    // Edit values as per instructions
    cpu.writeMemoryValue(1, 12);
    cpu.writeMemoryValue(2, 2);

    // Run and check the state of the cpu
    cpu.run();
    cout << "Int Code @ Index 0: " << cpu.getMemoryValue(0) << endl;

    // Part 2 -------------------------------------
    std::cout << "Advent of Code 2019 - Day 2 Part 2" << std::endl;

    // We are searching for a noun/verb combo that produces output 19690720 after program halt
    cout << "Searching for noun/verb combo that produces output 19690720..." << endl;
    int noun = 0;
    int verb = 0;
    bool done = false;

    for (noun = 0; noun < 100; noun++)
    {
        for (verb = 0; verb < 100; verb++)
        {
            // Reset the cpu
            cpu = IntCodeCPU{int_codes};
            // Write the noun/verb pair
            cpu.writeMemoryValue(1, noun);
            cpu.writeMemoryValue(2, verb);
            // Execute cpu
            try
            {
                cpu.run();
            }
            catch (const std::out_of_range &e)
            {
                
            }

            // Check output
            if (cpu.getMemoryValue(0) == 19690720)
            {
                done = true;
            }
            if (done)
                break;
        }
        if (done)
            break;
    }

    cout << "Noun/Verb combo that produces output 19690720 is: Noun: " << noun << " , Verb: " << verb << endl;
}