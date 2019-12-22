#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <advent_of_code_2019/adventlib/input_helper.h>

using namespace std;
using namespace adventlib;

int main(int argc, char **argv)
{
    std::cout << "Advent of Code 2019 - Day 4 Part 1" << std::endl;
    ros::init(argc, argv, "day4");

    // Input ranges
    int low_input = 387638;
    int high_input = 919123;

    // Vector of accepted inputs
    vector<int> accepted_inputs_part1, accepted_inputs_part2;

    // Iterate through the range of inputs
    for (int i = low_input; i <= high_input; i++)
    {
        bool success = true;

        // Helper values
        string num = to_string(i);
        int len = num.length();

        // Test for 6 digit number
        if (i < 100000 || i > 999999)
            success = false;

        // Test for adjacent number match
        int match_count = 0;
        for (int y = 0; y < len - 1; y++)
        {
            if (num.at(y) == num.at(y + 1))
                match_count++;
        }
        if (match_count == 0)
            success = false;

        // Test for non-decreasing numbers
        for (int y = 0; y < len - 1; y++)
        {
            if (num.at(y) > num.at(y + 1))
                success = false;
        }

        // If all criteria met, add to vector
        if (success)
            accepted_inputs_part1.push_back(i);

        // Check for part 2 criteria
        if (success)
        {
            // Test for non-adjecent grouping
            int new_match_count = 0;
            for (int y = 0; y < len - 1; y++)
            {
                if (num.at(y) == num.at(y + 1))
                {
                    if (y == 0)
                    {
                        if (num.at(y) != num.at(y + 2))
                            new_match_count++;
                    }
                    else if (y == len - 2)
                    {
                        if (num.at(y) != num.at(y - 1))
                            new_match_count++;
                    }
                    else
                    {
                        if (num.at(y) != num.at(y - 1) && num.at(y) != num.at(y + 2))
                            new_match_count++;
                    }
                }
            }
            if (new_match_count > 0)
                accepted_inputs_part2.push_back(i);
        }
    }

    cout << "Accepted inputs part 1: " << accepted_inputs_part1.size() << endl;

    std::cout << "Advent of Code 2019 - Day 4 Part 2" << std::endl;
    cout << "Accepted inputs part 2: " << accepted_inputs_part2.size() << endl;
}