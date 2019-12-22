#pragma once

// std type includes
#include <string>
#include <vector>
#include <stdexcept>

// File access and parsing
#include <fstream>

// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>

namespace adventlib
{

/**
 * @brief The InputHelper class provides functions to open, read, and parse input files. The functions are designed to work with the formatting
 * of the Advent of Code puzzle inputs.
 * 
 */
class InputHelper
{
public:

    /**
     * @brief Construct a new Input Helper object
     * 
     * @param file_path The path to the file to parse.
     */
    InputHelper(std::string file_path);
    ~InputHelper();

    /// Returns the InputVector, which is a newline seperated vector parsed from the input file.
    std::vector<std::string> getInputVector();

    /// Converts a vector of strings to a vector of integers
    static std::vector<int> convertStringVectorToIntVector(const std::vector<std::string> &string_vector);

    /// Converts a vector of strings to a vector of doubles
    static std::vector<double> convertStringVectorToDoubleVector(const std::vector<std::string> &string_vector);

    /// Parses a character delimited string and returns a vector of strings split by the delimiter value.
    static std::vector<std::string> parseDelimitedStringToVector(std::string delimited_string, std::string delimiter);

private:

    /// The FileHelper's input file
    std::ifstream file_;

    /// The vector obtained from the input file, seperated by newlines
    std::vector<std::string> input_vector_;

};

} // namespace::adventlib
