/**
 * @file input_helper.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-12-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <advent_of_code_2019/adventlib/input_helper.h>

using namespace std;

namespace adventlib
{

InputHelper::InputHelper(string file_path)
{
    // Open the file and ensure it exists
    string full_path = ros::package::getPath("advent_of_code_2019") + "/" + file_path;
    file_ = ifstream{full_path, ios::in};
    if(!file_)
    {
        throw runtime_error("InputHelper: File not found. File path: " + full_path);
    }

    // Read the file line by line and store in vector
    string s;
    while(getline(file_, s))
    {
        input_vector_.push_back(s);
    }
}

InputHelper::~InputHelper()
{

}

vector<string> InputHelper::getInputVector()
{
    return input_vector_;
}

vector<int> InputHelper::convertStringVectorToIntVector(const vector<string> &string_vector)
{
    vector<int> rtn_vector;
    for(auto s : string_vector)
    {
        rtn_vector.push_back(stoi(s));
    }
    return rtn_vector;
}

vector<double> InputHelper::convertStringVectorToDoubleVector(const vector<string> &string_vector)
{
    vector<double> rtn_vector;
    for(auto s : string_vector)
    {
        rtn_vector.push_back(stod(s));
    }
    return rtn_vector;
}

vector<string> InputHelper::parseDelimitedStringToVector(string delimited_string, string delimiter)
{
    vector<string> rtn_vector;

    // Search for instances of delimiter
    vector<size_t> delimiter_positions;
    size_t index = 0;

    while(index < delimited_string.size())
    {
        size_t delimiter_pos = delimited_string.find(delimiter, index);
        if(delimiter_pos != string::npos)
        {
            delimiter_positions.push_back(delimiter_pos);
            index = delimiter_pos + 1;
        }
        else break;
    }

    // Pull sub strings out from between delimiters
    for(size_t i = 0; i <= delimiter_positions.size(); i++)
    {
        // If we are on the first delimiter
        if(i == 0)
        {
            size_t end = delimiter_positions.at(i);
            rtn_vector.push_back(delimited_string.substr(0, end));
        }
        // If we are on the last delimiter
        else if(i == delimiter_positions.size())
        {
            size_t start = delimiter_positions.at(i - 1) + 1;
            rtn_vector.push_back(delimited_string.substr(start));
        }
        else
        {
            size_t start = delimiter_positions.at(i - 1) + 1;
            size_t end = delimiter_positions.at(i);
            size_t len = end - start;
            rtn_vector.push_back(delimited_string.substr(start, len));
        } 
    }
    return rtn_vector;
}



} // namespace advent_helper