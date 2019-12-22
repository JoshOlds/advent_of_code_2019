#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace adventlib
{

namespace int_code_cpu
{

enum PARAMETER_MODE
{
    POSITION = 0,
    IMMEDIATE = 1
};

/**
 * @brief An 'IntCode' CPU designed to the specification of the Advent of Code challenge.
 * 
 */
class IntCodeCPU
{
public:
    /// Constructs an IntCodeCPU object from an initial memory value
    IntCodeCPU(std::vector<int> initial_memory);

    ~IntCodeCPU();

    /// Runs this int code cpu until a halt command is executed
    void run();

    /// Executes an operation at the memory address specified. Dispatches the correct operation based on memory value.
    void executeOperation(int address);


    /// Writes a value to the memory address specified
    void writeMemoryValue(int address, int value){ memory_[address] = value; }

    /// Returns a copied dump of the cpu's memory
    std::vector<int> getMemoryDump(){ return memory_; }

    /// Returns a copy of the value at the memory address specified
    int getMemoryValue(size_t address){ return memory_.at(address); }

private:

        /// Gets the operation mode of the specified parameter at the specified operation address
    PARAMETER_MODE getParameterMode(int operation_address, int parameter_index);

    /// Returns a parameter of an operation at the given address based on parameter index
    int getParameter(int operation_address, int parameter_index);

    /// Adds parameter 1 to parameter 2 and stores in parameter 3
    void additionOp(int address);

    /// Multiplies parameter 1 with parameter 2 and stores in parameter 3
    void multiplicationOp(int address);

    /// Takes an input from the user and saves it at parameter 1 position.
    void inputOp(int address);

    /// Takes a value from parameter 1 position and outputs it to the screen.
    void outputOp(int address);

    /// If the first parameter is non-zero, sets the instruction pointer to the value from the second parameter. Otherwise, it does nothing.
    void jumpIfTrueOp(int address);

    /// If the first parameter is zero, it sets the instruction pointer to the value from the second parameter. Otherwise, it does nothing.
    void jumpIfFalseOp(int address);

    /// If the first parameter is less than the second parameter, store 1 in the position given by the third parameter. Otherwise, store 0.
    void lessThanOp(int address);

    /// If the first parameter is equal to the second parameter, store 1 in the position given by the third parameter. Otherwise, store 0.
    void equalsOp(int address);

    /// Halts cpu operation
    void haltOp(int address);

    /// The memory of this cpu
    std::vector<int> memory_;

    /// The instruction pointer that points to the address of the current (next) command
    size_t instruction_pointer_;

    /// Signals a halt of the cpu operation
    bool halt_signal_;
};

} // namespace::int_code_cpu
} // namespace::advent
