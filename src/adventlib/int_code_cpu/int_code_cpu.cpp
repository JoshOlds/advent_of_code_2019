#include <advent_of_code_2019/adventlib/int_code_cpu/int_code_cpu.h>

using namespace std;

namespace adventlib
{
namespace int_code_cpu
{

IntCodeCPU::IntCodeCPU(std::vector<int> initial_memory)
{
    memory_ = initial_memory;
    instruction_pointer_ = 0;
    halt_signal_ = false;
}

IntCodeCPU::~IntCodeCPU()
{
}

void IntCodeCPU::run()
{
    while (instruction_pointer_ < memory_.size() - 1 && halt_signal_ == false)
    {
        executeOperation(instruction_pointer_);
    }
}

void IntCodeCPU::executeOperation(int address)
{
    // Get the operation value (last two digits of the int code)
    int op_value = getMemoryValue(address);
    int digit_low = op_value % 10;
    int digit_high = ((op_value % 100) - digit_low) / 10;
    int op = digit_low + (digit_high * 10);

    switch (op)
    {
    case 1:
        additionOp(address);
        break;

    case 2:
        multiplicationOp(address);
        break;

    case 3:
        inputOp(address);
        break;

    case 4:
        outputOp(address);
        break;

    case 5:
        jumpIfTrueOp(address);
        break;

    case 6:
        jumpIfFalseOp(address);
        break;

    case 7:
        lessThanOp(address);
        break;

    case 8:
        equalsOp(address);
        break;

    case 99:
        haltOp(address);
        break;

    default:
        cerr << "IntCodeCPU: Bad instruction called: " << op << " at memory location: " << address << ". Halting program." << endl;
        halt_signal_ = true;
    }
}

PARAMETER_MODE IntCodeCPU::getParameterMode(int operation_address, int parameter_index)
{
    if (parameter_index < 1)
        throw std::invalid_argument("IntCodeCPU: Parameter index must be 1 or greater.");

    int value = getMemoryValue(operation_address);

    // We want the value starting at the thousandths place and incrementing based on parameter index
    return PARAMETER_MODE((value / (10 * int(pow(10, parameter_index)))) % 10);
}

int IntCodeCPU::getParameter(int operation_address, int parameter_index)
{
    if (parameter_index < 1)
        throw std::invalid_argument("IntCodeCPU: Parameter index must be 1 or greater.");

    PARAMETER_MODE mode = getParameterMode(operation_address, parameter_index);
    int value = getMemoryValue(operation_address + parameter_index);

    switch (mode)
    {
    case PARAMETER_MODE::POSITION:
        return getMemoryValue(value);
        break;

    case PARAMETER_MODE::IMMEDIATE:
        return value;
        break;

    default:
        throw std::invalid_argument("IntCodeCPU: Invalid parameter mode passed to getParameter().");
    }
}

void IntCodeCPU::additionOp(int address)
{
    int parameter1 = getParameter(address, 1);
    int parameter2 = getParameter(address, 2);
    int parameter3 = getMemoryValue(address + 3);

    writeMemoryValue(parameter3, parameter1 + parameter2);
    instruction_pointer_ += 4;
}

void IntCodeCPU::multiplicationOp(int address)
{
    int parameter1 = getParameter(address, 1);
    int parameter2 = getParameter(address, 2);
    int parameter3 = getMemoryValue(address + 3);

    writeMemoryValue(parameter3, parameter1 * parameter2);
    instruction_pointer_ += 4;
}

void IntCodeCPU::inputOp(int address)
{
    int value = 0;
    cout << "IntCodeCPU: Please enter an input: ";
    cin >> value;

    int parameter1 = getMemoryValue(address + 1);
    writeMemoryValue(parameter1, value);
    instruction_pointer_ += 2;
}

void IntCodeCPU::outputOp(int address)
{
    int parameter1 = getParameter(address, 1);
    cout << parameter1 << endl;
    instruction_pointer_ += 2;
}

void IntCodeCPU::jumpIfTrueOp(int address)
{
    int parameter1 = getParameter(address, 1);
    int parameter2 = getParameter(address, 2);

    if (parameter1 != 0)
        instruction_pointer_ = parameter2;
    else
        instruction_pointer_ += 3;
}

void IntCodeCPU::jumpIfFalseOp(int address)
{
    int parameter1 = getParameter(address, 1);
    int parameter2 = getParameter(address, 2);

    if (parameter1 == 0)
        instruction_pointer_ = parameter2;
    else
        instruction_pointer_ += 3;
}

void IntCodeCPU::lessThanOp(int address)
{
    int parameter1 = getParameter(address, 1);
    int parameter2 = getParameter(address, 2);
    int parameter3 = getMemoryValue(address + 3);

    if (parameter1 < parameter2)
        writeMemoryValue(parameter3, 1);
    else
        writeMemoryValue(parameter3, 0);

    instruction_pointer_ += 4;
}

void IntCodeCPU::equalsOp(int address)
{
    int parameter1 = getParameter(address, 1);
    int parameter2 = getParameter(address, 2);
    int parameter3 = getMemoryValue(address + 3);

    if (parameter1 == parameter2)
        writeMemoryValue(parameter3, 1);
    else
        writeMemoryValue(parameter3, 0);

    instruction_pointer_ += 4;
}

void IntCodeCPU::haltOp(int address)
{
    halt_signal_ = true;
    instruction_pointer_ += 1;
}

} // namespace int_code_cpu
} // namespace adventlib