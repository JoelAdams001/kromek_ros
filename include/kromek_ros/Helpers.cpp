#include "Helpers.hpp"
#include <iostream>

std::string input(std::string_view prompt) {
    std::cout << prompt << ' ';
    std::string cmd;
    std::getline(std::cin, cmd);
    return cmd;
}