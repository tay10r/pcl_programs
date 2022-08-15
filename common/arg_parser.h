#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include <cstdint>

class ArgParser final
{
public:
  ArgParser(int argc, char** argv);

  std::string find_string(const std::string& option, const std::string& else_);

  std::uint32_t find_uint32(const std::string& option, std::uint32_t else_);

  float find_float(const std::string& option, float else_);

private:
  std::vector<std::string> args_;
};
