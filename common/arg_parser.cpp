#include "arg_parser.h"

ArgParser::ArgParser(int argc, char** argv)
{
  for (int i = 1; i < argc; i++)
    args_.emplace_back(argv[i]);
}

std::string
ArgParser::find_string(const std::string& option, const std::string& else_)
{
  for (std::size_t i = 0; i < args_.size(); i++) {

    if (args_[i] != option)
      continue;

    args_.erase(args_.begin() + i);

    if (i >= args_.size())
      return else_;

    std::string value = std::move(args_[i]);

    args_.erase(args_.begin() + i);

    return value;
  }

  return else_;
}

std::uint32_t
ArgParser::find_uint32(const std::string& option, std::uint32_t else_)
{
  for (std::size_t i = 0; i < args_.size(); i++) {

    if (args_[i] != option)
      continue;

    args_.erase(args_.begin() + i);

    if (i >= args_.size())
      return else_;

    std::uint32_t value = std::stoul(args_[i]);

    args_.erase(args_.begin() + i);

    return value;
  }

  return else_;
}

float
ArgParser::find_float(const std::string& option, float else_)
{
  for (std::size_t i = 0; i < args_.size(); i++) {

    if (args_[i] != option)
      continue;

    args_.erase(args_.begin() + i);

    if (i >= args_.size())
      return else_;

    float value = std::stof(args_[i]);

    args_.erase(args_.begin() + i);

    return value;
  }

  return else_;
}
