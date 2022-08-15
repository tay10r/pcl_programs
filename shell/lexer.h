#pragma once

#include <optional>
#include <string_view>

#include <cstddef>

enum class TokenKind
{
  Comment,
  Space,
  Newline,
  Id,
  Int,
  Symbol
};

struct Position final
{
  std::size_t line = 1;

  std::size_t column = 1;
};

struct Token final
{
  TokenKind kind;

  std::string_view data;

  Position position;
};

class Lexer final
{
public:
  Lexer(std::string_view data, std::size_t line);

  bool done() const { return offset_ >= data_.size(); }

  std::optional<Token> lex();

private:
  Token produce(TokenKind kind, std::size_t size);

  char peek(std::size_t relative_offset) const
  {
    const std::size_t absolute_offset = offset_ + relative_offset;

    if (absolute_offset < data_.size())
      return data_[absolute_offset];
    else
      return 0;
  }

private:
  std::string_view data_;

  Position position_;

  std::size_t offset_ = 0;
};
