#include "lexer.h"

Lexer::Lexer(std::string_view data, std::size_t line)
  : data_(data)
  , position_{ line, 1 }
{
}

std::optional<Token>
Lexer::lex()
{
  if (offset_ >= data_.size())
    return std::nullopt;

  if (peek(0) == '\n')
    return produce(TokenKind::Newline, 1);
  else if ((peek(0) == '\r') && (peek(1) == '\n'))
    return produce(TokenKind::Newline, 2);

  if ((peek(0) == ' ') || (peek(1) == '\t'))
    return produce(TokenKind::Space, 1);

  return produce(TokenKind::Symbol, 1);
}

Token
Lexer::produce(TokenKind kind, std::size_t size)
{
  const Token token{ kind, data_.substr(offset_, size), position_ };

  for (std::size_t i = 0; i < size; i++) {
    if (peek(i) == '\n') {
      position_.line++;
      position_.column = 1;
    } else {
      position_.column++;
    }
  }

  return token;
}
