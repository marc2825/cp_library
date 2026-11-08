#pragma once
#include <cstddef>
#include <stdexcept>
#include <string>
namespace cp {
enum class ParseErrorKind {
    expected_operand,
    unexpected_token,
    expected_closing,
    invalid_escape,
    invalid_range,
    depth_limit,
    node_limit,
    unknown_identifier,
    division_by_zero,
    negative_exponent,
    exponent_limit
};
class ParseError : public std::invalid_argument {
    ParseErrorKind kind_;
    std::size_t position_;

  public:
    ParseError(ParseErrorKind kind, std::size_t position, const std::string &message)
        : std::invalid_argument(message + " at byte " + std::to_string(position)), kind_(kind),
          position_(position) {}
    ParseErrorKind kind() const noexcept { return kind_; }
    std::size_t position() const noexcept { return position_; }
};
struct ParseLimits {
    std::size_t max_nodes = 1000000, max_depth = 256;
};
} // namespace cp
