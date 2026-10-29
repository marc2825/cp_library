#pragma once
#include "../basic_utility/big_integer.hpp"
#include "parse_error.hpp"
#include <climits>
#include <functional>
#include <map>
#include <string_view>
#include <vector>
namespace cp {
enum class ExpressionKind { integer, identifier, unary, binary, group };
struct ExpressionNode {
    ExpressionKind kind;
    std::size_t begin, end, token;
    int left = -1, right = -1;
    char operation = 0;
};
class ExpressionTree {
    std::string source_;
    std::vector<ExpressionNode> nodes_;
    int root_ = -1;
    friend ExpressionTree parse_expression(std::string, ParseLimits);

  public:
    const std::string &source() const { return source_; }
    const std::vector<ExpressionNode> &nodes() const { return nodes_; }
    int root() const { return root_; }
    std::string_view text(int node) const {
        const auto &n = nodes_.at(node);
        return std::string_view(source_).substr(n.begin, n.end - n.begin);
    }
};
// Recursive descent with precedence climbing. ^ is right associative; -2^2 is -(2^2).
inline ExpressionTree parse_expression(std::string source, ParseLimits limits = {}) {
    ExpressionTree tree;
    tree.source_ = std::move(source);
    const auto &s = tree.source_;
    std::size_t p = 0;
    const auto space = [&] {
        while (p < s.size() &&
               (s[p] == ' ' || s[p] == '\t' || s[p] == '\n' || s[p] == '\r' || s[p] == '\f' || s[p] == '\v'))
            ++p;
    };
    const auto digit = [](char c) { return c >= '0' && c <= '9'; };
    const auto letter = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; };
    const auto add = [&](ExpressionNode n) {
        if (tree.nodes_.size() >= limits.max_nodes || tree.nodes_.size() >= INT_MAX)
            throw ParseError(ParseErrorKind::node_limit, p, "expression node limit");
        tree.nodes_.push_back(n);
        return static_cast<int>(tree.nodes_.size() - 1);
    };
    const auto precedence = [](char c) {
        return c == '+' || c == '-' ? 1 : c == '*' || c == '/' || c == '%' ? 2 : c == '^' ? 4 : 0;
    };
    std::function<int(int, std::size_t)> expression = [&](int minimum, std::size_t depth) -> int {
        space();
        if (depth > limits.max_depth)
            throw ParseError(ParseErrorKind::depth_limit, p, "expression recursion limit");
        const auto begin = p;
        if (p == s.size())
            throw ParseError(ParseErrorKind::expected_operand, p, "expected operand");
        int left;
        if (s[p] == '+' || s[p] == '-') {
            const char op = s[p++];
            const int child = expression(3, depth + 1);
            left = add({ExpressionKind::unary, begin, tree.nodes_[child].end, begin, child, -1, op});
        } else if (s[p] == '(') {
            ++p;
            left = expression(1, depth + 1);
            space();
            if (p == s.size() || s[p] != ')')
                throw ParseError(ParseErrorKind::expected_closing, p, "expected )");
            ++p;
            left = add({ExpressionKind::group, begin, p, begin, left});
        } else if (digit(s[p])) {
            do {
                ++p;
            } while (p < s.size() && digit(s[p]));
            left = add({ExpressionKind::integer, begin, p, begin});
        } else if (letter(s[p])) {
            do {
                ++p;
            } while (p < s.size() && (letter(s[p]) || digit(s[p])));
            left = add({ExpressionKind::identifier, begin, p, begin});
        } else
            throw ParseError(ParseErrorKind::expected_operand, p, "expected operand");
        for (;;) {
            space();
            if (p == s.size())
                break;
            const char op = s[p];
            const int priority = precedence(op);
            if (priority < minimum)
                break;
            const auto token = p++;
            const int right = expression(priority + (op != '^'), depth + 1);
            left = add({ExpressionKind::binary, tree.nodes_[left].begin, tree.nodes_[right].end, token, left,
                        right, op});
        }
        return left;
    };
    tree.root_ = expression(1, 0);
    space();
    if (p != s.size())
        throw ParseError(ParseErrorKind::unexpected_token, p, "unexpected trailing input");
    return tree;
}
// Nodes are postorder: folding is iterative and may use any caller-supplied value type.
template <class T, class Literal, class Variable, class Unary, class Binary>
T evaluate_expression(const ExpressionTree &tree, Literal literal, Variable variable, Unary unary,
                      Binary binary) {
    if (tree.root() < 0)
        throw std::invalid_argument("empty expression tree");
    std::vector<T> values;
    values.reserve(tree.nodes().size());
    for (std::size_t i = 0; i < tree.nodes().size(); ++i) {
        const auto &n = tree.nodes()[i];
        if (n.kind == ExpressionKind::integer)
            values.push_back(literal(tree.text(static_cast<int>(i))));
        else if (n.kind == ExpressionKind::identifier)
            values.push_back(variable(tree.text(static_cast<int>(i))));
        else if (n.kind == ExpressionKind::group)
            values.push_back(values[n.left]);
        else if (n.kind == ExpressionKind::unary)
            values.push_back(unary(n.operation, values[n.left]));
        else
            values.push_back(binary(n.operation, values[n.left], values[n.right]));
    }
    return std::move(values[tree.root()]);
}
inline BigInt evaluate_integer_expression(const ExpressionTree &tree,
                                          const std::map<std::string, BigInt> &variables = {},
                                          std::uint64_t max_exponent = 1000000) {
    if (tree.root() < 0)
        throw std::invalid_argument("empty expression tree");
    std::vector<BigInt> values;
    values.reserve(tree.nodes().size());
    for (std::size_t i = 0; i < tree.nodes().size(); ++i) {
        const auto &n = tree.nodes()[i];
        BigInt value;
        if (n.kind == ExpressionKind::integer)
            value = BigInt(std::string(tree.text(static_cast<int>(i))));
        else if (n.kind == ExpressionKind::identifier) {
            auto it = variables.find(std::string(tree.text(static_cast<int>(i))));
            if (it == variables.end())
                throw ParseError(ParseErrorKind::unknown_identifier, n.token, "unknown identifier");
            value = it->second;
        } else if (n.kind == ExpressionKind::group)
            value = values[n.left];
        else if (n.kind == ExpressionKind::unary)
            value = n.operation == '-' ? -values[n.left] : values[n.left];
        else {
            const auto &a = values[n.left];
            const auto &b = values[n.right];
            switch (n.operation) {
            case '+':
                value = a + b;
                break;
            case '-':
                value = a - b;
                break;
            case '*':
                value = a * b;
                break;
            case '/':
            case '%':
                if (!b)
                    throw ParseError(ParseErrorKind::division_by_zero, n.token, "division by zero");
                value = n.operation == '/' ? a / b : a % b;
                break;
            case '^': {
                if (b < 0)
                    throw ParseError(ParseErrorKind::negative_exponent, n.token, "negative integer exponent");
                if (b > BigInt(max_exponent))
                    throw ParseError(ParseErrorKind::exponent_limit, n.token, "integer exponent limit");
                auto exponent = b.to_integral<std::uint64_t>();
                BigInt base = a;
                value = 1;
                for (; exponent; exponent >>= 1) {
                    if (exponent & 1)
                        value *= base;
                    if (exponent > 1)
                        base *= base;
                }
                break;
            }
            default:
                throw std::logic_error("invalid expression operation");
            }
        }
        values.push_back(std::move(value));
    }
    return std::move(values[tree.root()]);
}
} // namespace cp
