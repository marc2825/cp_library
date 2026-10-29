#pragma once
#include "automata.hpp"
#include "parse_error.hpp"
#include <array>
#include <functional>
namespace cp {
// Byte regex: literals, escapes, ., classes/ranges, concatenation, |, (), *, +, ?.
// Full-match semantics; empty alternatives denote epsilon. No backreferences or counted repetition.
inline NFA regex_to_nfa(const std::string &pattern, ParseLimits limits = {}) {
    if (!limits.max_nodes)
        throw ParseError(ParseErrorKind::node_limit, 0, "NFA state budget");
    NFA nfa(1, 256);
    std::size_t p = 0;
    struct Fragment {
        int first, last;
    };
    const auto node = [&] {
        if (static_cast<std::size_t>(nfa.size()) >= limits.max_nodes)
            throw ParseError(ParseErrorKind::node_limit, p, "NFA state budget");
        return nfa.add_state();
    };
    const auto empty = [&] {
        int a = node(), b = node();
        nfa.add_transition(a, -1, b);
        return Fragment{a, b};
    };
    const auto escaped = [&]() -> int {
        const auto begin = p;
        if (p == pattern.size())
            throw ParseError(ParseErrorKind::invalid_escape, p, "dangling escape");
        const unsigned char c = pattern[p++];
        if (c == 'n')
            return '\n';
        if (c == 'r')
            return '\r';
        if (c == 't')
            return '\t';
        if (c == '0')
            return 0;
        if (c != 'x')
            return c;
        int value = 0;
        for (int k = 0; k < 2; ++k) {
            if (p == pattern.size())
                throw ParseError(ParseErrorKind::invalid_escape, begin, "expected two hex digits");
            const char h = pattern[p++];
            const int d = h >= '0' && h <= '9'   ? h - '0'
                          : h >= 'a' && h <= 'f' ? h - 'a' + 10
                          : h >= 'A' && h <= 'F' ? h - 'A' + 10
                                                 : -1;
            if (d < 0)
                throw ParseError(ParseErrorKind::invalid_escape, p - 1, "invalid hex escape");
            value = value * 16 + d;
        }
        return value;
    };
    std::function<Fragment(std::size_t)> expression, concatenation, repeat, atom;
    atom = [&](std::size_t depth) -> Fragment {
        if (depth > limits.max_depth)
            throw ParseError(ParseErrorKind::depth_limit, p, "regex recursion limit");
        if (p == pattern.size())
            throw ParseError(ParseErrorKind::expected_operand, p, "expected regex atom");
        const auto at = p;
        const unsigned char c = pattern[p++];
        if (c == '(') {
            auto result = expression(depth + 1);
            if (p == pattern.size() || pattern[p] != ')')
                throw ParseError(ParseErrorKind::expected_closing, p, "expected )");
            ++p;
            return result;
        }
        std::array<bool, 256> chars{};
        if (c == '.')
            chars.fill(true);
        else if (c == '[') {
            bool negate = p < pattern.size() && pattern[p] == '^';
            if (negate)
                ++p;
            bool any = false;
            const auto character = [&]() {
                if (p == pattern.size())
                    throw ParseError(ParseErrorKind::expected_closing, p, "expected ]");
                const unsigned char x = pattern[p++];
                return x == '\\' ? escaped() : int(x);
            };
            while (p < pattern.size() && pattern[p] != ']') {
                int low = character(), high = low;
                any = true;
                if (p + 1 < pattern.size() && pattern[p] == '-' && pattern[p + 1] != ']') {
                    ++p;
                    high = character();
                    if (high < low)
                        throw ParseError(ParseErrorKind::invalid_range, p, "reversed character range");
                }
                for (int x = low; x <= high; ++x)
                    chars[x] = true;
            }
            if (p == pattern.size() || !any)
                throw ParseError(ParseErrorKind::expected_closing, p, "empty or unclosed class");
            ++p;
            if (negate)
                for (auto &x : chars)
                    x = !x;
        } else if (c == '\\')
            chars[escaped()] = true;
        else {
            if (c == '*' || c == '+' || c == '?' || c == ')' || c == '|' || c == '{' || c == '}' ||
                c == '^' || c == '$')
                throw ParseError(ParseErrorKind::unexpected_token, at,
                                 "unexpected or unsupported regex operator");
            chars[c] = true;
        }
        int a = node(), b = node();
        for (int x = 0; x < 256; ++x)
            if (chars[x])
                nfa.add_transition(a, x, b);
        return {a, b};
    };
    repeat = [&](std::size_t depth) {
        auto f = atom(depth);
        if (p < pattern.size() && (pattern[p] == '*' || pattern[p] == '+' || pattern[p] == '?')) {
            char op = pattern[p++];
            int a = node(), b = node();
            nfa.add_transition(a, -1, f.first);
            nfa.add_transition(f.last, -1, b);
            if (op != '+')
                nfa.add_transition(a, -1, b);
            if (op != '?')
                nfa.add_transition(f.last, -1, f.first);
            f = {a, b};
            if (p < pattern.size() && (pattern[p] == '*' || pattern[p] == '+' || pattern[p] == '?'))
                throw ParseError(ParseErrorKind::unexpected_token, p, "stacked repetition");
        }
        return f;
    };
    concatenation = [&](std::size_t depth) {
        if (p == pattern.size() || pattern[p] == '|' || pattern[p] == ')')
            return empty();
        auto f = repeat(depth);
        while (p < pattern.size() && pattern[p] != '|' && pattern[p] != ')') {
            auto g = repeat(depth);
            nfa.add_transition(f.last, -1, g.first);
            f.last = g.last;
        }
        return f;
    };
    expression = [&](std::size_t depth) {
        if (depth > limits.max_depth)
            throw ParseError(ParseErrorKind::depth_limit, p, "regex recursion limit");
        auto f = concatenation(depth);
        while (p < pattern.size() && pattern[p] == '|') {
            ++p;
            auto g = concatenation(depth);
            int a = node(), b = node();
            nfa.add_transition(a, -1, f.first);
            nfa.add_transition(a, -1, g.first);
            nfa.add_transition(f.last, -1, b);
            nfa.add_transition(g.last, -1, b);
            f = {a, b};
        }
        return f;
    };
    auto result = expression(0);
    if (p != pattern.size())
        throw ParseError(ParseErrorKind::unexpected_token, p, "trailing regex input");
    nfa.set_start(result.first);
    nfa.set_accepting(result.last);
    return nfa;
}
inline bool regex_full_match(const NFA &nfa, const std::string &text) {
    if (nfa.alphabet_size() != 256)
        throw std::invalid_argument("byte matching needs 256 symbols");
    std::vector<int> word;
    for (unsigned char c : text)
        word.push_back(c);
    return nfa.accepts(word);
}
} // namespace cp
