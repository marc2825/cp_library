#pragma once
#include <string_view>
#include "suffix_automaton.hpp"

namespace cp {
// 連続する共通部分文字列。SAMを共有。docs/string_algorithm/longest_common_substring.md
inline CommonSubstringResult longest_common_substring(std::string_view first, std::string_view second) {
    return SuffixAutomaton(first).longest_common_substring(second);
}
} // namespace cp
