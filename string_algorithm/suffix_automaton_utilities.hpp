#pragma once
#include "detail/sam_index.hpp"
#include "suffix_automaton.hpp"
#include <string>

namespace cp {
// Static companion index. Keeps the existing online SAM API unchanged.
class SuffixAutomatonUtilities {
    SuffixAutomaton automaton_;
    string_algorithm_detail::SAMIndex index_;
    static std::vector<int> terminals(const SuffixAutomaton &a) {
        std::vector<int> out(a.size());
        for (std::size_t v = 1; v < a.states().size(); ++v)
            if (!a.states()[v].clone)
                out[a.states()[v].first_end] = static_cast<int>(v);
        return out;
    }

  public:
    explicit SuffixAutomatonUtilities(std::string_view text = {})
        : automaton_(text), index_(automaton_.states(), terminals(automaton_)) {}
    int size() const { return automaton_.size(); }
    const SuffixAutomaton &automaton() const { return automaton_; }
    std::uint64_t distinct_substrings() const { return index_.distinct_count(); }
    // Zero-based unsigned-byte lexicographic order, excluding the empty substring.
    std::optional<std::string> kth_substring(std::uint64_t k) const {
        return index_.kth(automaton_.states(), k);
    }
    std::vector<int> occurrences(std::string_view pattern) const {
        if (pattern.empty()) {
            std::vector<int> out(size() + 1);
            std::iota(out.begin(), out.end(), 0);
            return out;
        }
        if (pattern.size() > static_cast<std::size_t>(size()))
            return {};
        return index_.occurrences(string_algorithm_detail::sam_walk(automaton_.states(), pattern),
                                  static_cast<int>(pattern.size()));
    }
};
} // namespace cp
