#pragma once
#include "detail/common.hpp"
#include "detail/sam_index.hpp"
#include <map>
#include <optional>
#include <string>

namespace cp {
// A SAM over documents separated by a private symbol 256. Queries traverse byte edges only.
class GeneralizedSuffixAutomaton {
  public:
    struct Occurrence {
        int document, position;
        friend bool operator==(const Occurrence &a, const Occurrence &b) {
            return a.document == b.document && a.position == b.position;
        }
    };

  private:
    struct State {
        std::map<int, int> next;
        int length = 0, link = -1;
    };
    std::vector<State> states_{1};
    std::vector<int> terminals_, lengths_;
    std::vector<Occurrence> coordinates_;
    int last_ = 0;
    mutable std::optional<string_algorithm_detail::SAMIndex> index_;
    void extend(int c, Occurrence coordinate) {
        const int current = static_cast<int>(states_.size());
        states_.push_back({{}, states_[last_].length + 1, 0});
        int p = last_;
        while (p >= 0 && !states_[p].next.count(c)) {
            states_[p].next[c] = current;
            p = states_[p].link;
        }
        if (p >= 0) {
            const int q = states_[p].next.at(c);
            if (states_[q].length == states_[p].length + 1)
                states_[current].link = q;
            else {
                State copy = states_[q];
                copy.length = states_[p].length + 1;
                const int clone = static_cast<int>(states_.size());
                states_.push_back(std::move(copy));
                while (p >= 0) {
                    auto it = states_[p].next.find(c);
                    if (it == states_[p].next.end() || it->second != q)
                        break;
                    it->second = clone;
                    p = states_[p].link;
                }
                states_[q].link = states_[current].link = clone;
            }
        }
        last_ = current;
        terminals_.push_back(current);
        coordinates_.push_back(coordinate);
    }
    const string_algorithm_detail::SAMIndex &index() const {
        if (!index_)
            index_.emplace(states_, terminals_);
        return *index_;
    }

  public:
    GeneralizedSuffixAutomaton() = default;
    explicit GeneralizedSuffixAutomaton(const std::vector<std::string> &documents) {
        for (const auto &s : documents)
            add_document(s);
    }
    int document_count() const { return static_cast<int>(lengths_.size()); }
    std::size_t state_count() const { return states_.size(); }
    int add_document(std::string_view text) {
        const auto limit = static_cast<std::size_t>((std::numeric_limits<int>::max() - 1) / 2);
        if (terminals_.size() >= limit || text.size() >= limit - terminals_.size())
            throw std::length_error("generalized SAM too large");
        index_.reset();
        const int id = document_count();
        lengths_.push_back(static_cast<int>(text.size()));
        int position = 0;
        for (unsigned char c : text)
            extend(c, {id, position++});
        extend(256, {-1, -1});
        return id;
    }
    bool contains(std::string_view pattern) const {
        return document_count() && string_algorithm_detail::sam_walk(states_, pattern) >= 0;
    }
    std::uint64_t count_occurrences(std::string_view pattern) const {
        if (pattern.empty())
            return terminals_.size(); // each document contributes length+1 boundaries
        return index().count(string_algorithm_detail::sam_walk(states_, pattern));
    }
    std::vector<Occurrence> occurrences(std::string_view pattern) const {
        std::vector<Occurrence> out;
        if (pattern.empty()) {
            for (int d = 0; d < document_count(); ++d)
                for (int p = 0; p <= lengths_[d]; ++p)
                    out.push_back({d, p});
        } else if (pattern.size() <= terminals_.size()) {
            for (int p : index().occurrences(string_algorithm_detail::sam_walk(states_, pattern),
                                             static_cast<int>(pattern.size())))
                out.push_back(coordinates_[p]);
        }
        return out;
    }
    std::vector<int> containing_documents(std::string_view pattern) const {
        auto positions = occurrences(pattern);
        std::vector<int> out;
        for (auto p : positions)
            if (out.empty() || out.back() != p.document)
                out.push_back(p.document);
        return out;
    }
    std::uint64_t distinct_substrings() const { return index().distinct_count(); }
    std::optional<std::string> kth_substring(std::uint64_t k) const { return index().kth(states_, k); }
};
} // namespace cp
