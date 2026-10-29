#pragma once
#include "../dp_optimization/digit_dp.hpp"
#include <algorithm>
#include <climits>
#include <map>
#include <numeric>
#include <queue>
namespace cp {
class DFA {
    int alphabet_, start_;
    std::vector<std::vector<int>> transitions_;
    std::vector<bool> accepting_;
    void state(int q) const {
        if (q < 0 || q >= size())
            throw std::out_of_range("DFA state");
    }
    void symbol(int a) const {
        if (a < 0 || a >= alphabet_)
            throw std::out_of_range("DFA symbol");
    }

  public:
    DFA(int states, int alphabet, int start = 0) : alphabet_(alphabet), start_(start) {
        if (states <= 0 || alphabet < 0 || start < 0 || start >= states)
            throw std::invalid_argument("invalid DFA dimensions/start");
        transitions_.assign(states, std::vector<int>(alphabet, -1));
        accepting_.assign(states, false);
    }
    int size() const { return static_cast<int>(transitions_.size()); }
    int alphabet_size() const { return alphabet_; }
    int start() const { return start_; }
    void set_start(int q) {
        state(q);
        start_ = q;
    }
    int add_state(bool accepting = false) {
        if (size() == INT_MAX)
            throw std::length_error("too many DFA states");
        auto row = std::vector<int>(alphabet_, -1);
        accepting_.push_back(accepting);
        try {
            transitions_.push_back(std::move(row));
        } catch (...) {
            accepting_.pop_back();
            throw;
        }
        return size() - 1;
    }
    void set_transition(int q, int a, int to) {
        state(q);
        symbol(a);
        if (to != -1)
            state(to);
        transitions_[q][a] = to;
    }
    int transition(int q, int a) const {
        symbol(a);
        if (q == -1)
            return -1;
        state(q);
        return transitions_[q][a];
    }
    void set_accepting(int q, bool value = true) {
        state(q);
        accepting_[q] = value;
    }
    bool accepting(int q) const {
        if (q == -1)
            return false;
        state(q);
        return accepting_[q];
    }
    bool accepts(const std::vector<int> &word) const {
        int q = start_;
        for (int a : word)
            q = transition(q, a);
        return accepting(q);
    }
    const std::vector<std::vector<int>> &transitions() const { return transitions_; }
};
class NFA {
  public:
    struct Edge {
        int symbol, to;
    }; // symbol=-1 is epsilon
  private:
    int alphabet_, start_;
    std::vector<std::vector<Edge>> edges_;
    std::vector<bool> accepting_;
    void state(int q) const {
        if (q < 0 || q >= size())
            throw std::out_of_range("NFA state");
    }

  public:
    NFA(int states, int alphabet, int start = 0) : alphabet_(alphabet), start_(start) {
        if (states <= 0 || alphabet < 0 || start < 0 || start >= states)
            throw std::invalid_argument("invalid NFA dimensions/start");
        edges_.resize(states);
        accepting_.assign(states, false);
    }
    int size() const { return static_cast<int>(edges_.size()); }
    int alphabet_size() const { return alphabet_; }
    int start() const { return start_; }
    void set_start(int q) {
        state(q);
        start_ = q;
    }
    int add_state(bool accepting = false) {
        if (size() == INT_MAX)
            throw std::length_error("too many NFA states");
        accepting_.push_back(accepting);
        try {
            edges_.emplace_back();
        } catch (...) {
            accepting_.pop_back();
            throw;
        }
        return size() - 1;
    }
    void set_accepting(int q, bool value = true) {
        state(q);
        accepting_[q] = value;
    }
    bool accepting(int q) const {
        state(q);
        return accepting_[q];
    }
    void add_transition(int q, int symbol, int to) {
        state(q);
        state(to);
        if (symbol < -1 || symbol >= alphabet_)
            throw std::out_of_range("NFA symbol");
        edges_[q].push_back({symbol, to});
    }
    const std::vector<Edge> &edges(int q) const {
        state(q);
        return edges_[q];
    }
    std::vector<int> epsilon_closure(const std::vector<int> &initial) const {
        std::vector<bool> seen(size());
        std::vector<int> queue;
        for (int q : initial) {
            state(q);
            if (!seen[q]) {
                seen[q] = true;
                queue.push_back(q);
            }
        }
        for (std::size_t i = 0; i < queue.size(); ++i)
            for (auto e : edges_[queue[i]])
                if (e.symbol == -1 && !seen[e.to]) {
                    seen[e.to] = true;
                    queue.push_back(e.to);
                }
        std::sort(queue.begin(), queue.end());
        return queue;
    }
    std::vector<int> advance(const std::vector<int> &active, int symbol) const {
        if (symbol < 0 || symbol >= alphabet_)
            throw std::out_of_range("NFA symbol");
        std::vector<int> next;
        for (int q : epsilon_closure(active)) {
            for (auto e : edges_[q])
                if (e.symbol == symbol)
                    next.push_back(e.to);
        }
        return epsilon_closure(next);
    }
    bool accepts(const std::vector<int> &word) const {
        auto active = epsilon_closure({start_});
        for (int a : word)
            active = advance(active, a);
        for (int q : active)
            if (accepting_[q])
                return true;
        return false;
    }
};
inline DFA complete_dfa(DFA a) {
    int sink = -1;
    const int n = a.size();
    for (int q = 0; q < n; ++q)
        for (int c = 0; c < a.alphabet_size(); ++c)
            if (a.transition(q, c) < 0) {
                if (sink < 0) {
                    sink = a.add_state();
                    for (int b = 0; b < a.alphabet_size(); ++b)
                        a.set_transition(sink, b, sink);
                }
                a.set_transition(q, c, sink);
            }
    return a;
}
inline DFA complement_dfa(DFA a) {
    a = complete_dfa(std::move(a));
    for (int q = 0; q < a.size(); ++q)
        a.set_accepting(q, !a.accepting(q));
    return a;
}
struct DeterminizedNFA {
    DFA automaton;
    std::vector<std::vector<int>> subsets;
};
inline DeterminizedNFA determinize(const NFA &nfa, std::size_t max_states = 1000000) {
    if (!max_states)
        throw std::length_error("DFA state budget");
    auto first = nfa.epsilon_closure({nfa.start()});
    DeterminizedNFA out{DFA(1, nfa.alphabet_size()), {first}};
    std::map<std::vector<int>, int> ids{{first, 0}};
    for (std::size_t i = 0; i < out.subsets.size(); ++i) {
        for (int q : out.subsets[i])
            if (nfa.accepting(q)) {
                out.automaton.set_accepting(static_cast<int>(i));
                break;
            }
        for (int c = 0; c < nfa.alphabet_size(); ++c) {
            auto next = nfa.advance(out.subsets[i], c);
            auto found = ids.find(next);
            int to;
            if (found != ids.end())
                to = found->second;
            else {
                if (out.subsets.size() >= max_states)
                    throw std::length_error("DFA state budget");
                to = out.automaton.add_state();
                ids.emplace(next, to);
                out.subsets.push_back(std::move(next));
            }
            out.automaton.set_transition(static_cast<int>(i), c, to);
        }
    }
    return out;
}
enum class AutomatonBoolean { intersection, unite, difference, symmetric_difference };
struct DFAProduct {
    DFA automaton;
    std::vector<std::pair<int, int>> states;
};
inline DFAProduct dfa_product(const DFA &a, const DFA &b,
                              AutomatonBoolean operation = AutomatonBoolean::intersection,
                              std::size_t max_states = 1000000) {
    if (a.alphabet_size() != b.alphabet_size())
        throw std::invalid_argument("DFA alphabets differ");
    if (operation != AutomatonBoolean::intersection && operation != AutomatonBoolean::unite &&
        operation != AutomatonBoolean::difference && operation != AutomatonBoolean::symmetric_difference)
        throw std::invalid_argument("invalid DFA Boolean operation");
    if (!max_states)
        throw std::length_error("DFA product budget");
    std::pair<int, int> first{a.start(), b.start()};
    DFAProduct out{DFA(1, a.alphabet_size()), {first}};
    std::map<std::pair<int, int>, int> ids{{first, 0}};
    for (std::size_t i = 0; i < out.states.size(); ++i) {
        const auto [p, q] = out.states[i];
        const bool x = a.accepting(p), y = b.accepting(q);
        out.automaton.set_accepting(static_cast<int>(i), operation == AutomatonBoolean::intersection ? x && y
                                                         : operation == AutomatonBoolean::unite      ? x || y
                                                         : operation == AutomatonBoolean::difference
                                                             ? x && !y
                                                             : x != y);
        for (int c = 0; c < a.alphabet_size(); ++c) {
            std::pair<int, int> next{a.transition(p, c), b.transition(q, c)};
            auto found = ids.find(next);
            int to;
            if (found != ids.end())
                to = found->second;
            else {
                if (out.states.size() >= max_states)
                    throw std::length_error("DFA product budget");
                to = out.automaton.add_state();
                ids.emplace(next, to);
                out.states.push_back(next);
            }
            out.automaton.set_transition(static_cast<int>(i), c, to);
        }
    }
    return out;
}
struct MinimizedDFA {
    DFA automaton;
    std::vector<int> original_to_state;
    std::vector<std::vector<int>> blocks;
};
// Moore refinement after completion/reachability. Maps unreachable original states to -1.
inline MinimizedDFA minimize_dfa(const DFA &original) {
    const auto a = complete_dfa(original);
    const int n = a.size(), sigma = a.alphabet_size();
    std::vector<int> reachable{a.start()}, index(n, -1);
    index[a.start()] = 0;
    for (std::size_t i = 0; i < reachable.size(); ++i)
        for (int c = 0; c < sigma; ++c) {
            const int q = a.transition(reachable[i], c);
            if (index[q] < 0) {
                index[q] = static_cast<int>(reachable.size());
                reachable.push_back(q);
            }
        }
    std::vector<int> groups(reachable.size());
    for (std::size_t i = 0; i < groups.size(); ++i)
        groups[i] = a.accepting(reachable[i]);
    for (;;) {
        std::map<std::vector<int>, int> ids;
        std::vector<int> next;
        for (std::size_t i = 0; i < reachable.size(); ++i) {
            std::vector<int> key{groups[i]};
            for (int c = 0; c < sigma; ++c)
                key.push_back(groups[index[a.transition(reachable[i], c)]]);
            auto [it, inserted] = ids.emplace(std::move(key), static_cast<int>(ids.size()));
            (void)inserted;
            next.push_back(it->second);
        }
        if (groups == next)
            break;
        groups = std::move(next);
    }
    const int count = *std::max_element(groups.begin(), groups.end()) + 1;
    MinimizedDFA out{DFA(count, sigma, groups[0]), std::vector<int>(original.size(), -1),
                     std::vector<std::vector<int>>(count)};
    for (std::size_t i = 0; i < reachable.size(); ++i) {
        const int q = reachable[i], g = groups[i];
        out.blocks[g].push_back(q);
        if (q < original.size())
            out.original_to_state[q] = g;
        out.automaton.set_accepting(g, a.accepting(q));
        for (int c = 0; c < sigma; ++c)
            out.automaton.set_transition(g, c, groups[index[a.transition(q, c)]]);
    }
    return out;
}
// Canonical decimal representation: ignore padding, but represent zero by one digit 0.
template <class Count = long long>
Count dfa_digit_count(const std::string &upper, const DFA &a, bool canonical = true) {
    if (a.alphabet_size() != 10)
        throw std::invalid_argument("decimal DFA needs ten symbols");
    return digit_dp<Count>(
        upper, a.size(), a.start(),
        [&](int state, int digit, bool, bool next_started) {
            return canonical && !next_started ? a.start() : a.transition(state, digit);
        },
        [&](int state, bool started) {
            return a.accepting(canonical && !started ? a.transition(a.start(), 0) : state);
        });
}
} // namespace cp
