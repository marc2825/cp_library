#pragma once
#include <algorithm>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cp {
// CNFのDPLL。2 watched literals・単位伝播・明示decision stack。最悪指数時間。
// 外部literalは±(variable+1)、0は禁止。solveごとに仮定と伝播状態を作り直す。
class SatSolver {
    int variables_;
    std::vector<std::vector<int>> clauses_;
    int encode(int literal) const {
        if (literal == 0 || literal > variables_ || literal < -variables_)
            throw std::out_of_range("SAT literal");
        return literal > 0 ? 2 * (literal - 1) : 2 * (-literal - 1) + 1;
    }
public:
    explicit SatSolver(int variables) : variables_(variables) {
        if (variables < 0 || variables > std::numeric_limits<int>::max() / 2)
            throw std::invalid_argument("SAT variable count");
    }
    void add_clause(const std::vector<int>& literals) {
        std::vector<int> clause;
        for (int x : literals) clause.push_back(encode(x));
        std::sort(clause.begin(), clause.end());
        clause.erase(std::unique(clause.begin(), clause.end()), clause.end());
        for (std::size_t i = 1; i < clause.size(); ++i)
            if ((clause[i] ^ 1) == clause[i-1]) return; // 恒真節
        clauses_.push_back(std::move(clause));
    }
    std::optional<std::vector<bool>> solve(const std::vector<int>& assumptions = {}) const {
        auto clauses = clauses_;
        std::vector<int> encoded;
        for (int x : assumptions) encoded.push_back(encode(x));
        std::vector<std::vector<std::size_t>> watch(static_cast<std::size_t>(variables_) * 2);
        std::vector<int> value(variables_, -1), trail;
        std::vector<std::size_t> occurrences(variables_);
        std::size_t head = 0;
        auto truth = [&](int x) { return value[x/2] < 0 ? -1 : value[x/2] ^ (x & 1); };
        auto assign = [&](int x) {
            if (truth(x) >= 0) return truth(x) == 1;
            value[x/2] = 1 ^ (x & 1); trail.push_back(x); return true;
        };
        for (std::size_t c = 0; c < clauses.size(); ++c) {
            auto& clause = clauses[c];
            if (clause.empty()) return std::nullopt;
            for (int x : clause) ++occurrences[x/2];
            if (clause.size() == 1) { if (!assign(clause[0])) return std::nullopt; }
            else { watch[clause[0]].push_back(c); watch[clause[1]].push_back(c); }
        }
        for (int x : encoded) if (!assign(x)) return std::nullopt;
        auto propagate = [&]() {
            while (head < trail.size()) {
                const int falsified = trail[head++] ^ 1;
                auto& watched = watch[falsified];
                for (std::size_t i = 0; i < watched.size();) {
                    const auto c = watched[i]; auto& clause = clauses[c];
                    if (clause[0] == falsified) std::swap(clause[0], clause[1]);
                    if (truth(clause[0]) == 1) { ++i; continue; }
                    std::size_t replacement = 2;
                    while (replacement < clause.size() && truth(clause[replacement]) == 0) ++replacement;
                    if (replacement < clause.size()) {
                        std::swap(clause[1], clause[replacement]);
                        watch[clause[1]].push_back(c);
                        watched[i] = watched.back(); watched.pop_back();
                    } else {
                        if (!assign(clause[0])) return false;
                        ++i;
                    }
                }
            }
            return true;
        };
        struct Decision { int literal; std::size_t mark; bool flipped; };
        std::vector<Decision> decisions;
        for (;;) {
            if (!propagate()) {
                bool resumed = false;
                while (!decisions.empty()) {
                    auto& d = decisions.back();
                    while (trail.size() > d.mark) { value[trail.back()/2] = -1; trail.pop_back(); }
                    head = trail.size();
                    if (!d.flipped) { d.flipped = true; assign(d.literal ^ 1); resumed = true; break; }
                    decisions.pop_back();
                }
                if (!resumed) return std::nullopt;
                continue;
            }
            int variable = -1;
            for (int v = 0; v < variables_; ++v)
                if (value[v] < 0 && (variable < 0 || occurrences[v] > occurrences[variable])) variable = v;
            if (variable < 0) {
                std::vector<bool> result(variables_);
                for (int v = 0; v < variables_; ++v) result[v] = value[v] != 0;
                return result;
            }
            decisions.push_back({2 * variable, trail.size(), false});
            assign(2 * variable);
        }
    }
};
} // namespace cp
