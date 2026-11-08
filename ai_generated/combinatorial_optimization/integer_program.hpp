#pragma once
#include "linear_program.hpp"
#include <optional>
namespace cp {
enum class IntegerProgramStatus { optimal, infeasible, node_limit, lp_limit };
struct IntegerProgramOptions {
    std::size_t max_nodes = 10000;
    SimplexOptions simplex;
};
struct IntegerProgramResult {
    IntegerProgramStatus status = IntegerProgramStatus::infeasible;
    bool has_solution = false;
    BigRational value = 0;
    std::vector<BigInt> solution;
    std::optional<BigRational> upper_bound;
    std::size_t nodes = 0;
};
// Exact bounded integer LP: maximize c*x, A*x<=b, lower<=x<=upper, all x integral.
// LP relaxations use the same rational simplex; no floating rounding of branch bounds.
inline IntegerProgramResult
bounded_integer_program(const std::vector<std::vector<BigRational>> &a, const std::vector<BigRational> &b,
                        const std::vector<BigRational> &c, const std::vector<BigInt> &lower,
                        const std::vector<BigInt> &upper, IntegerProgramOptions options = {}) {
    lp_detail::validate(a, b, c);
    const auto n = c.size();
    if (lower.size() != n || upper.size() != n)
        throw std::invalid_argument("integer bounds dimension differs");
    IntegerProgramResult out;
    for (std::size_t j = 0; j < n; j++)
        if (lower[j] > upper[j])
            return out;
    struct Node {
        std::vector<BigInt> lo, hi;
        std::optional<BigRational> bound;
    };
    std::vector<Node> stack{{lower, upper, std::nullopt}};
    while (!stack.empty()) {
        if (out.has_solution && stack.back().bound && *stack.back().bound <= out.value) {
            stack.pop_back();
            continue;
        }
        if (out.nodes == options.max_nodes) {
            out.status = IntegerProgramStatus::node_limit;
            break;
        }
        auto node = std::move(stack.back());
        stack.pop_back();
        ++out.nodes;
        auto rows = a;
        auto rhs = b;
        BigRational constant = 0;
        for (std::size_t j = 0; j < n; j++)
            constant += c[j] * BigRational(node.lo[j]);
        for (std::size_t i = 0; i < a.size(); i++)
            for (std::size_t j = 0; j < n; j++)
                rhs[i] -= a[i][j] * BigRational(node.lo[j]);
        for (std::size_t j = 0; j < n; j++) {
            std::vector<BigRational> row(n);
            row[j] = 1;
            rows.push_back(std::move(row));
            rhs.emplace_back(node.hi[j] - node.lo[j]);
        }
        const auto relaxation = linear_program(rows, rhs, c, options.simplex);
        if (relaxation.status == LPStatus::iteration_limit) {
            stack.push_back(std::move(node));
            out.status = IntegerProgramStatus::lp_limit;
            break;
        }
        if (relaxation.status == LPStatus::infeasible)
            continue;
        if (relaxation.status != LPStatus::optimal)
            throw std::logic_error("bounded rational LP relaxation failed");
        const auto bound = constant + relaxation.value;
        if (out.has_solution && bound <= out.value)
            continue;
        std::size_t branch = n;
        std::vector<BigInt> candidate(n);
        BigInt split;
        for (std::size_t j = 0; j < n; j++) {
            const auto value = BigRational(node.lo[j]) + relaxation.primal[j];
            auto q = divmod(value.numerator(), value.denominator());
            if (q.second) {
                branch = j;
                split = q.first - (q.second.sign() < 0 ? 1 : 0);
                break;
            }
            candidate[j] = q.first;
        }
        if (branch == n) {
            out.has_solution = true;
            out.value = bound;
            out.solution = std::move(candidate);
            continue;
        }
        auto right = node;
        right.lo[branch] = split + 1;
        right.bound = bound;
        node.hi[branch] = split;
        node.bound = bound;
        if (right.lo[branch] <= right.hi[branch])
            stack.push_back(std::move(right));
        if (node.lo[branch] <= node.hi[branch])
            stack.push_back(std::move(node));
    }
    if (stack.empty()) {
        out.status = out.has_solution ? IntegerProgramStatus::optimal : IntegerProgramStatus::infeasible;
        if (out.has_solution)
            out.upper_bound = out.value;
    } else {
        bool known = true;
        std::optional<BigRational> bound =
            out.has_solution ? std::optional<BigRational>(out.value) : std::nullopt;
        for (const auto &node : stack) {
            if (!node.bound) {
                known = false;
                break;
            }
            if (!bound || *node.bound > *bound)
                bound = node.bound;
        }
        if (known)
            out.upper_bound = bound;
    }
    return out;
}
} // namespace cp
