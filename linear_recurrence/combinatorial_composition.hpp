#pragma once
#include "../fps/exp.hpp"
#include "../fps/inv.hpp"
#include "../fps/log.hpp"
namespace cp {
enum class LabelledConstruction { set, sequence, cycle };
// f is an EGF stored as ordinary coefficients (already divided by factorial).
// SET(f)=exp(f), SEQ(f)=1/(1-f), CYC(f)=-log(1-f), f(0)=0.
template <int Mod = 998244353, int Root = 3>
std::vector<int> labelled_generating_function_compose(const std::vector<int> &f, std::size_t n,
                                                      LabelledConstruction operation) {
    using F = fps_detail::Field<Mod>;
    if (!f.empty() && F::norm(f[0]))
        throw std::domain_error("labelled construction requires no empty component");
    if (operation == LabelledConstruction::set)
        return fps_exp<Mod, Root>(f, n);
    auto g = fps_detail::prefix<Mod>(f, n);
    if (!n)
        return {};
    for (auto &x : g)
        x = F::sub(0, x);
    g[0] = 1;
    if (operation == LabelledConstruction::sequence)
        return fps_inv<Mod, Root>(g, n);
    if (operation != LabelledConstruction::cycle)
        throw std::invalid_argument("unknown labelled construction");
    g = fps_log<Mod, Root>(g, n);
    for (auto &x : g)
        x = F::sub(0, x);
    return g;
}
} // namespace cp
