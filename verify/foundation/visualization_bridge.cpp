#include <cstdint>
#include <cstdio>
#include <iostream>
#include <limits>
#include <string>
#include "basic_utility/all.hpp"
int main() {
    using S = cp::StaticModInt<12>;
    S a(-5), b(5);
    std::cout << "static_modint {\"a\":\"" << a << "\",\"b\":\"" << b << "\",\"sum\":\"" << a+b
              << "\",\"product\":\"" << a*b << "\",\"inverse\":\"" << b.inv() << "\"}\n";
    using D = cp::DynamicModInt<4>;
    D::set_mod(12); D x(5); const auto old = x.val(); D::set_mod(11);
    std::string status = "valid";
    try { (void)x.val(); } catch (const std::logic_error&) { status = "stale"; }
    std::cout << "dynamic_modint {\"oldValue\":\"" << old << "\",\"status\":\"" << status << "\",\"newValue\":\"" << D(5) << "\"}\n";
    std::cout << "barrett_reduction \"" << cp::BarrettReduction(7).reduce(std::numeric_limits<std::uint64_t>::max()) << "\"\n";
    using M = cp::MontgomeryModInt64<101>;
    std::cout << "montgomery_modint \"" << M(12)*M(13) << "\"\n";
    std::cout << "modint61 \"" << cp::ModInt61(-1)*cp::ModInt61(-1) << "\"\n";
    auto* file = std::tmpfile();
    if (!file) return 1;
    std::fputs(" -128 127 128 +7",file); std::rewind(file);
    {
        cp::BasicFastScanner<3> in(file); std::int8_t v = 0;
        std::cout << "fast_io [";
        for (int i = 0; i < 4; ++i) {
            if (i) std::cout << ',';
            try { if (!in.read(v)) return 2; std::cout << '"' << int(v) << '"'; }
            catch (const std::out_of_range&) { std::cout << "\"overflow\""; }
        }
        std::cout << "]\n";
    }
    std::fclose(file);
    std::int8_t p = 127, q = 3;
    auto value = [](std::optional<std::int8_t> v) { return v ? std::to_string(*v) : "none"; };
    std::cout << "safe_integer {\"sum\":\"" << value(cp::checked_add(p,q)) << "\",\"saturatedSum\":\"" << int(cp::saturating_add(p,q))
              << "\",\"product\":\"" << value(cp::checked_mul(p,q)) << "\",\"saturatedProduct\":\"" << int(cp::saturating_mul(p,q))
              << "\",\"floor\":\"" << int(cp::floor_div(p,q)) << "\",\"ceil\":\"" << int(cp::ceil_div(p,q)) << "\"}\n";
}
