#include <cstdint>
#include <iostream>
#include <string>
#include "basic_utility/all.hpp"

template <class M> void modular(cp::FastScanner& in, cp::FastPrinter& out) {
    __int128_t a, b; std::uint64_t e;
    if (!in.read(a, b, e)) throw std::runtime_error("missing modular input");
    M x(a), y(b);
    out.write(x.val(), y.val(), (x+y).val(), (x-y).val(), (x*y).val(), x.pow(e).val());
    const auto inverse = y.try_inv();
    out.write(' ');
    if (inverse) out.writeln(inverse->val()); else out.writeln("none");
}
template <class T> void safety(cp::FastScanner& in, cp::FastPrinter& out) {
    T a, b;
    if (!in.read(a, b)) throw std::runtime_error("missing arithmetic input");
    for (const auto& value : {cp::checked_add(a,b), cp::checked_sub(a,b), cp::checked_mul(a,b)}) {
        if (value) out.write(*value); else out.write("none");
        out.write(' ');
    }
    out.write(cp::saturating_add(a,b), cp::saturating_sub(a,b), cp::saturating_mul(a,b));
    for (int i = 0; i < 2; ++i) {
        out.write(' ');
        try { out.write(i ? cp::ceil_div(a,b) : cp::floor_div(a,b)); }
        catch (const std::invalid_argument&) { out.write("invalid"); }
        catch (const std::overflow_error&) { out.write("overflow"); }
    }
    out.writeln();
}
int main() {
    cp::FastScanner in; cp::FastPrinter out; std::string type;
    while (in.read(type)) {
        if (type == "D") {
            std::uint32_t m; in.read(m); cp::DynamicModInt<>::set_mod(m);
            modular<cp::DynamicModInt<>>(in,out);
        } else if (type == "S") modular<cp::StaticModInt<4294967295U>>(in,out);
        else if (type == "M") modular<cp::MontgomeryModInt64<9223372036854775807ULL>>(in,out);
        else if (type == "H") modular<cp::ModInt61>(in,out);
        else if (type == "B") {
            std::uint32_t m, a, b; std::uint64_t x; in.read(m,x,a,b);
            cp::BarrettReduction r(m); out.writeln(r.reduce(x), r.multiply(a,b));
        } else if (type == "R") {
            std::uint64_t m, a, b; __uint128_t x; in.read(m,x,a,b);
            cp::MontgomeryReduction64 r(m);
            out.writeln(r.reduce(x), r.encode(a), r.decode(r.multiply_encoded(r.encode(a),r.encode(b))));
        } else if (type == "I64") safety<std::int64_t>(in,out);
        else if (type == "U64") safety<std::uint64_t>(in,out);
        else if (type == "I128") safety<__int128_t>(in,out);
        else if (type == "U128") safety<__uint128_t>(in,out);
        else throw std::runtime_error("unknown query");
    }
    out.flush();
}
