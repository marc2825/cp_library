// PROBLEM https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_B
#include "basic_utility/fast_io.hpp"
#include "basic_utility/static_modint.hpp"
int main() {
    cp::FastScanner in; cp::FastPrinter out;
    long long base; std::uint64_t exponent;
    if (in.read(base,exponent)) out.writeln(cp::ModInt1000000007(base).pow(exponent).val());
    out.flush();
}
