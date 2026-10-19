// competitive-verifier: STANDALONE
#include <cassert>
#include <cstdio>
#include "basic_utility/big_integer.hpp"
#include "basic_utility/fast_io.hpp"
#include "basic_utility/rollback_array.hpp"
int main() {
    auto value = cp::BigInt::parse("123456789012345678901234567890");
    assert((value * 3 / 3) == value);
    cp::RollbackArray<int> a(3, 0);
    const auto snapshot = a.snapshot();
    a.set(1, 7);
    assert(a[1] == 7);
    a.rollback(snapshot);
    assert(a[1] == 0);
    auto* file = std::tmpfile();
    assert(file);
    { cp::FastPrinter out(file); out.writeln(-123, 456); out.flush(); }
    std::rewind(file);
    { cp::FastScanner in(file); int x, y; assert(in.read(x, y) && x == -123 && y == 456); }
    std::fclose(file);
}
