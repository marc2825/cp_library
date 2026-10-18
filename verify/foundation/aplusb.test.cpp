// PROBLEM https://judge.yosupo.jp/problem/aplusb
#include "basic_utility/fast_io.hpp"
int main() {
    cp::FastScanner in; cp::FastPrinter out;
    long long a, b;
    if (in.read(a,b)) out.writeln(a+b);
    out.flush();
}
