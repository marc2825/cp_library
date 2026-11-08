#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include "../dp_optimization/detail/arithmetic.hpp"

namespace cp {
// sum_{i=0}^{n-1}floor((a*i+b)/m)。0<=n<2^32、1<=m<2^32。
// a,bは符号付き64bit。正規化の加算/乗算を含めlong long超過は例外。
inline long long floor_sum(long long n,long long m,long long a,long long b) {
    const auto bound=static_cast<long long>(std::numeric_limits<std::uint32_t>::max());
    if (n<0 || n>bound || m<=0 || m>bound) throw std::invalid_argument("floor_sum n,m outside 32bit domain");
    if (!n) return 0;
    auto quotient=[m](long long x) { auto q=x/m; return x%m<0?q-1:q; };
    const auto qa=quotient(a),qb=quotient(b);
    auto ua=static_cast<std::uint64_t>(a%m<0?a%m+m:a%m);
    auto ub=static_cast<std::uint64_t>(b%m<0?b%m+m:b%m);
    std::uint64_t un=static_cast<std::uint64_t>(n),um=static_cast<std::uint64_t>(m),sum=0;
    while (true) {
        if (ua>=um) { sum+=un*(un-1)/2*(ua/um); ua%=um; }
        if (ub>=um) { sum+=un*(ub/um); ub%=um; }
        const auto y=ua*un+ub;
        if (y<um) break;
        un=y/um; ub=y%um; std::swap(um,ua);
    }
    const auto triangle=n%2?n*((n-1)/2):(n/2)*(n-1);
    return detail::dp_add(detail::dp_add(detail::dp_multiply(triangle,qa),detail::dp_multiply(n,qb)),static_cast<long long>(sum));
}
} // namespace cp
