#include <algorithm>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>
#include "number_theory_convolution/all.hpp"
#include "number_theory_basic/smallest_prime_factor.hpp"
#include "modint.hpp"
#define CHECK(...) do { if (!(__VA_ARGS__)) { std::cerr << "check failed at " << __LINE__ << ": " << #__VA_ARGS__ << '\n'; std::abort(); } } while (false)
template <class E, class F> void throws(F f) { bool caught = false; try { f(); } catch (const E&) { caught = true; } CHECK(caught); }
int array_cases = 0, pair_cases = 0;
int mu(int n) {
    int sign = 1;
    for (int p = 2; p <= n; ++p) if (n % p == 0) {
        n /= p; sign = -sign;
        if (n % p == 0) return 0;
    }
    return sign;
}
template <class T> std::vector<T> naive_transform(const std::vector<T>& f, bool divisor, bool inverse) {
    auto result = f;
    for (std::size_t i = 1; i < f.size(); ++i) {
        result[i] = T(0);
        for (std::size_t j = 1; j < f.size(); ++j) {
            if (!(divisor ? i % j == 0 : j % i == 0)) continue;
            const int sign = inverse ? mu(static_cast<int>(divisor ? i / j : j / i)) : 1;
            if (sign > 0) result[i] += f[j];
            if (sign < 0) result[i] -= f[j];
        }
    }
    return result;
}
template <class T> std::vector<T> naive_convolution(const std::vector<T>& a, const std::vector<T>& b, int kind) {
    std::vector<T> result(std::max(a.size(), b.size()), T(0));
    for (std::size_t i = 1; i < a.size(); ++i) for (std::size_t j = 1; j < b.size(); ++j) {
        const auto k = kind == 0 ? i * j : kind == 1 ? std::gcd(i, j) : std::lcm(i, j);
        if (k < result.size()) result[k] += a[i] * b[j];
    }
    return result;
}
void check_transforms(const std::vector<long long>& f) {
    auto dz = f, dm = f, mz = f, mm = f;
    cp::divisor_zeta_transform(dz); cp::divisor_mobius_transform(dm);
    cp::multiple_zeta_transform(mz); cp::multiple_mobius_transform(mm);
    CHECK(dz == naive_transform(f, true, false) && dm == naive_transform(f, true, true));
    CHECK(mz == naive_transform(f, false, false) && mm == naive_transform(f, false, true));
    cp::divisor_mobius_transform(dz); cp::divisor_zeta_transform(dm);
    cp::multiple_mobius_transform(mz); cp::multiple_zeta_transform(mm);
    CHECK(dz == f && dm == f && mz == f && mm == f);
    ++array_cases;
}
template <class T> void check_convolutions(const std::vector<T>& a, const std::vector<T>& b) {
    const auto before_a = a, before_b = b;
    CHECK(cp::dirichlet_convolution(a, b) == naive_convolution(a, b, 0));
    CHECK(cp::gcd_convolution(a, b) == naive_convolution(a, b, 1));
    CHECK(cp::lcm_convolution(a, b) == naive_convolution(a, b, 2));
    CHECK(a == before_a && b == before_b);
    ++pair_cases;
}
struct Composite {
    int value;
    Composite(long long x = 0) : value(static_cast<int>((x % 12 + 12) % 12)) {}
    Composite& operator+=(const Composite& x) { value = (value + x.value) % 12; return *this; }
    Composite& operator-=(const Composite& x) { value = (value - x.value + 12) % 12; return *this; }
    friend Composite operator*(const Composite& a, const Composite& b) { return Composite(a.value * b.value); }
    friend bool operator==(const Composite& a, const Composite& b) { return a.value == b.value; }
};
struct Matrix {
    long long a, b, c, d;
    Matrix(long long x = 0) : a(x), b(0), c(0), d(x) {}
    Matrix(long long aa, long long bb, long long cc, long long dd) : a(aa), b(bb), c(cc), d(dd) {}
    Matrix& operator+=(const Matrix& x) { a+=x.a; b+=x.b; c+=x.c; d+=x.d; return *this; }
    Matrix& operator-=(const Matrix& x) { a-=x.a; b-=x.b; c-=x.c; d-=x.d; return *this; }
    friend Matrix operator*(const Matrix& x, const Matrix& y) { return {x.a*y.a+x.b*y.c,x.a*y.b+x.b*y.d,x.c*y.a+x.d*y.c,x.c*y.b+x.d*y.d}; }
    friend bool operator==(const Matrix& x, const Matrix& y) { return x.a==y.a && x.b==y.b && x.c==y.c && x.d==y.d; }
};
struct AddOnly { int value; AddOnly& operator+=(const AddOnly& x) { value += x.value; return *this; } };
void small_tests() {
    check_transforms({});
    for (int n = 0, cases = 1; n <= 8; ++n, cases *= 3) for (int code = 0; code < cases; ++code) {
        std::vector<long long> f(n+1); f[0] = LLONG_MIN; int digits = code;
        for (int i = 1; i <= n; ++i) { f[i] = digits % 3 - 1; digits /= 3; }
        check_transforms(f);
    }
    for (int n = 0, cases = 1; n <= 4; ++n, cases *= 3) for (int x = 0; x < cases; ++x) for (int y = 0; y < cases; ++y) {
        std::vector<long long> a(n+1), b(n+1); a[0]=LLONG_MAX; b[0]=LLONG_MIN;
        int ax=x, by=y; for (int i=1;i<=n;++i) { a[i]=ax%3-1; b[i]=by%3-1; ax/=3; by/=3; }
        check_convolutions(a,b);
    }
    std::mt19937 random(314016);
    for (int trial=0;trial<400;++trial) {
        std::vector<long long> a(random()%36),b(random()%36);
        for (auto& x:a) x=static_cast<int>(random()%19)-9;
        for (auto& x:b) x=static_cast<int>(random()%19)-9;
        check_transforms(a); check_convolutions(a,b);
        std::vector<ModInt> am(a.begin(),a.end()),bm(b.begin(),b.end());
        std::vector<Composite> ac(a.begin(),a.end()),bc(b.begin(),b.end());
        check_convolutions(am,bm); check_convolutions(ac,bc);
        auto dm=am; auto mc=ac; cp::divisor_mobius_transform(dm); cp::multiple_mobius_transform(mc);
        CHECK(dm==naive_transform(am,true,true) && mc==naive_transform(ac,false,true));
    }
    check_convolutions<long long>({},{}); check_convolutions<long long>({},{9,2,3});
    check_convolutions<long long>({9},{7}); check_convolutions<long long>({9,5},{});
    std::vector<Matrix> a{{0},{1,2,0,1},{0,1,1,0},{2,0,1,1}},b{{0},{1,0,1,2},{2,1,0,1}};
    check_convolutions(a,b);
    CHECK(!(cp::dirichlet_convolution(a,b)==cp::dirichlet_convolution(b,a)));
    std::vector<AddOnly> sum{{7},{1},{2},{3},{4}}; cp::divisor_zeta_transform(sum);
    CHECK(sum[0].value==7 && sum[4].value==7);
    cp::multiple_zeta_transform(sum); CHECK(sum[1].value==15);
    // 出力上限を超えるLCM/積を捨てる。GCDには大きい添字も寄与する。
    std::vector<long long> x{0,0,1,0},y{0,0,0,1};
    CHECK(cp::lcm_convolution(x,y)==std::vector<long long>(4));
    CHECK(cp::gcd_convolution(x,y)==std::vector<long long>({0,1,0,0}));
    x.resize(7);y.resize(7);CHECK(cp::lcm_convolution(x,y)[6]==1 && cp::dirichlet_convolution(x,y)[6]==1);
    std::cout << array_cases << " transform arrays; " << pair_cases << " convolution pairs; signed/modular/composite/noncommutative oracles passed\n";
}
template <class T> void small_integer_arithmetic() {
    const int lo=std::numeric_limits<T>::lowest(),hi=std::numeric_limits<T>::max();
    for (int a=lo;a<=hi;++a) for (int b=lo;b<=hi;++b) for (int kind=0;kind<3;++kind) {
        const int want=kind==0?a+b:kind==1?a-b:a*b;
        const auto operation=[&] { T x=static_cast<T>(a),y=static_cast<T>(b); if(kind==0)cp::detail::arithmetic_add_assign(x,y);else if(kind==1)cp::detail::arithmetic_subtract_assign(x,y);else x=cp::detail::arithmetic_multiply(x,y);return x;};
        if(want<lo||want>hi) throws<std::overflow_error>(operation);else CHECK(operation()==want);
    }
}
void boundaries() {
    small_integer_arithmetic<std::int8_t>();small_integer_arithmetic<std::uint8_t>();
    for (const auto& original:std::vector<std::vector<long long>>{{LLONG_MIN},{LLONG_MAX},{0,LLONG_MIN,0},{0,LLONG_MAX,0}}) {
        auto f=original;cp::divisor_zeta_transform(f);cp::divisor_mobius_transform(f);CHECK(f==original);
    }
    std::vector<long long> high{0,LLONG_MAX,1};throws<std::overflow_error>([&]{cp::divisor_zeta_transform(high);});
    std::vector<long long> low{0,LLONG_MIN,-1};throws<std::overflow_error>([&]{cp::multiple_zeta_transform(low);});
    std::vector<long long> under{0,1,LLONG_MIN};throws<std::overflow_error>([&]{cp::divisor_mobius_transform(under);});
    std::vector<unsigned long long> unsigned_under{0,1,0};throws<std::overflow_error>([&]{cp::divisor_mobius_transform(unsigned_under);});
    std::vector<long long> a{LLONG_MIN,LLONG_MIN},b{LLONG_MAX,-1},one{0,1};
    for(int kind=0;kind<3;++kind){throws<std::overflow_error>([&]{if(kind==0)cp::dirichlet_convolution(a,b);else if(kind==1)cp::gcd_convolution(a,b);else cp::lcm_convolution(a,b);});}
    CHECK(cp::dirichlet_convolution(a,one)[1]==LLONG_MIN && cp::gcd_convolution(a,one)[1]==LLONG_MIN && cp::lcm_convolution(a,one)[1]==LLONG_MIN);
    CHECK(a==std::vector<long long>({LLONG_MIN,LLONG_MIN}) && b==std::vector<long long>({LLONG_MAX,-1}));
    CHECK(cp::detail::arithmetic_bound(static_cast<std::size_t>(INT_MAX)+1)==INT_MAX);
    throws<std::length_error>([]{cp::detail::arithmetic_bound(static_cast<std::size_t>(INT_MAX)+2);});
    std::cout << "393216 small-integer arithmetic cases; extrema/overflow/empty/index-zero/bounds passed\n";
}
struct Counted {
    inline static long long adds=0,subtracts=0,multiplies=0;
    long long value;
    Counted(long long v=0):value(v){}
    Counted& operator+=(const Counted& x){++adds;value+=x.value;return *this;}
    Counted& operator-=(const Counted& x){++subtracts;value-=x.value;return *this;}
    friend Counted operator*(const Counted& a,const Counted& b){++multiplies;return Counted(a.value*b.value);}
    static void reset(){adds=subtracts=multiplies=0;}
};
void large_tests() {
    const int n=1000000;
    std::vector<int> phi(n+1),tau(n+1);std::iota(phi.begin(),phi.end(),0);
    for(int p=2;p<=n;++p)if(phi[p]==p)for(int k=p;k<=n;k+=p)phi[k]-=phi[k]/p;
    long long pair_count=0;for(int d=1;d<=n;++d)for(int k=d;k<=n;k+=d){++tau[k];++pair_count;}
    std::vector<long long> coprime(n+1);coprime[1]=1;for(int i=2;i<=n;++i)coprime[i]=coprime[i-1]+2LL*phi[i];
    std::vector<Counted> one(n+1,Counted(1));one[0]=Counted(0);
    const cp::PrimeSieve primes(n);long long steps=0;for(int p:primes.primes())steps+=n/p;
    for(int kind=0;kind<4;++kind){auto f=one;Counted::reset();if(kind==0)cp::divisor_zeta_transform(f);if(kind==1)cp::divisor_mobius_transform(f);if(kind==2)cp::multiple_zeta_transform(f);if(kind==3)cp::multiple_mobius_transform(f);CHECK(Counted::adds+Counted::subtracts==steps && Counted::multiplies==0);if(kind==0)for(int i=1;i<=n;++i)CHECK(f[i].value==tau[i]);}
    Counted::reset();auto gcd=cp::gcd_convolution(one,one);
    CHECK(Counted::adds==2*steps && Counted::subtracts==steps && Counted::multiplies==n);
    for(int i=1;i<=n;++i)CHECK(gcd[i].value==coprime[n/i]);
    Counted::reset();auto lcm=cp::lcm_convolution(one,one);
    CHECK(Counted::adds==2*steps && Counted::subtracts==steps && Counted::multiplies==n);
    const cp::SmallestPrimeFactor spf(n);
    for(int i=1;i<=n;++i){int x=i,expected=1;while(x>1){const int p=spf[x];int exponent=0;do{x/=p;++exponent;}while(x>1&&spf[x]==p);expected*=2*exponent+1;}CHECK(lcm[i].value==expected);}
    Counted::reset();auto dirichlet=cp::dirichlet_convolution(one,one);
    CHECK(Counted::adds==pair_count && Counted::multiplies==pair_count && Counted::subtracts==0);
    for(int i=1;i<=n;++i)CHECK(dirichlet[i].value==tau[i]);
    std::cout << "1-million bound; all transform/convolution entries and exact O(N log log N)/harmonic operation counts passed\n";
}
int main(){small_tests();boundaries();large_tests();}
