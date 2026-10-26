#include "common.hpp"
#include "combinatorics/q_binomial.hpp"
#include "combinatorics/stirling_small_prime.hpp"
#include "combinatorics/unlabeled_trees.hpp"
#include "combinatorics/combinatorial_transforms.hpp"
#include "combinatorics/stirling.hpp"
#include <string>
using U = std::uint64_t;
std::string canonical(const std::vector<std::vector<int>>& g, int v, int parent) {
    std::vector<std::string> children;
    for (int u : g[v]) if (u != parent) children.push_back(canonical(g, u, v));
    std::sort(children.begin(), children.end()); std::string result = "(";
    for (const auto& s : children) result += s;
    return result + ")";
}
int main() {
    std::size_t checks = 0;
    for (int p : {2,3,5,7,11,13,17,31,43}) for (int q = 0; q < p; ++q) {
        cp::QBinomial calc(p, q);
        std::vector<long long> row(101); row[0] = 1;
        long long factorial = 1, qi = 1, integer = 0;
        for (int n = 0; n <= 100; ++n) {
            for (int k = 0; k <= n + 1; ++k) { CHECK(calc(n, k) == row[k]); ++checks; }
            CHECK(calc.factorial(n) == factorial);
            std::vector<long long> next(102); long long power = 1;
            for (int k = 0; k <= n + 1; ++k) {
                next[k] = ((k ? row[k - 1] : 0) + power * row[k]) % p; power = power * q % p;
            }
            row = next; integer = (integer + qi) % p; qi = qi * q % p; factorial = factorial * integer % p;
        }
        const U n = std::numeric_limits<U>::max();
        CHECK(calc(n, 0) == 1); CHECK(calc(n, n) == 1);
        for (U k : {U(1), U(2), n / 2, n - 1}) {
            const auto expected = (calc(n - 1, k - 1) + cp::mod_pow(q, static_cast<long long>(k % (p - 1) + p - 1), p) * calc(n - 1, k)) % p;
            CHECK(calc(n, k) == expected);
        }
    }
    throws<std::invalid_argument>([]{cp::QBinomial x(9, 2);});
    throws<std::length_error>([]{cp::QBinomial x(1000000007, 2);});
    CHECK(cp::QBinomial(7, -1)(8, 4) == cp::QBinomial(7, 6)(8, 4));
    std::mt19937_64 random(710281);
    for (int p : {2,3,5,7,11,13,17,31}) {
        cp::StirlingSmallPrime s(p);
        auto first = cp::stirling_first(250, p), second = cp::stirling_second(250, p);
        for (int n = 0; n <= 250; ++n) for (int k = 0; k <= n; ++k) {
            CHECK(s.first(n, k) == first[n][k]); CHECK(s.second(n, k) == second[n][k]);
            CHECK(s.first(n, k, true) == (((n-k)&1) ? (p-first[n][k])%p : first[n][k])); ++checks;
        }
        for (int it = 0; it < 4000; ++it) {
            U n = random() | 1, k = it % 2 ? random() % n + 1 : n - random() % std::min<U>(n, 1000);
            CHECK(static_cast<U>(s.first(n, k)) == (s.first(n-1,k-1) + (n-1)%p * s.first(n-1,k))%p);
            CHECK(static_cast<U>(s.second(n, k)) == (s.second(n-1,k-1) + k%p * s.second(n-1,k))%p);
            ++checks;
        }
        const U n = std::numeric_limits<U>::max();
        CHECK(s.first(n,n)==1); CHECK(s.second(n,n)==1); CHECK(s.second(n,1)==1);
        CHECK(s.first(1,2)==0); CHECK(s.second(1,2)==0);
    }
    throws<std::invalid_argument>([]{cp::StirlingSmallPrime x(1);});
    throws<std::length_error>([]{cp::StirlingSmallPrime x(1000000007);});
    const auto trees = cp::unlabeled_tree_counts(60);
    const int rt[] = {0,1,1,2,4,9,20,48,115,286,719};
    const int ut[] = {0,1,1,1,2,3,6,11,23,47,106};
    for (int n = 0; n <= 10; ++n) { CHECK(trees.rooted[n] == cp::BigInt(rt[n])); CHECK(trees.unrooted[n] == cp::BigInt(ut[n])); }
    for (int n = 1; n <= 7; ++n) {
        std::set<std::string> rooted, unrooted;
        int codes = 1; for (int i = 0; i < n - 2; ++i) codes *= n;
        for (int code = 0; code < codes; ++code) {
            std::vector<int> degree(n, 1), prufer(std::max(0, n-2)); int v = code;
            for (auto& x : prufer) { x=v%n; v/=n; ++degree[x]; }
            std::vector<std::vector<int>> g(n);
            auto edge = [&](int a, int b) { g[a].push_back(b); g[b].push_back(a); };
            for (int x : prufer) { int leaf=0; while(degree[leaf]!=1)++leaf; edge(leaf,x); --degree[leaf]; --degree[x]; }
            if (n > 1) { int a=0; while(degree[a]!=1)++a; int b=a+1; while(degree[b]!=1)++b; edge(a,b); }
            std::string smallest;
            for(int r=0;r<n;++r){auto s=canonical(g,r,-1);rooted.insert(s);if(r==0||s<smallest)smallest=s;}
            unrooted.insert(smallest);
        }
        CHECK(trees.rooted[n] == cp::BigInt(rooted.size())); CHECK(trees.unrooted[n] == cp::BigInt(unrooted.size()));
    }
    throws<std::length_error>([]{cp::unlabeled_tree_counts(5,4);});
    constexpr int mod=998244353;
    using F=cp::fps_detail::Field<mod>;
    for(int length : {0,1,2,12,65,128}) {
        std::vector<int> a(length); for(auto& x:a)x=static_cast<int>(random()%mod);
        auto b=cp::lah_transform(a); auto back=cp::lah_transform(b,true); CHECK(back==a);
        std::vector<std::vector<int>> lah(length,std::vector<int>(length));
        if(length)lah[0][0]=1;
        for(int n=1;n<length;++n)for(int k=1;k<=n;++k)lah[n][k]=F::add(lah[n-1][k-1],F::mul(n+k-1,lah[n-1][k]));
        for(int n=0;n<length;++n){int expected=0;for(int k=0;k<=n;++k)expected=F::add(expected,F::mul(lah[n][k],a[k]));CHECK(b[n]==expected);}
        if(length)a[0]=0;
        CHECK(cp::euler_transform(cp::euler_transform(a),true)==a);
    }
    for(int trial=0;trial<30;++trial){
        std::vector<int> a(96), expected(96);expected[0]=1;
        for(int k=1;k<96;++k){a[k]=random()%4;for(int copy=0;copy<a[k];++copy)for(int j=k;j<96;++j)expected[j]=F::add(expected[j],expected[j-k]);}
        CHECK(cp::euler_transform(a)==expected);
    }
    std::vector<int> rooted_exponents(60);for(int i=1;i<60;++i)rooted_exponents[i]=(trees.rooted[i]%cp::BigInt(mod)).to_integral<int>();
    const auto forest=cp::euler_transform(rooted_exponents);
    for(int n=1;n<=60;++n)CHECK(forest[n-1]==(trees.rooted[n]%cp::BigInt(mod)).to_integral<int>());
    throws<std::invalid_argument>([]{cp::euler_transform({1,2});});
    throws<std::invalid_argument>([]{cp::euler_transform({0,2},true);});
    CHECK(cp::lah_transform(std::vector<int>{-1})[0]==mod-1);
    CHECK(cp::euler_transform(std::vector<int>{}) .empty());
    std::cout<<"counting: "<<checks<<" q/Stirling comparisons, tree isomorphism enumeration and transform oracles passed\n";
}
