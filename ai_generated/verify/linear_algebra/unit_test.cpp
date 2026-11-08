#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>
#include "linear_algebra/all.hpp"
#include "linear_recurrence/all.hpp"
#include "modint.hpp"
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
template <class E, class F> void rejects(F f) { bool caught=false;try{f();}catch(const E&){caught=true;}require(caught,"missing exception"); }
int norm(long long x,int p){return static_cast<int>((x%p+p)%p);}
struct Rational {
    long long n,d;
    Rational(long long a=0,long long b=1):n(a),d(b){if(d<0){n=-n;d=-d;}const auto g=std::gcd(n,d);n/=g;d/=g;}
    Rational& operator+=(const Rational& x){*this=Rational(n*x.d+x.n*d,d*x.d);return *this;}
    Rational& operator-=(const Rational& x){*this=Rational(n*x.d-x.n*d,d*x.d);return *this;}
    friend Rational operator*(const Rational&a,const Rational&b){return {a.n*b.n,a.d*b.d};}
    friend Rational operator/(const Rational&a,const Rational&b){return {a.n*b.d,a.d*b.n};}
    friend bool operator==(const Rational&a,const Rational&b){return a.n==b.n&&a.d==b.d;}
};
struct Count {
    long long value;static std::size_t additions,products;
    Count(long long x=0):value(x){}
    Count&operator+=(const Count&x){++additions;value+=x.value;return *this;}
    friend Count operator*(const Count&a,const Count&b){++products;return a.value*b.value;}
};
std::size_t Count::additions=0,Count::products=0;
void matrices(std::mt19937_64& rng){
    std::size_t count=0;
    for(std::size_t n=0;n<=4;++n)for(std::size_t k=0;k<=4;++k)for(std::size_t m=0;m<=4;++m)for(int trial=0;trial<4;++trial){
        cp::Matrix<long long>a(n,k),b(k,m);for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<k;++j)a(i,j)=static_cast<int>(rng()%11)-5;
        for(std::size_t i=0;i<k;++i)for(std::size_t j=0;j<m;++j)b(i,j)=static_cast<int>(rng()%11)-5;
        const auto saved_a=a,saved_b=b,c=a*b;require(c.rows()==n&&c.cols()==m,"product shape");
        for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<m;++j){long long direct=0;for(std::size_t t=0;t<k;++t)direct+=a(i,t)*b(t,j);require(c(i,j)==direct,"rectangular direct dot product");}
        require(a==saved_a&&b==saved_b&&a.transposed().transposed()==a,"matrix immutability and transpose");
        std::vector<long long>x(k,2);const auto y=cp::matrix_vector_multiply(a,x);require(y.size()==n,"vector output shape");
        for(std::size_t i=0;i<n;++i){long long direct=0;for(std::size_t j=0;j<k;++j)direct+=a(i,j)*x[j];require(y[i]==direct,"column-vector direction");}++count;
    }
    cp::Matrix<int> a{{1,2},{3,4}};auto b=a;b.at(0,1)=9;require(a.at(0,1)==2&&b.at(0,1)==9,"matrix value semantics");
    auto moved=std::move(b);require(moved.at(0,1)==9&&b.rows()==0&&b.cols()==0,"move constructor retains valid empty source");
    b=std::move(moved);require(b.at(0,1)==9&&moved.rows()==0&&moved.cols()==0,"move assignment retains valid empty source");
    auto& same=b;b=std::move(same);require(b.at(0,1)==9,"self move assignment");
    rejects<std::invalid_argument>([]{cp::Matrix<int>bad{{1,2},{3}};});
    rejects<std::invalid_argument>([]{cp::Matrix<int>bad(std::vector<std::vector<int>>{{1},{2,3}});});
    rejects<std::length_error>([]{cp::Matrix<int>bad(std::numeric_limits<std::size_t>::max(),2);});
    rejects<std::out_of_range>([&]{a.at(2,0);});const auto& ca=a;rejects<std::out_of_range>([&]{ca.at(0,2);});
    rejects<std::invalid_argument>([]{cp::matrix_multiply(cp::Matrix<int>(2,3),cp::Matrix<int>(2,1));});
    rejects<std::invalid_argument>([]{cp::matrix_vector_multiply(cp::Matrix<int>(0,2),std::vector<int>{1});});
    rejects<std::invalid_argument>([]{cp::matrix_power(cp::Matrix<int>(0,2),0);});
    require(cp::Matrix<int>(0,3)!=cp::Matrix<int>(0,4),"empty dimension retained");
    using C=std::complex<long long>;require((cp::Matrix<C>{{C(1,1)}}*cp::Matrix<C>{{C(1,-1)}})(0,0)==C(2,0),"complex product");
    cp::Matrix<Count> large(64,80,Count(1));Count::additions=Count::products=0;auto c=large*cp::Matrix<Count>(80,72,Count(1));
    require(Count::products==64*80*72&&Count::additions==Count::products,"exact product arithmetic count");for(const auto& x:c.data())require(x.value==80,"large product all outputs");
    std::cout<<count<<" rectangular integer products; shape/value semantics/transpose/vector/generic arithmetic passed\n";
}
template<int P>void powers(std::mt19937_64&rng){
    using F=TestModInt<P>;
    for(int trial=0;trial<50;++trial){const std::size_t n=rng()%6;cp::Matrix<F>a(n,n);for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)a(i,j)=static_cast<int>(rng()%41)-20;
        auto expected=cp::Matrix<F>::identity(n);for(int k=0;k<=12;++k){require(cp::matrix_power(a,k)==expected,"power vs repeated multiplication");expected=expected*a;}
    }
    cp::Matrix<F> swap{{0,1},{1,0}};require(cp::matrix_power(swap,std::numeric_limits<std::uint64_t>::max())==swap,"uint64 maximum exponent");
    auto empty=cp::matrix_power(cp::Matrix<F>(),std::numeric_limits<std::uint64_t>::max());require(empty.rows()==0&&empty.cols()==0,"empty square power");
    cp::Matrix<F> cycle(32,32);for(int i=0;i<32;++i)cycle(i,(i+1)%32)=1;
    auto huge=cp::matrix_power(cycle,std::numeric_limits<std::uint64_t>::max());
    for(int i=0;i<32;++i)for(int j=0;j<32;++j)require(huge(i,j)==F(j==(i+31)%32),"large cycle power by direct index permutation");
}
template<int P>std::size_t systems(){
    std::size_t total=0;const int max_size=P==2?3:2;
    for(int n=0;n<=max_size;++n)for(int m=0;m<=max_size;++m){int tables=1;for(int i=0;i<n*(m+1);++i)tables*=P;
        for(int code=0;code<tables;++code){int value=code;cp::Matrix<int>a(n,m);std::vector<int>b(n);for(int i=0;i<n;++i){for(int j=0;j<m;++j){a(i,j)=value%P;value/=P;}b[i]=value%P;value/=P;}
            auto saved=a;auto result=cp::gauss_jordan_mod<P>(a,b);require(saved==a,"solver input preserved");
            std::set<std::vector<int>>solutions,homogeneous;int candidates=1;for(int i=0;i<m;++i)candidates*=P;
            for(int xcode=0;xcode<candidates;++xcode){std::vector<int>x(m);int v=xcode;for(auto&t:x){t=v%P;v/=P;}bool fits=true,zero=true;for(int i=0;i<n;++i){int sum=0;for(int j=0;j<m;++j)sum=norm(sum+1LL*a(i,j)*x[j],P);fits&=sum==b[i];zero&=sum==0;}if(fits)solutions.insert(x);if(zero)homogeneous.insert(x);}
            std::size_t null_dimension=0;for(auto size=homogeneous.size();size>1;size/=P)++null_dimension;
            require(result.rank==static_cast<std::size_t>(m)-null_dimension,"rank from enumerated homogeneous solutions");
            require(result.pivot_columns.size()==result.rank&&result.reduced.rows()==static_cast<std::size_t>(n)&&result.reduced.cols()==static_cast<std::size_t>(m+1),"reduced dimensions");
            for(std::size_t i=0;i<result.rank;++i)for(int row=0;row<n;++row)require(result.reduced(row,result.pivot_columns[i])==(static_cast<std::size_t>(row)==i),"normalized pivot columns");
            require(result.has_solution()==!solutions.empty(),"solution existence by exhaustive assignment");
            if(!result.has_solution()){require(result.particular.empty()&&result.nullspace_basis.empty(),"no solution has no vectors");++total;continue;}
            require(result.nullspace_basis.size()==null_dimension,"nullity and basis count");
            require((result.status==cp::LinearSystemStatus::unique)==(solutions.size()==1),"unique/multiple");
            std::set<std::vector<int>>generated;int combinations=1;for(std::size_t i=0;i<null_dimension;++i)combinations*=P;
            for(int cc=0;cc<combinations;++cc){auto x=result.particular;int v=cc;for(const auto& direction:result.nullspace_basis){const int scale=v%P;v/=P;for(int j=0;j<m;++j)x[j]=norm(x[j]+scale*direction[j],P);}generated.insert(x);}
            require(generated==solutions,"entire affine solution space and basis independence");++total;
        }
    }
    return total;
}
void real_systems(std::mt19937_64&rng){
    for(int trial=0;trial<160;++trial){const std::size_t n=1+rng()%12;cp::Matrix<double>a(n,n);std::vector<double>x(n);for(auto&v:x)v=static_cast<int>(rng()%41)-20;
        for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)a(i,j)=i==j?100:static_cast<int>(rng()%11)-5;
        const auto b=cp::matrix_vector_multiply(a,x);auto r=cp::gauss_jordan(a,b);require(r.status==cp::LinearSystemStatus::unique&&r.rank==n,"diagonally dominant solve");for(std::size_t i=0;i<n;++i)require(std::abs(r.particular[i]-x[i])<1e-8,"known floating solution");
    }
    auto swap=cp::gauss_jordan(cp::Matrix<double>{{1e-20,1},{1,1}},std::vector<double>{2,3});require(std::abs(swap.particular[0]-1)<1e-10,"partial pivot chooses large value");
    auto small_factor=cp::gauss_jordan(cp::Matrix<double>{{1,1e20},{1e-20,0}},std::vector<double>{1,0});require(small_factor.rank==2&&std::abs(small_factor.particular[1]-1e-20)<1e-30,"tiny elimination factor has a large contribution");
    auto tiny=cp::gauss_jordan(cp::Matrix<double>{{1e-12}},std::vector<double>{0});require(tiny.rank==0,"absolute epsilon drops tiny coefficient");require(cp::gauss_jordan(cp::Matrix<double>{{1e-12}},std::vector<double>{0},1e-14).rank==1,"explicit epsilon");
    auto multi=cp::gauss_jordan(cp::Matrix<double>{{1,2,3},{2,4,6}},std::vector<double>{4,8});require(multi.rank==1&&multi.nullspace_basis.size()==2,"real dependent rows");for(auto&v:multi.nullspace_basis){auto y=cp::matrix_vector_multiply(cp::Matrix<double>{{1,2,3},{2,4,6}},v);for(auto x:y)require(std::abs(x)<1e-10,"real nullspace residual");}
    require(!cp::gauss_jordan(cp::Matrix<double>{{1,2},{2,4}},std::vector<double>{1,3}).has_solution(),"inconsistent real system");
    auto exact=cp::gauss_jordan(cp::Matrix<Rational>{{2,1},{1,-1}},std::vector<Rational>{1,0});require(exact.particular==std::vector<Rational>({{1,3},{1,3}}),"rational exact field");
    auto prime=cp::gauss_jordan(cp::Matrix<TestModInt<7>>{{2,1},{1,2}},std::vector<TestModInt<7>>{1,0});require(prime.has_solution()&&prime.particular==std::vector<TestModInt<7>>({3,2}),"custom finite field");
    auto noeq=cp::gauss_jordan(cp::Matrix<double>(0,3),std::vector<double>{});require(noeq.rank==0&&noeq.nullspace_basis.size()==3&&noeq.particular.size()==3,"zero equations preserve all variables");
    cp::Matrix<int> triangular(128,128);std::vector<int> sums(128);for(int i=0;i<128;++i){sums[i]=i+1;for(int j=0;j<=i;++j)triangular(i,j)=1;}
    auto dense=cp::gauss_jordan_mod(triangular,sums);require(dense.status==cp::LinearSystemStatus::unique&&dense.particular==std::vector<int>(128,1),"128-variable elimination");
    auto allfree=cp::gauss_jordan_mod(cp::Matrix<int>(0,256),{});require(allfree.nullspace_basis.size()==256,"explicit nullspace output when there are no equations");
    require(cp::gauss_jordan(cp::Matrix<double>(2,0),std::vector<double>{0,0}).status==cp::LinearSystemStatus::unique,"zero variables unique empty solution");
    require(!cp::gauss_jordan(cp::Matrix<double>(2,0),std::vector<double>{0,1}).has_solution(),"zero variables contradiction");
    rejects<std::invalid_argument>([]{cp::gauss_jordan(cp::Matrix<double>(2,3),std::vector<double>{0});});
    for(double epsilon:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})rejects<std::invalid_argument>([&]{cp::gauss_jordan(cp::Matrix<double>(),std::vector<double>{},epsilon);});
    for(double bad:{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){
        rejects<std::invalid_argument>([&]{cp::gauss_jordan(cp::Matrix<double>{{bad}},std::vector<double>{0});});
        rejects<std::invalid_argument>([&]{cp::gauss_jordan(cp::Matrix<double>{{1}},std::vector<double>{bad});});
        rejects<std::invalid_argument>([&]{cp::matrix_power(cp::Matrix<double>{{bad}},0);});
        rejects<std::invalid_argument>([&]{cp::matrix_multiply(cp::Matrix<double>{{bad}},cp::Matrix<double>{{0}});});
    }
    rejects<std::overflow_error>([]{cp::gauss_jordan(cp::Matrix<double>{{1e-100}},std::vector<double>{1e300},1e-110);});
    rejects<std::overflow_error>([]{cp::matrix_multiply(cp::Matrix<double>{{1e300}},cp::Matrix<double>{{1e300}});});
    std::cout<<"160 real systems; rational/custom field, pivots, nullspace, epsilon, empty dimensions and nonfinite rejection passed\n";
}
void integer_limits(){
    using L=long long;const auto hi=std::numeric_limits<L>::max(),lo=std::numeric_limits<L>::lowest();
    require(cp::matrix_power(cp::Matrix<L>{{hi}},1)(0,0)==hi,"no unused square overflow");
    require(cp::matrix_multiply(cp::Matrix<L>{{lo}},cp::Matrix<L>{{1}})(0,0)==lo,"lowest integral product");
    rejects<std::overflow_error>([&]{cp::matrix_multiply(cp::Matrix<L>{{lo}},cp::Matrix<L>{{-1}});});
    rejects<std::overflow_error>([&]{cp::matrix_multiply(cp::Matrix<L>{{hi,1}},cp::Matrix<L>{{1},{1}});});
    rejects<std::overflow_error>([&]{cp::matrix_multiply(cp::Matrix<L>{{hi,1,-1}},cp::Matrix<L>{{1},{1},{1}});});
    rejects<std::overflow_error>([&]{cp::matrix_power(cp::Matrix<L>{{hi}},2);});
    const auto uh=std::numeric_limits<unsigned long long>::max();require(cp::matrix_power(cp::Matrix<unsigned long long>{{uh}},1)(0,0)==uh,"unsigned max");
    rejects<std::overflow_error>([&]{cp::matrix_multiply(cp::Matrix<unsigned long long>{{uh}},cp::Matrix<unsigned long long>{{2}});});
    for(int a=-128;a<=127;++a)for(int b=-128;b<=127;++b){const int product=a*b;bool overflow=false;try{auto c=cp::matrix_multiply(cp::Matrix<signed char>{{static_cast<signed char>(a)}},cp::Matrix<signed char>{{static_cast<signed char>(b)}});require(product>=-128&&product<=127&&c(0,0)==product,"int8 product exact");}catch(const std::overflow_error&){overflow=true;}require(overflow==(product< -128||product>127),"int8 product checked");}
    std::cout<<"65536 int8 products; signed/unsigned extremes and intermediate overflow passed\n";
}
std::set<std::uint64_t> span(const std::vector<std::uint64_t>&a){std::set<std::uint64_t>s{0};for(auto x:a){auto before=s;for(auto y:before)s.insert(x^y);}return s;}
void xor_tests(std::mt19937_64&rng){
    std::size_t cases=0;
    for(int n=0,limit=1;n<=5;++n,limit*=8)for(int code=0;code<limit;++code){std::vector<std::uint64_t>a;cp::XorBasis<std::uint8_t>b;int v=code;std::set<std::uint64_t>s{0};
        for(int i=0;i<n;++i){auto x=static_cast<std::uint8_t>(v%8);v/=8;const auto before=b.basis();const bool independent=!s.count(x);require(b.insert(x)==independent,"insert independence");if(!independent)require(b.basis()==before,"dependent insertion unchanged");a.push_back(x);s=span(a);}
        require(s.size()==(std::size_t(1)<<b.rank()),"dimension from enumerated span");
        for(int seed=0;seed<16;++seed){std::uint64_t low=255,high=0;for(auto x:s){low=std::min(low,x^seed);high=std::max(high,x^seed);}require(b.min_xor(seed)==low&&b.max_xor(seed)==high&&b.contains(seed)==bool(s.count(seed)),"all XOR queries by subsets");}++cases;
    }
    for(int trial=0;trial<100;++trial){std::vector<std::uint64_t>a(1+rng()%12);for(auto&x:a)x=rng();auto s=span(a);cp::XorBasis<>b,other;for(std::size_t i=0;i<a.size();++i)(i%2?b:other).insert(a[i]);int before=b.rank(),added=b.merge(other);require(b.rank()==before+added,"merge dimensions");require(b.merge(b)==0,"self merge");for(int q=0;q<12;++q){auto seed=rng(),low=seed,high=seed;for(auto x:s){low=std::min(low,seed^x);high=std::max(high,seed^x);}require(b.min_xor(seed)==low&&b.max_xor(seed)==high,"full-width XOR subsets");}for(auto x:s)require(b.contains(x),"full-width contains span");}
    cp::XorBasis<>full;for(int i=0;i<64;++i)require(full.insert(std::uint64_t(1)<<i),"all64 independent pivots");require(full.rank()==64&&full.max_xor()==std::numeric_limits<std::uint64_t>::max()&&full.min_xor(~std::uint64_t(0))==0,"rank64 without shifting64");
    for(int i=0;i<100000;++i)require(!full.insert(rng()),"large redundant history");
    full.clear();require(full.empty()&&full.rank()==0&&full.contains(0)&&!full.contains(1)&&full.max_xor(13)==13,"clear and empty queries");
    require(cp::matrix_power_mod<17>({{1,1},{1,0}},10)[0][1]==4,"matrix recurrence API retained");
    std::cout<<cases<<" short XOR sequences, 100 full-width histories, merge/self/64 pivots/100000 redundant insertions passed\n";
}
}
int main(){std::mt19937_64 rng(320020);matrices(rng);powers<1>(rng);powers<12>(rng);powers<998244353>(rng);std::cout<<"150 modular matrix histories / 1950 powers by repeated products passed\n";auto count=systems<2>()+systems<3>();std::cout<<count<<" finite-field systems: full solution sets and homogeneous dimensions passed\n";real_systems(rng);integer_limits();xor_tests(rng);}
