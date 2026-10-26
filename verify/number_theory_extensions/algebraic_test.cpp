#include "common.hpp"
#include "../../number_theory_basic/gaussian_integer.hpp"
#include "../../number_theory_basic/continued_fraction.hpp"
#include "../../number_theory_basic/pell_equation.hpp"
#include "../../number_theory_basic/sum_two_squares.hpp"
using cp::BigInt;using cp::BigRational;using cp::GaussianInteger;
int main(){
    for(int ar=-5;ar<=5;++ar)for(int ai=-5;ai<=5;++ai)for(int br=-5;br<=5;++br)for(int bi=-5;bi<=5;++bi){
        GaussianInteger a(ar,ai),b(br,bi);if(b==GaussianInteger{})continue;auto [q,r]=cp::gaussian_divmod(a,b);
        CHECK(a==b*q+r);CHECK(2*r.norm()<=b.norm());const auto g=cp::gaussian_gcd(a,b);CHECK(g.real>0&&g.imag>=0);CHECK(cp::gaussian_divmod(a,g).second==GaussianInteger{});CHECK(cp::gaussian_divmod(b,g).second==GaussianInteger{});CHECK(cp::gaussian_gcd(b,a)==g);
    }
    CHECK(cp::gaussian_gcd({}, {})==GaussianInteger{});
    throws<std::domain_error>([]{cp::gaussian_divmod(GaussianInteger(1),{});});
    const BigInt big("100000000000000000000000000000000000000");
    auto g=cp::gaussian_gcd(GaussianInteger(big,big),GaussianInteger(0,2*big));CHECK(g==GaussianInteger(big,big));
    for(int n=-30;n<=30;++n)for(int d=1;d<=13;++d){BigRational rational(n,d);const auto coefficients=cp::continued_fraction(rational);
        CHECK(cp::continued_fraction_convergents(coefficients).back()==rational);CHECK(coefficients.size()==1||coefficients.back()>1);
        for(int bound=1;bound<=8;++bound){const auto best=cp::best_rational_approximation(rational,bound);CHECK(best.denominator()<=bound);
            for(int q=1;q<=bound;++q){const int near=int(std::floor(double(n)*q/d));for(int p=near;p<=near+1;++p){const BigRational candidate(p,q);CHECK(cp::abs(best-rational)<=cp::abs(candidate-rational));if(cp::abs(best-rational)==cp::abs(candidate-rational)){CHECK(best.denominator()<=candidate.denominator());if(best.denominator()==candidate.denominator())CHECK(best<=candidate);}}}
        }
    }
    CHECK(cp::best_rational_approximation(BigRational(355,113),100)==BigRational(311,99));
    CHECK(cp::continued_fraction_convergents(cp::continued_fraction(BigRational(big+1,big))).back()==BigRational(big+1,big));
    CHECK(cp::best_rational_approximation(BigRational(big+1,big),big-1)==BigRational(big,big-1));
    throws<std::invalid_argument>([]{cp::continued_fraction_convergents({1,0});});
    throws<std::invalid_argument>([]{cp::best_rational_approximation(BigRational(1,2),0);});
    for(std::uint64_t D=2;D<=80;++D){const auto root=cp::integer_kth_root(D,2);if(root*root==D)continue;
        for(int sign:{-1,1}){auto solution=cp::pell_fundamental(D,sign);if(solution){CHECK(solution->x*solution->x-BigInt(D)*solution->y*solution->y==sign);CHECK(solution->y>0);}
            for(std::uint64_t y=1;y<=3000;++y){const auto value=D*y*y+sign,x=cp::integer_kth_root(value,2);if(x*x==value){CHECK(solution);CHECK(solution->y<=y);break;}if(solution)CHECK(solution->y!=y);}
            if(solution){auto power=cp::pell_solution_power(D,*solution,5);CHECK(power.x*power.x-BigInt(D)*power.y*power.y==sign);}
        }
    }
    auto p61=cp::pell_fundamental(61);CHECK(p61&&p61->x==1766319049&&p61->y==226153980);
    auto p661=cp::pell_fundamental(661);CHECK(p661&&p661->x==BigInt("16421658242965910275055840472270471049"));
    CHECK(!cp::pell_fundamental(3,-1));CHECK(cp::pell_solution_power(2,{3,2},0).x==1);
    throws<std::invalid_argument>([]{cp::pell_fundamental(4);});throws<std::invalid_argument>([]{cp::pell_fundamental(2,0);});
    throws<std::length_error>([]{cp::pell_fundamental(61,1,1);});throws<std::invalid_argument>([]{cp::pell_solution_power(2,{2,1},3);});
    for(std::uint64_t n=0;n<=3000;++n){std::vector<std::pair<std::uint64_t,std::uint64_t>> expected;
        for(std::uint64_t a=0;a*a<=n;++a)for(std::uint64_t b=a;a*a+b*b<=n;++b)if(a*a+b*b==n)expected.emplace_back(a,b);
        CHECK(cp::sum_two_squares(n)==expected);
    }
    for(std::uint64_t n:std::vector<std::uint64_t>{std::numeric_limits<std::uint64_t>::max(),18446744073709551557ULL,10000000000000000000ULL})
        for(auto [a,b]:cp::sum_two_squares(n)){CHECK(a<=b);CHECK(BigInt(a)*a+BigInt(b)*b==n);}
    CHECK(!cp::sum_two_squares(18446744073709551557ULL).empty());
    throws<std::length_error>([]{cp::sum_two_squares(5,1);});
    std::cout<<"algebraic: Gaussian norms/gcd, bounded rational optimum, Pell and all two-square representations passed\n";
}
