#include "common.hpp"
#include "combinatorics/rsk.hpp"
#include "combinatorics/hook_length.hpp"
#include <string>
int main(){
    std::size_t comparisons=0;std::mt19937 random(779);
    for(int n=0;n<=7;++n){std::vector<int> p(n);std::iota(p.begin(),p.end(),1);
        do{auto result=cp::rsk_word(p);CHECK(cp::is_standard_young_tableau(result.insertion));CHECK(cp::is_standard_young_tableau(result.recording));CHECK(cp::inverse_rsk_word(result)==p);++comparisons;}while(std::next_permutation(p.begin(),p.end()));
    }
    for(int n=0;n<=8;++n){int bound=1;for(int i=0;i<n;++i)bound*=3;
        for(int mask=0;mask<bound;++mask){int code=mask;std::vector<int>w(n),dp(n,1);for(auto&x:w){x=code%3;code/=3;}
            int longest=0;for(int i=0;i<n;++i){for(int j=0;j<i;++j)if(w[j]<=w[i])dp[i]=std::max(dp[i],dp[j]+1);longest=std::max(longest,dp[i]);}
            auto pair=cp::rsk_word(w);CHECK(cp::inverse_rsk_word(pair)==w);CHECK(cp::is_semistandard_young_tableau(pair.insertion));CHECK(cp::is_standard_young_tableau(pair.recording));
            CHECK(longest==(pair.insertion.empty()?0:static_cast<int>(pair.insertion[0].size())));++comparisons;
        }
    }
    for(int it=0;it<400;++it){std::size_t rows=random()%6,cols=random()%6;
        std::vector<std::vector<std::uint64_t>>m(rows,std::vector<std::uint64_t>(cols));for(auto&r:m)for(auto&x:r)x=random()%3;
        const auto pair=cp::rsk_matrix(m);CHECK(cp::inverse_rsk_matrix(pair,rows,cols)==m);
        auto transposed=std::vector<std::vector<std::uint64_t>>(cols,std::vector<std::uint64_t>(rows));
        for(std::size_t r=0;r<rows;++r)for(std::size_t c=0;c<cols;++c)transposed[c][r]=m[r][c];
        auto q=cp::rsk_matrix(transposed);CHECK(q.insertion==pair.recording);CHECK(q.recording==pair.insertion);++comparisons;
    }
    CHECK(cp::inverse_rsk_word(cp::rsk_word(std::vector<std::string>{"pear","apple","apple"}))==std::vector<std::string>({"pear","apple","apple"}));
    CHECK(cp::inverse_rsk_word(cp::rsk_word(std::vector<int>{3,1,2},std::greater<int>{}),std::greater<int>{})==std::vector<int>({3,1,2}));
    for(int n=0;n<=9;++n){cp::BigInt sum=0,factorial=1;for(int i=1;i<=n;++i)factorial*=cp::BigInt(i);
        cp::YoungShape shape;
        std::function<void(int,int)> partitions=[&](int left,int last){
            if(left){for(int x=std::min(left,last);x>0;--x){shape.push_back(x);partitions(left-x,x);shape.pop_back();}return;}
            std::set<cp::YoungTableau<std::size_t>> seen;
            const auto count=cp::enumerate_standard_young_tableaux(shape,[&](const auto&t){CHECK(cp::is_standard_young_tableau(t));CHECK(seen.insert(t).second);CHECK(cp::transpose_young_tableau(cp::transpose_young_tableau(t))==t);return true;});
            const auto exact=cp::standard_young_tableaux_count(shape);CHECK(exact==cp::BigInt(count));CHECK(cp::standard_young_tableaux_count(cp::conjugate_young_shape(shape))==exact);sum+=exact*exact;
            for(std::uint64_t mod:{1ULL,2ULL,4ULL,7ULL,12ULL,998244353ULL,18446744073709551615ULL})CHECK(cp::standard_young_tableaux_count_mod(shape,mod)==(exact%cp::BigInt(mod)).to_integral<std::uint64_t>());
            if(n<=6)for(int alphabet=0;alphabet<=4;++alphabet){std::vector<std::pair<int,int>>cells;cp::YoungTableau<int>t(shape.size());for(std::size_t r=0;r<shape.size();++r){t[r].resize(shape[r]);for(int c=0;c<shape[r];++c)cells.emplace_back(r,c);}
                int cnt=0;std::function<void(std::size_t)> fill=[&](std::size_t i){if(i==cells.size()){++cnt;return;}auto[r,c]=cells[i];int lo=1;if(c)lo=std::max(lo,t[r][c-1]);if(r)lo=std::max(lo,t[r-1][c]+1);for(int x=lo;x<=alphabet;++x){t[r][c]=x;fill(i+1);}};fill(0);
                CHECK(cp::semistandard_young_tableaux_count(shape,alphabet)==cp::BigInt(cnt));}
            ++comparisons;
        };partitions(n,n);CHECK(sum==factorial);
    }
    CHECK(cp::young_tableau_descents(cp::YoungTableau<int>{{1,3,5},{2,4}})==std::vector<std::size_t>({1,3}));
    CHECK(cp::enumerate_standard_young_tableaux({3,2},[](const auto&){return false;},1)==1);
    throws<std::length_error>([]{cp::enumerate_standard_young_tableaux({3,2},[](const auto&){return true;},1);});
    throws<std::invalid_argument>([]{cp::standard_young_tableaux_count({2,3});});
    throws<std::invalid_argument>([]{cp::young_shape_size({0});});
    throws<std::length_error>([]{cp::young_shape_size({100},99);});
    throws<std::invalid_argument>([]{cp::standard_young_tableaux_count_mod({2,1},0);});
    throws<std::invalid_argument>([]{cp::rsk_biword(std::vector<std::pair<int,int>>{{1,2},{1,1}});});
    throws<std::invalid_argument>([]{cp::inverse_rsk_word(cp::RSKPair<std::size_t,int>{{{1}},{{2}}});});
    throws<std::invalid_argument>([]{cp::inverse_rsk_biword(cp::RSKPair<int,int>{{{1,1}},{{1},{1}}});});
    throws<std::invalid_argument>([]{cp::rsk_matrix({{1},{1,2}});});
    throws<std::length_error>([]{cp::rsk_matrix({{2}},1);});
    throws<std::out_of_range>([]{cp::inverse_rsk_matrix(cp::rsk_matrix({{1}}),0,0);});
    CHECK(!cp::is_standard_young_tableau(cp::YoungTableau<int>{{-1}}));
    CHECK(!cp::is_standard_young_tableau(cp::YoungTableau<int>{{1,1}}));
    CHECK(!cp::is_semistandard_young_tableau(cp::YoungTableau<int>{{1},{1}}));
    CHECK(cp::semistandard_young_tableaux_count({2},std::numeric_limits<std::uint64_t>::max())>cp::BigInt(std::numeric_limits<std::uint64_t>::max()));
    std::vector<int> increasing(10000);std::iota(increasing.begin(),increasing.end(),0);CHECK(cp::inverse_rsk_word(cp::rsk_word(increasing))==increasing);
    cp::YoungShape tall(10000,1);CHECK(cp::enumerate_standard_young_tableaux(tall,[](const auto&t){return cp::is_standard_young_tableau(t);})==1);
    std::cout<<"tableaux: "<<comparisons<<" RSK/enumeration comparisons, hook-content brute force and n! identity passed\n";
}
