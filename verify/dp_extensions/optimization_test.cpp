#include "common.hpp"
#include "dp_optimization/monge_matrix_product.hpp"
#include "dp_optimization/weighted_slope_trick.hpp"
#include "dp_optimization/slope_trick.hpp"
#include "dp_optimization/dynamic_finite_state_dp.hpp"
using namespace cp;
int main(){std::mt19937 rng(10430);
    for(int trial=0;trial<600;++trial){int n=int(rng()%12),k=int(rng()%13),m=int(rng()%14);std::vector<std::vector<long long>>a(n,std::vector<long long>(k)),b(k,std::vector<long long>(m));for(auto&row:a)for(auto&x:row)x=int(rng()%101)-50;std::vector<int>x(k),y(m);int cur=0;for(auto&v:x){cur+=int(rng()%5);v=cur;}cur=0;for(auto&v:y){cur+=int(rng()%5);v=cur;}for(int i=0;i<k;++i)for(int j=0;j<m;++j)b[i][j]=1LL*(x[i]-y[j])*(x[i]-y[j])+3*i-5*j;
        auto result=monge_min_plus_product(n,k,m,[&](int i,int j){return a[i][j];},[&](int i,int j){return b[i][j];});
        for(int i=0;i<n;++i)for(int j=0;j<m;++j){std::optional<long long>best;int arg=-1;for(int q=0;q<k;++q)if(!best||a[i][q]+b[q][j]<*best){best=a[i][q]+b[q][j];arg=q;}CHECK(result.value[i][j]==best&&result.argmin[i][j]==arg);}
    }
    std::size_t calls=0;auto large=monge_min_plus_product(30,1000,1200,[&](int i,int k){++calls;return 1LL*(i-k)*(i-k);},[](int k,int j){return 1LL*(k-j)*(k-j);});CHECK(large.value.size()==30&&calls<30U*2200*20);
    for(int trial=0;trial<120;++trial){WeightedSlopeTrick actual;SlopeTrick unit;std::vector<long long>values(401);
        for(int step=0;step<40;++step){int kind=int(rng()%7),a=int(rng()%21)-10,w=int(rng()%5);if(kind<=2){if(kind==0){actual.add_abs(a,w);for(int q=0;q<w;++q)unit.add_abs(a);}if(kind==1){actual.add_a_minus_x(a,w);for(int q=0;q<w;++q)unit.add_a_minus_x(a);}if(kind==2){actual.add_x_minus_a(a,w);for(int q=0;q<w;++q)unit.add_x_minus_a(a);}for(int x=-200;x<=200;++x)values[x+200]+=w*(kind==0?std::abs(x-a):kind==1?std::max(0,a-x):std::max(0,x-a));}
            else if(kind==3){actual.prefix_min();unit.prefix_min();for(int i=1;i<=400;++i)values[i]=std::min(values[i],values[i-1]);}
            else if(kind==4){actual.suffix_min();unit.suffix_min();for(int i=399;i>=0;--i)values[i]=std::min(values[i],values[i+1]);}
            else if(kind==5){actual.add_constant(a);unit.add_constant(a);for(auto&x:values)x+=a;}
            else {actual.reflect();std::reverse(values.begin(),values.end());for(int x=-40;x<=40;++x)CHECK(actual.evaluate(x)==values[x+200]);actual.reflect();std::reverse(values.begin(),values.end());}
            CHECK(actual.minimum()==unit.minimum());for(int x=-30;x<=30;++x)CHECK(actual.evaluate(x)==values[x+200]&&actual.evaluate(x)==unit.evaluate(x));auto bounds=actual.argmin();if(bounds.first)CHECK(actual.evaluate(*bounds.first)==actual.minimum());if(bounds.second)CHECK(actual.evaluate(*bounds.second)==actual.minimum());
        }
        actual.shift(-3,4);unit.shift(-3,4);for(int x=-30;x<=30;++x){long long expected=LLONG_MAX;for(int y=x-4;y<=x+3;++y)expected=std::min(expected,values[y+200]);CHECK(actual.evaluate(x)==expected&&actual.evaluate(x)==unit.evaluate(x));}
        auto other=actual;actual.merge(other);for(int x=-30;x<=30;++x)CHECK(actual.evaluate(x)==2*unit.evaluate(x));actual.merge(actual);for(int x=-30;x<=30;++x)CHECK(actual.evaluate(x)==4*unit.evaluate(x));
    }
    WeightedSlopeTrick huge;huge.add_abs(LLONG_MIN,BigInt("1000000000000000000000000000000"));huge.add_abs(LLONG_MAX,BigInt("1000000000000000000000000000000"));CHECK(huge.breakpoint_count()<=4);CHECK(huge.minimum()==BigInt("18446744073709551615000000000000000000000000000000"));huge.reflect();CHECK(huge.evaluate(0)==huge.minimum());throws<std::invalid_argument>([&]{huge.add_abs(0,-1);});
    WeightedSlopeTrick alias;alias.add_abs(-10);alias.add_abs(10);auto saved=alias;const auto point=alias.minimum();alias.add_abs(alias.minimum(),alias.minimum());saved.add_abs(point,point);for(int x=-30;x<=30;++x)CHECK(alias.evaluate(x)==saved.evaluate(x));
    using M=DynamicFiniteStateDP::Matrix;
    for(int trial=0;trial<100;++trial){int n=int(rng()%18),k=1+int(rng()%5);auto random_matrix=[&](){M a(k,std::vector<std::optional<long long>>(k));for(auto&row:a)for(auto&x:row)if(rng()%3)x=int(rng()%17)-8;return a;};std::vector<M>a;for(int i=0;i<n;++i)a.push_back(random_matrix());DynamicFiniteStateDP dp(k,a);
        for(int step=0;step<60;++step){if(n&&rng()%2){int p=int(rng()%n);a[p]=random_matrix();dp.set(p,a[p]);}int l=int(rng()%unsigned(n+1)),r=int(rng()%unsigned(n+1));if(l>r)std::swap(l,r);auto product=dp.product(l,r);
            for(int source=0;source<k;++source){std::vector<std::optional<long long>>d(k);d[source]=0;for(int p=l;p<r;++p){std::vector<std::optional<long long>>next(k);for(int i=0;i<k;++i)if(d[i])for(int j=0;j<k;++j)if(a[p][i][j]){auto v=*d[i]+*a[p][i][j];if(!next[j]||v<*next[j])next[j]=v;}d=next;}CHECK(product[source]==d);for(int target=0;target<k;++target){auto path=dp.restore(l,r,source,target);CHECK(bool(path)==bool(d[target]));if(path){CHECK(path->cost==*d[target]&&path->states.size()==std::size_t(r-l+1));CHECK(path->states.front()==source&&path->states.back()==target);long long cost=0;for(int p=l;p<r;++p){auto edge=a[p][path->states[p-l]][path->states[p-l+1]];CHECK(edge);cost+=*edge;}CHECK(cost==path->cost);}}}
        }
    }
    DynamicFiniteStateDP safe(1,{M{{LLONG_MAX}},M{{0}}});throws<std::overflow_error>([&]{safe.set(1,M{{1}});});CHECK(safe.all_product()[0][0]==LLONG_MAX&&safe.product(1,2)[0][0]==0);
    throws<std::invalid_argument>([]{DynamicFiniteStateDP(0,{});});throws<std::overflow_error>([]{monge_min_plus_product(1,1,1,[](int,int){return LLONG_MAX;},[](int,int){return 1;});});
    std::cout<<"optimization: 600 Monge cubic oracles and call bound, 4800 weighted-slope steps, huge weights, 6000 dynamic DP updates/queries with restored paths passed\n";
}
