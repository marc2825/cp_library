#include <algorithm>
#include <array>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <vector>
#include "dp_optimization/all.hpp"

#define CHECK(...) do { if (!(__VA_ARGS__)) { std::cerr << "check failed at " << __LINE__ << ": " << #__VA_ARGS__ << '\n'; std::abort(); } } while (false)
std::mt19937 rng(312016);
int arrays = 0, pairs = 0, bags = 0, grids = 0, tours = 0;
template <class Exception, class F> void throws(F f) { bool caught = false; try { f(); } catch (const Exception&) { caught = true; } CHECK(caught); }
struct Mod {
    static constexpr int mod = 1000000007;
    long long value = 0;
    Mod(long long v = 0) : value((v % mod + mod) % mod) {}
    Mod& operator+=(const Mod& x) { value = (value + x.value) % mod; return *this; }
    Mod& operator-=(const Mod& x) { value = (value - x.value + mod) % mod; return *this; }
    bool operator==(const Mod& x) const { return value == x.value; }
};

void sequence_tests() {
    for (int n = 0, cases = 1; n <= 8; ++n, cases *= 3) for (int code = 0; code < cases; ++code) {
        std::vector<long long> a(n); int state = code;
        for (auto& value : a) { value = state % 3 - 1; state /= 3; }
        const auto original = a;
        for (bool strict : {false, true}) for (bool decreasing : {false, true}) {
            const auto cmp = [&](long long x, long long y) { return decreasing ? x > y : x < y; };
            auto result = cp::lis(a, strict, cmp);
            std::vector<int> ending(n, 1);
            for (int i = 0; i < n; ++i) for (int j = 0; j < i; ++j)
                if (strict ? cmp(a[j], a[i]) : !cmp(a[i], a[j])) ending[i] = std::max(ending[i], ending[j] + 1);
            CHECK(result.length_ending == ending && a == original);
            CHECK(result.length == (n ? *std::max_element(ending.begin(), ending.end()) : 0));
            CHECK(static_cast<int>(result.indices.size()) == result.length);
            for (int i = 0; i < result.length; ++i) {
                CHECK(0 <= result.indices[i] && result.indices[i] < n);
                if (i) { const int u = result.indices[i - 1], v = result.indices[i]; CHECK(u < v && (strict ? cmp(a[u], a[v]) : !cmp(a[v], a[u]))); }
            }
        }
        std::vector<long long> heights = a; for (auto& h : heights) ++h;
        const auto rectangle = cp::largest_rectangle_histogram(heights);
        long long best = 0;
        for (int l = 0; l < n; ++l) { long long minimum = LLONG_MAX; for (int r = l; r < n; ++r) { minimum = std::min(minimum, heights[r]); best = std::max(best, minimum * (r - l + 1)); } }
        CHECK(rectangle.area == best);
        CHECK(rectangle.area == rectangle.height * (rectangle.right - rectangle.left));
        for (int i = rectangle.left; i < rectangle.right; ++i) CHECK(heights[i] >= rectangle.height);
        ++arrays;
    }
    CHECK(cp::lis(std::vector<long long>{LLONG_MAX, 0, LLONG_MIN}, true, std::greater<long long>{}).length == 3);
    CHECK(cp::lis(std::vector<std::string>{"b", "a", "c"}).length == 2);
    CHECK(cp::lis(std::vector<int>{3, 1, 2}).length == 2);
}
std::vector<std::string> words(int max_length) {
    std::vector<std::string> result;
    for (int length = 0; length <= max_length; ++length) for (int mask = 0; mask < (1 << length); ++mask) {
        std::string s; for (int i = 0; i < length; ++i) s += mask >> i & 1 ? 'b' : 'a'; result.push_back(s);
    }
    return result;
}
int lcs_oracle(const std::string& a, const std::string& b) {
    int best = 0;
    for (int mask = 0; mask < (1 << a.size()); ++mask) {
        std::size_t next = 0; int length = 0; bool valid = true;
        for (std::size_t i = 0; i < a.size(); ++i) if (mask >> i & 1) {
            next = b.find(a[i], next); if (next == std::string::npos) { valid = false; break; } ++next; ++length;
        }
        if (valid) best = std::max(best, length);
    }
    return best;
}
int edit_oracle(const std::string& a, const std::string& b) {
    // Shortest path among actual strings, not the prefix DP recurrence.
    std::queue<std::string> queue; std::map<std::string, int> distance; queue.push(a); distance[a] = 0;
    const auto max_length = std::max(a.size(), b.size());
    while (!queue.empty()) {
        const auto s = queue.front(); queue.pop(); const int d = distance[s]; if (s == b) return d;
        auto visit = [&](const std::string& t) { if (distance.emplace(t, d + 1).second) queue.push(t); };
        for (std::size_t i = 0; i < s.size(); ++i) { auto t = s; t.erase(i, 1); visit(t); for (char c : {'a', 'b'}) { t = s; t[i] = c; visit(t); } }
        if (s.size() < max_length) for (std::size_t i = 0; i <= s.size(); ++i) for (char c : {'a', 'b'}) { auto t = s; t.insert(i, 1, c); visit(t); }
    }
    std::abort();
}
void alignment_tests() {
    for (const auto& a : words(5)) for (const auto& b : words(5)) {
        const int best = lcs_oracle(a, b); auto result = cp::lcs(a, b);
        CHECK(result.length == best && cp::lcs_length(a, b) == best && static_cast<int>(result.matches.size()) == best);
        int previous_a = -1, previous_b = -1;
        for (auto [i, j] : result.matches) { CHECK(previous_a < i && previous_b < j && a[i] == b[j]); previous_a = i; previous_b = j; }
        ++pairs;
    }
    for (const auto& a : words(3)) for (const auto& b : words(3)) {
        const int best = edit_oracle(a, b); const auto result = cp::edit_alignment(a, b);
        CHECK(result.distance == best && cp::edit_distance(a, b) == best);
        int i = 0, j = 0, cost = 0; std::string produced;
        for (auto step : result.steps) {
            CHECK(step.source_index == i && step.target_index == j);
            if (step.kind == cp::EditKind::Keep) { CHECK(a[i] == b[j]); produced += a[i]; ++i; ++j; }
            else if (step.kind == cp::EditKind::Erase) { ++i; ++cost; }
            else { produced += b[j++]; if (step.kind == cp::EditKind::Replace) ++i; ++cost; }
        }
        CHECK(i == static_cast<int>(a.size()) && j == static_cast<int>(b.size()) && produced == b && cost == best);
    }
    CHECK(cp::edit_distance(std::string("kitten"), std::string("sitting")) == 3);
    CHECK(cp::edit_distance(std::vector<int>{1,2,3}, std::vector<int>{2,3}) == 1);
    CHECK(cp::lcs_length(std::vector<int>{1,2,3}, std::vector<int>{2,4}) == 1);
    auto equal = [](char a, char b) { return (a | 32) == (b | 32); };
    CHECK(cp::lcs(std::string("AbC"), std::string("aBc"), equal).length == 3);
    CHECK(cp::edit_alignment(std::string("AbC"), std::string("aBc"), equal).distance == 0);
}
using Table = std::vector<std::optional<long long>>;
Table bag_oracle(int capacity, const std::vector<cp::BoundedKnapsackItem>& items) {
    Table best(capacity + 1);
    std::function<void(int,int,long long)> enumerate = [&](int index, int weight, long long value) {
        if (index == static_cast<int>(items.size())) { if (!best[weight] || *best[weight] < value) best[weight] = value; return; }
        const auto item = items[index];
        for (int count = 0; count <= item.count && weight + count * item.weight <= capacity; ++count)
            enumerate(index + 1, weight + count * item.weight, value + count * item.value);
    };
    enumerate(0, 0, 0); return best;
}
void knapsack_tests() {
    std::vector<cp::BoundedKnapsackItem> catalog;
    for (int w : {0,1,2}) for (int v : {-2,3}) for (int count : {0,1,2}) catalog.push_back({w,v,count});
    for (int n = 0, cases = 1; n <= 3; ++n, cases *= 18) for (int code = 0; code < cases; ++code) {
        int state = code; std::vector<cp::BoundedKnapsackItem> items;
        for (int i = 0; i < n; ++i) { items.push_back(catalog[state % 18]); state /= 18; }
        for (int capacity = 0; capacity <= 6; ++capacity) CHECK(cp::bounded_knapsack(capacity, items).dp == bag_oracle(capacity, items));
        ++bags;
    }
    for (int trial = 0; trial < 400; ++trial) {
        const int capacity = rng() % 17, n = rng() % 6;
        std::vector<cp::KnapsackItem> items; std::vector<cp::BoundedKnapsackItem> bounded, once, unlimited;
        for (int i = 0; i < n; ++i) {
            const int w = rng() % 8; const long long v = static_cast<int>(rng() % 19) - 9;
            items.push_back({w,v}); bounded.push_back({w,v,static_cast<long long>(rng()%4)}); once.push_back({w,v,1});
        }
        auto zero_one = cp::zero_one_knapsack(capacity, items), multiple = cp::bounded_knapsack(capacity, bounded);
        CHECK(zero_one.dp == bag_oracle(capacity, once) && multiple.dp == bag_oracle(capacity, bounded));
        for (auto& item : items) { ++item.weight; unlimited.push_back({item.weight,item.value,capacity/item.weight}); }
        CHECK(cp::unbounded_knapsack(capacity, items).dp == bag_oracle(capacity, unlimited));
        long long best = 0; for (const auto& value : zero_one.dp) if (value) best = std::max(best, *value);
        CHECK(zero_one.best_value() == best); ++bags;
    }
    CHECK(cp::zero_one_knapsack(1, {{1,LLONG_MIN}}).dp[1] == LLONG_MIN);
    CHECK(cp::zero_one_knapsack(0, {{0,LLONG_MAX}}).best_value() == LLONG_MAX);
    CHECK(cp::bounded_knapsack(1, {{1,LLONG_MIN,1}}).dp[1] == LLONG_MIN);
    CHECK(cp::bounded_knapsack(0, {{0,1,LLONG_MAX}}).best_value() == LLONG_MAX);
    CHECK(cp::bounded_knapsack(10, {{3,7,LLONG_MAX}}).best_value() == 21);
    CHECK(cp::bounded_knapsack(1, {{0,LLONG_MIN,LLONG_MAX}}).best_value() == 0);
    throws<std::overflow_error>([] { cp::zero_one_knapsack(0, {{0,LLONG_MAX},{0,1}}); });
    throws<std::overflow_error>([] { cp::unbounded_knapsack(2, {{1,LLONG_MAX}}); });
    throws<std::overflow_error>([] { cp::bounded_knapsack(0, {{0,LLONG_MAX,2}}); });
}
void bitset_tests() {
    for (int capacity : {0,1,62,63,64,65,127,128,129,255,1000}) {
        cp::BitsetKnapsack bag(capacity); std::vector<bool> reachable(capacity + 1); reachable[0] = true;
        for (int trial = 0; trial < 250; ++trial) {
            const int weight = trial < 9 ? std::array<int,9>{0,1,63,64,65,127,128,129,1001}[trial] : rng() % 300;
            bag.add(weight);
            for (int s = capacity; s >= weight; --s) reachable[s] = reachable[s] || reachable[s - weight];
            std::vector<int> sums;
            for (int s = 0; s <= capacity; ++s) { CHECK(bag.contains(s) == reachable[s]); if (reachable[s]) sums.push_back(s); }
            CHECK(bag.reachable_sums() == sums && !bag.contains(-1) && !bag.contains(capacity + 1));
            for (std::size_t word = 0; word < bag.words().size(); ++word) {
                std::uint64_t expected = 0;
                for (int bit = 0; bit < 64 && word * 64 + bit <= static_cast<std::size_t>(capacity); ++bit) if (reachable[word * 64 + bit]) expected |= std::uint64_t(1) << bit;
                CHECK(bag.words()[word] == expected);
            }
        }
        bag.reset(); CHECK(bag.reachable_sums() == std::vector<int>{0});
    }
}
template <class Cost> long long interval_oracle(int n, Cost cost) {
    std::vector<std::pair<int,int>> groups; for (int i = 0; i < n; ++i) groups.emplace_back(i,i+1);
    std::optional<long long> answer;
    std::function<void(const std::vector<std::pair<int,int>>&,long long)> merge = [&](const auto& current, long long total) {
        if (current.size() <= 1) { if (!answer || total < *answer) answer = total; return; }
        for (std::size_t i = 1; i < current.size(); ++i) {
            auto next = current; const auto a = current[i-1], b = current[i]; next[i-1] = {a.first,b.second}; next.erase(next.begin()+i);
            merge(next, total + cost(a.first,a.second,b.second));
        }
    };
    merge(groups,0); return *answer;
}
void interval_tests() {
    for (int trial = 0; trial < 200; ++trial) {
        const int n = rng() % 8; std::vector<long long> prefix(n+1);
        for (int i=0;i<n;++i) prefix[i+1] = prefix[i] + static_cast<int>(rng()%12)-3;
        const bool split_cost = trial % 2;
        auto cost = [&](int l,int k,int r) { return prefix[r]-prefix[l]+(split_cost ? k*k-l*r : 0); };
        const auto dp = cp::interval_dp(n,cost); CHECK(dp.value() == interval_oracle(n,cost));
        std::set<std::pair<int,int>> active; for(int i=0;i<n;++i)active.emplace(i,i+1);
        long long total=0;
        for(auto step:dp.merge_order()) { const auto [l,k,r]=step; CHECK(active.erase({l,k})==1 && active.erase({k,r})==1); active.emplace(l,r);total+=cost(l,k,r); }
        CHECK(total==dp.value() && dp.merge_order().size()==static_cast<std::size_t>(n?n-1:0));
    }
    int calls=0;CHECK(cp::interval_dp(1,[&](int,int,int){++calls;return 0LL;}).value()==0 && calls==0);
    CHECK(cp::interval_dp(2,[](int,int,int){return LLONG_MAX;}).value()==LLONG_MAX);
    throws<std::overflow_error>([] { cp::interval_dp(3,[](int,int,int){return LLONG_MAX;}); });
}
void digit_tests() {
    for (int mod=1;mod<=9;++mod) for(int bound=0;bound<=1000;++bound) {
        long long expected=0;
        for(int x=1;x<=bound;++x){int s=0;for(int y=x;y;y/=10)s+=y%10;if(s%mod==0)++expected;}
        auto transition=[=](int state,int digit,bool,bool){return (state+digit)%mod;};
        CHECK(cp::digit_dp(std::to_string(bound),mod,0,transition,[](int state,bool started){return !state&&started;})==expected);
        CHECK(cp::digit_dp("00"+std::to_string(bound),mod,0,transition,[](int state,bool){return !state;})==expected+1);
    }
    // No digit 4, and adjacent significant digits must differ. Leading zeros are ignored.
    for(int bound=0;bound<500;++bound) {
        long long expected=0;
        for(int x=1;x<=bound;++x){auto s=std::to_string(x);bool valid=s.find('4')==std::string::npos;for(std::size_t i=1;i<s.size();++i)valid &= s[i]!=s[i-1];expected+=valid;}
        auto transition=[](int state,int digit,bool started,bool next_started){if(!next_started)return 10;if(digit==4||(started&&digit==state))return -1;return digit;};
        CHECK(cp::digit_dp(std::to_string(bound),11,10,transition,[](int,bool started){return started;})==expected);
    }
    auto next=[](int,int,bool,bool){return 0;};auto accept=[](int,bool){return true;};
    CHECK(cp::digit_dp(std::to_string(LLONG_MAX-1),1,0,next,accept)==LLONG_MAX);
    CHECK(cp::digit_dp<unsigned long long>(std::to_string(ULLONG_MAX-1),1,0,next,accept)==ULLONG_MAX);
    throws<std::overflow_error>([&]{cp::digit_dp(std::to_string(LLONG_MAX),1,0,next,accept);});
    throws<std::overflow_error>([&]{cp::digit_dp<unsigned long long>(std::to_string(ULLONG_MAX),1,0,next,accept);});
    throws<std::invalid_argument>([&]{cp::digit_dp("",1,0,next,accept);});
    throws<std::invalid_argument>([&]{cp::digit_dp("-1",1,0,next,accept);});
    throws<std::invalid_argument>([&]{cp::digit_dp("1x",1,0,next,accept);});
}
void subset_tests() {
    for(int n=0;n<=10;++n){const int states=1<<n;std::vector<long long> f(states);for(auto& x:f)x=static_cast<int>(rng()%21)-10;auto zeta=f;cp::subset_zeta_transform(zeta);
        for(int mask=0;mask<states;++mask){long long expected=0;std::set<std::uint64_t> submasks;cp::for_each_submask(mask,[&](std::uint64_t sub){CHECK(submasks.insert(sub).second);});for(int sub=0;sub<=mask;++sub)if((sub&mask)==sub){expected+=f[sub];CHECK(submasks.erase(sub)==1);}CHECK(submasks.empty()&&zeta[mask]==expected);}
        cp::subset_mobius_transform(zeta);CHECK(zeta==f);
    }
    std::vector<Mod> modular(16,Mod(1000000006));const auto original=modular;cp::subset_zeta_transform(modular);cp::subset_mobius_transform(modular);CHECK(modular==original);
    int callbacks=0;cp::for_each_submask(0,[&](std::uint64_t){++callbacks;},false);CHECK(callbacks==0);
    std::vector<std::uint64_t> high;cp::for_each_submask((std::uint64_t(1)<<63)|1,[&](std::uint64_t sub){high.push_back(sub);});CHECK(high==std::vector<std::uint64_t>({(std::uint64_t(1)<<63)|1,std::uint64_t(1)<<63,1,0}));
    throws<std::invalid_argument>([]{std::vector<int> a;cp::subset_zeta_transform(a);});
    throws<std::invalid_argument>([]{std::vector<int> a(3);cp::subset_mobius_transform(a);});
    for(int trial=0;trial<300;++trial){const int n=2+rng()%7,start=rng()%n;std::vector<std::vector<std::optional<long long>>> cost(n,std::vector<std::optional<long long>>(n));
        for(int u=0;u<n;++u)for(int v=0;v<n;++v)if(u!=v&&rng()%4)cost[u][v]=static_cast<int>(rng()%21)-10;
        std::vector<int> order;for(int v=0;v<n;++v)if(v!=start)order.push_back(v);std::optional<long long> expected;
        do{bool valid=true;long long value=0;int from=start;for(int v:order){if(!cost[from][v]){valid=false;break;}value+=*cost[from][v];from=v;}if(valid&&cost[from][start]){value+=*cost[from][start];if(!expected||value<*expected)expected=value;}}while(std::next_permutation(order.begin(),order.end()));
        auto result=cp::minimum_hamiltonian_cycle(cost,start);CHECK(result.exists==expected.has_value());
        if(expected){CHECK(result.cost==*expected&&result.vertices.size()==static_cast<std::size_t>(n+1)&&result.vertices.front()==start&&result.vertices.back()==start);std::set<int> visited(result.vertices.begin(),result.vertices.end()-1);CHECK(static_cast<int>(visited.size())==n);long long sum=0;for(int i=1;i<=n;++i){auto w=cost[result.vertices[i-1]][result.vertices[i]];CHECK(w.has_value());sum+=*w;}CHECK(sum==result.cost);}++tours;
    }
    CHECK(cp::minimum_hamiltonian_cycle({}).exists&&cp::minimum_hamiltonian_cycle({}).vertices.empty());
    CHECK(cp::minimum_hamiltonian_cycle({{std::nullopt}}).vertices==std::vector<int>{0});
    throws<std::length_error>([]{cp::minimum_hamiltonian_cycle(std::vector<std::vector<std::optional<long long>>>(21));});
    throws<std::invalid_argument>([]{cp::minimum_hamiltonian_cycle({{0,1},{2}});});
    throws<std::overflow_error>([]{cp::minimum_hamiltonian_cycle({{std::nullopt,LLONG_MAX},{1,std::nullopt}});});
    CHECK(cp::minimum_hamiltonian_cycle({{std::nullopt,LLONG_MAX},{0,std::nullopt}}).cost==LLONG_MAX);
}
void rectangle_tests() {
    for(int h=0;h<=3;++h)for(int w=0;w<=4;++w)for(int mask=0;mask<(1<<(h*w));++mask){std::vector<std::vector<bool>> blocked(h,std::vector<bool>(w));for(int i=0;i<h;++i)for(int j=0;j<w;++j)blocked[i][j]=mask>>(i*w+j)&1;
        long long best=0;for(int top=0;top<h;++top)for(int bottom=top+1;bottom<=h;++bottom)for(int left=0;left<w;++left)for(int right=left+1;right<=w;++right){bool valid=true;for(int i=top;i<bottom;++i)for(int j=left;j<right;++j)valid &= !blocked[i][j];if(valid)best=std::max(best,1LL*(bottom-top)*(right-left));}
        auto result=cp::largest_rectangle_grid(blocked);CHECK(result.area==best&&result.area==1LL*(result.bottom-result.top)*(result.right-result.left));for(int i=result.top;i<result.bottom;++i)for(int j=result.left;j<result.right;++j)CHECK(!blocked[i][j]);++grids;
    }
    CHECK(cp::largest_rectangle_histogram({LLONG_MAX}).area==LLONG_MAX);
    throws<std::overflow_error>([]{cp::largest_rectangle_histogram({LLONG_MAX,LLONG_MAX});});
    throws<std::invalid_argument>([]{cp::largest_rectangle_grid({{false},{false,false}});});
}
void arithmetic_tests() {
    for(auto a:{LLONG_MIN,LLONG_MIN+1,-100LL,-1LL,0LL,1LL,100LL,LLONG_MAX-1,LLONG_MAX})for(auto b:{-2LL,-1LL,0LL,1LL,2LL}){
        // Division-based oracle with special handling of -LLONG_MIN.
        const bool valid_add=b>0?a<=LLONG_MAX-b:b<0?a>=LLONG_MIN-b:true;
        if(valid_add)CHECK(cp::detail::dp_add(a,b)==a+b);else throws<std::overflow_error>([&]{cp::detail::dp_add(a,b);});
        bool valid_mul=true;if(b==2)valid_mul=LLONG_MIN/2<=a&&a<=LLONG_MAX/2;if(b==-2)valid_mul=-LLONG_MAX/2<=a&&a<=LLONG_MAX/2+1;if(b==-1)valid_mul=a!=LLONG_MIN;
        if(valid_mul)CHECK(cp::detail::dp_multiply(a,b)==a*b);else throws<std::overflow_error>([&]{cp::detail::dp_multiply(a,b);});
    }
}
void large_tests() {
    const int n=200000;std::vector<long long> a(n);std::iota(a.begin(),a.end(),0);CHECK(cp::lis(a).length==n);for(auto& x:a)++x;
    CHECK(cp::largest_rectangle_histogram(a).area==1LL*(n+1)*(n+1)/4);
    cp::BitsetKnapsack bits(n);for(int i=0;i<4000;++i)bits.add(64);for(int s=0;s<=n;++s)CHECK(bits.contains(s)==(s%64==0));bits.add(1);for(int s=0;s<=n;++s)CHECK(bits.contains(s)==(s%64<=1));
    CHECK(cp::bounded_knapsack(n,{{1,2,LLONG_MAX}}).dp.back()==2LL*n);
    CHECK(cp::zero_one_knapsack(n,{{n,7},{0,2}}).best_value()==9);
    CHECK(cp::unbounded_knapsack(n,{{1,2}}).best_value()==2LL*n);
    std::string upper(100000,'9');auto answer=cp::digit_dp<Mod>(upper,1,0,[](int,int,bool,bool){return 0;},[](int,bool){return true;});long long expected=1;for(int i=0;i<100000;++i)expected=expected*10%Mod::mod;CHECK(answer.value==expected);
    std::string s(2000,'a');CHECK(cp::lcs_length(s,s)==2000&&cp::edit_distance(s,s)==0);
    const int size=18;std::vector<std::vector<std::optional<long long>>> ring(size,std::vector<std::optional<long long>>(size));for(int v=0;v<size;++v)ring[v][(v+1)%size]=1;CHECK(cp::minimum_hamiltonian_cycle(ring).cost==size);
    CHECK(cp::interval_dp(160,[](int l,int,int r){return r-l;}).value()>0);
}
int main(){sequence_tests();alignment_tests();knapsack_tests();bitset_tests();interval_tests();digit_tests();subset_tests();rectangle_tests();arithmetic_tests();large_tests();std::cout<<arrays<<" exhaustive arrays; "<<pairs<<" LCS pairs; 225 edit graphs; "<<bags<<" bags; "<<grids<<" binary grids; "<<tours<<" TSP permutations; digit bounds/zero/overflow; word boundaries; 200000 elements/capacity; 100000 digits: passed\n";}
