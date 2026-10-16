#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "heuristic_search/all.hpp"
namespace {
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
template<class E, class F> void rejects(F fn) {
    bool caught = false; try { fn(); } catch (const E&) { caught = true; }
    require(caught, "expected exception");
}
struct FakeClock {
    using duration = std::chrono::milliseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<FakeClock>;
    static constexpr bool is_steady = true;
    static inline time_point current{};
    static time_point now() { return current; }
    static void advance(int ms) { current += duration(ms); }
};
void timers() {
    FakeClock::current = {};
    cp::BasicTimer<FakeClock> timer(2), unlimited;
    require(timer.elapsed() == 0 && timer.remaining() == 2 && timer.progress() == 0 && !timer.expired(), "timer initial");
    FakeClock::advance(500);
    require(timer.elapsed() == 0.5 && timer.remaining() == 1.5 && timer.progress() == 0.25, "timer units");
    FakeClock::advance(1500);
    require(timer.expired() && timer.remaining() == 0 && timer.progress() == 1, "exact deadline");
    FakeClock::advance(1000);
    require(timer.expired() && timer.progress() == 1 && timer.remaining() == 0, "clamped timer");
    require(!unlimited.expired() && std::isinf(unlimited.remaining()) && unlimited.progress() == 0, "unlimited timer");
    timer.reset(); require(timer.limit() == 2 && timer.elapsed() == 0, "reset retains limit");
    timer.reset(0); require(timer.expired() && timer.progress() == 1, "zero timer");
    for (double limit : {-1.0, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        rejects<std::invalid_argument>([&] { cp::BasicTimer<FakeClock> bad(limit); });
        rejects<std::invalid_argument>([&] { timer.reset(limit); });
        require(timer.limit() == 0, "invalid reset leaves old timer");
    }
    cp::Timer real;
    require(real.elapsed() >= 0 && !real.expired(), "real steady_clock smoke");
    std::cout << "timer: fake-clock boundaries/reset/infinity/invalid limits and steady-clock smoke passed\n";
}
template<class Int> void integer_ranges(cp::Random& random) {
    for (int trial = 0; trial < 2000; ++trial) {
        Int a = static_cast<Int>(random()), b = static_cast<Int>(random());
        if (a > b) std::swap(a,b);
        const auto v = random.integer(a,b);
        require(a <= v && v <= b, "integer interval bounds");
    }
    for (int i = 0; i < 1000; ++i) {
        const auto v = random.integer(std::numeric_limits<Int>::min(), std::numeric_limits<Int>::max());
        require(v >= std::numeric_limits<Int>::min() && v <= std::numeric_limits<Int>::max(), "full interval");
    }
}
void randoms() {
    cp::Random a(0), b(0);
    require(a() == UINT64_C(16294208416658607535) && a() == UINT64_C(7960286522194355700), "published algorithm seed zero vector");
    a.seed(); require(a() == b(), "reseed");
    for (int i = 0; i < 10000; ++i) require(a() == b(), "copy of seed reproduces raw stream");
    a = b; for (int i = 0; i < 1000; ++i) require(a.real() == b.real(), "real draws deterministic");
    for (auto bound : std::vector<std::uint64_t>{1,2,3,7,UINT64_C(1)<<63,(UINT64_C(1)<<63)+1,UINT64_MAX}) {
        for (int i = 0; i < 5000; ++i) require(a.uniform(bound) < bound, "uniform bounds, heavy rejection");
    }
    for (int i = 0; i < 10000; ++i) { const double x = a.real(); require(0 <= x && x < 1 && std::ldexp(x,53) == std::floor(std::ldexp(x,53)), "real half-open grid"); }
    integer_ranges<signed char>(a); integer_ranges<unsigned char>(a);
    integer_ranges<short>(a); integer_ranges<unsigned short>(a);
    integer_ranges<int>(a); integer_ranges<unsigned int>(a);
    integer_ranges<long long>(a); integer_ranges<unsigned long long>(a);
    for (int n = 0; n < 100; ++n) {
        std::vector<int> order(n); std::iota(order.begin(),order.end(),0); auto copy = order;
        a.seed(n); b.seed(n); a.shuffle(order.begin(),order.end()); b.shuffle(copy.begin(),copy.end());
        require(copy == order, "shuffle reproducible"); std::sort(order.begin(),order.end());
        for (int i = 0; i < n; ++i) require(order[i] == i, "shuffle permutation");
    }
    rejects<std::invalid_argument>([&] { a.uniform(0); });
    rejects<std::invalid_argument>([&] { a.integer(2,1); });
    std::vector<int> order(2); rejects<std::invalid_argument>([&] { a.shuffle(order.end(),order.begin()); });
    std::uniform_int_distribution<int> distribution(-5,5);
    require(distribution(a) >= -5, "standard URBG interface");
    std::cout << "random: known vectors, reproducibility, 8 integral types/full ranges, rejection ranges, real grid, shuffle/URBG passed\n";
}
bool better(int a, int b, cp::SearchGoal goal) { return goal == cp::SearchGoal::minimize ? a < b : a > b; }
void local_exhaustive() {
    std::size_t cases = 0;
    for (int n = 1; n <= 6; ++n) {
        int count = 1; for (int i = 0; i < n; ++i) count *= 3;
        for (int code = 0; code < count; ++code) {
            int x = code; std::vector<int> score(n); for (auto& s : score) { s = x % 3; x /= 3; }
            for (auto goal : {cp::SearchGoal::minimize,cp::SearchGoal::maximize})
            for (auto mode : {cp::LocalSearchMode::first_improvement,cp::LocalSearchMode::best_improvement})
            for (int start = 0; start < n; ++start) {
                auto neighbors = [n](int v) { std::vector<int> out; if (v) out.push_back(v-1); if (v+1<n) out.push_back(v+1); return out; };
                int current = start; std::size_t scans = 0, evals = 1, accepted = 0;
                for (;;) {
                    int next = current;
                    for (int v : neighbors(current)) { ++evals; if (better(score[v],score[next],goal)) { next=v; if(mode==cp::LocalSearchMode::first_improvement) break; } }
                    ++scans; if (next == current) break; current = next; ++accepted;
                }
                cp::LocalSearchOptions options; options.goal=goal; options.mode=mode;
                auto result = cp::local_search(start,[&](int v){return score[v];},neighbors,options);
                require(result.best_state==current && result.current_state==current && result.best_score==score[current] && result.current_score==score[current], "local vs independent line graph walk");
                require(result.iterations==scans && result.evaluations==evals && result.accepted==accepted && result.termination==cp::SearchTermination::local_optimum, "local counts/termination");
                ++cases;
            }
        }
    }
    std::cout << cases << " exhaustive local searches: both goals/modes, plateaus and scan/evaluation counts passed\n";
}
void beams(std::mt19937_64& rng) {
    std::size_t cases=0;
    for (int trial=0; trial<250; ++trial) {
        const int n=1+rng()%80; std::vector<std::vector<int>> graph(n); std::vector<int> score(n);
        for(int v=0;v<n;++v) { score[v]=int(rng()%9)-4; for(int j=0;j<int(rng()%5);++j)graph[v].push_back(rng()%n); }
        for(auto goal:{cp::SearchGoal::minimize,cp::SearchGoal::maximize}) for(std::size_t width:{1,2,5,40}) {
            cp::BeamSearchOptions options; options.width=width; options.max_depth=6; options.goal=goal;
            auto got=cp::beam_search(0,[&](int v){return score[v];},[&](int v){return graph[v];},options);
            std::vector<int> frontier{0}; std::size_t depth=0, evaluations=1; bool exhausted=false;
            for(;depth<6;) {
                std::vector<int> candidates; for(int v:frontier)for(int u:graph[v]){candidates.push_back(u);++evaluations;}
                if(candidates.empty()){exhausted=true;break;}
                std::vector<int> selected;
                while(!candidates.empty()&&selected.size()<width) {
                    auto best=candidates.begin(); for(auto it=candidates.begin();it!=candidates.end();++it)if(better(score[*it],score[*best],goal))best=it;
                    selected.push_back(*best);candidates.erase(best);
                }
                frontier=std::move(selected);++depth;
            }
            require(got.depth==depth && got.evaluations==evaluations && got.frontier.size()==frontier.size(),"beam counts/frontier");
            for(std::size_t i=0;i<frontier.size();++i)require(got.frontier[i].state==frontier[i] && got.frontier[i].score==score[frontier[i]],"beam stable top width vs repeated minimum selection");
            require(got.termination==(exhausted?cp::SearchTermination::exhausted:cp::SearchTermination::depth_limit),"beam termination");
            require(got.best().state==frontier.front(),"beam best final depth");++cases;
        }
    }
    // Complete binary tree: a wide beam equals exhaustive leaves; a narrow beam can miss the optimum.
    std::vector<int> score{0,1,2,100,100,-10,0}; auto expand=[](int x){return x<3?std::vector<int>{2*x+1,2*x+2}:std::vector<int>{};};
    cp::BeamSearchOptions options;options.max_depth=2;options.width=1;
    require(cp::beam_search(0,[&](int x){return score[x];},expand,options).best().score==100,"narrow beam can miss optimum");
    options.width=4;require(cp::beam_search(0,[&](int x){return score[x];},expand,options).best().score==-10,"wide beam matches leaves");
    std::cout << cases << " random cyclic/duplicate/terminal beam cases: repeated-selection oracle, stable ties, wide/narrow counterexample passed\n";
}
void annealing() {
    std::size_t cases=0;
    for (auto goal:{cp::SearchGoal::minimize,cp::SearchGoal::maximize}) for(auto cooling:{cp::CoolingSchedule::linear,cp::CoolingSchedule::geometric})
    for (int seed=0;seed<200;++seed) {
        cp::AnnealingOptions options;options.goal=goal;options.cooling=cooling;options.max_iterations=75;options.start_temperature=8;options.end_temperature=0.25L;
        auto score=[](int x){return x*x+3*x;}; auto propose=[](int x,cp::Random&r){return std::clamp(x+r.integer(-3,3),-20,20);};
        cp::Random random(seed), oracle(seed); const auto got=cp::simulated_annealing(0,score,propose,random,options);
        int current=0,best=0;std::size_t accepted=0;
        for(int step=0;step<75;++step){const int candidate=propose(current,oracle);const long double p=step/74.0L;
            const long double temp=cooling==cp::CoolingSchedule::linear?8*(1-p)+0.25L*p:8*std::pow(0.25L/8,p);
            const int gain=goal==cp::SearchGoal::maximize?score(candidate)-score(current):score(current)-score(candidate);
            if(gain>=0||oracle.real()<std::exp(static_cast<long double>(gain)/temp)){current=candidate;++accepted;if(better(score(current),score(best),goal))best=current;}}
        require(got.current_state==current && got.best_state==best && got.best_score==score(best) && got.current_score==score(current),"annealing vs independent fixed iteration simulation");
        require(got.iterations==75 && got.evaluations==76 && got.accepted==accepted && got.termination==cp::SearchTermination::iteration_limit && random()==oracle(),"SA counters and exact RNG consumption");++cases;
    }
    cp::AnnealingOptions options;options.max_iterations=1;options.start_temperature=options.end_temperature=1e-20L;
    cp::Random random(0);
    const auto hi=std::numeric_limits<std::uint64_t>::max();
    auto wide=cp::simulated_annealing(hi-1,[](auto x){return x;},[&](auto,cp::Random&){return hi;},random,options);
    require(wide.current_state==hi-1 && wide.accepted==0,"adjacent uint64 must not be treated equal");
    auto signed_wide=cp::simulated_annealing(std::numeric_limits<long long>::min(),[](auto x){return x;},[](auto,cp::Random&){return std::numeric_limits<long long>::max();},random,options);
    require(signed_wide.accepted==0,"signed min/max subtraction without overflow");
    options.start_temperature=options.end_temperature=1e30L;
    auto retained=cp::simulated_annealing(0,[](int x){return x;},[](int,cp::Random&){return 1;},random,options);
    require(retained.current_state==1 && retained.best_state==0 && retained.accepted==1,"accepted worsening retains best copy");
    for(auto cooling:{cp::CoolingSchedule::linear,cp::CoolingSchedule::geometric}){
        options.cooling=cooling;options.start_temperature=std::numeric_limits<long double>::max();options.end_temperature=std::numeric_limits<long double>::min();
        for(int i=0;i<=100;++i){auto t=cp::annealing_temperature(options,i/100.0L);require(std::isfinite(t)&&t>0,"extreme positive schedule finite");}
        require(cp::annealing_temperature(options,0)==options.start_temperature && cp::annealing_temperature(options,1)==options.end_temperature,"exact temperature endpoints");
    }
    std::cout<<cases<<" annealing simulations: both goals/schedules, exact RNG use, 64-bit differences, current/best and temperature extremes passed\n";
}
void budgets_and_invalid() {
    cp::Random random(42); auto evaluate=[](int x){return x;};auto propose=[](int x,cp::Random&){return x-1;};auto neighbors=[](int x){return std::vector<int>{x-1,x-2};};
    cp::AnnealingOptions sa;cp::LocalSearchOptions local;cp::BeamSearchOptions beam;
    sa.max_iterations=local.max_iterations=beam.max_depth=0;
    require(cp::simulated_annealing(5,evaluate,propose,random,sa).evaluations==1 && cp::local_search(5,evaluate,neighbors,local).iterations==0 && cp::beam_search(5,evaluate,neighbors,beam).depth==0,"zero budgets evaluate only initial");
    sa.max_iterations=local.max_iterations=beam.max_depth=100;
    sa.stop=local.stop=beam.stop=[] {return true;};
    require(cp::simulated_annealing(5,evaluate,propose,random,sa).termination==cp::SearchTermination::stopped && cp::local_search(5,evaluate,neighbors,local).termination==cp::SearchTermination::stopped && cp::beam_search(5,evaluate,neighbors,beam).termination==cp::SearchTermination::stopped,"immediate cooperative stop");
    int checks=0;local.stop=[&]{return ++checks==3;};
    auto l=cp::local_search(5,evaluate,neighbors,local);require(l.current_state==5 && l.evaluations==2 && l.iterations==0,"discard partial best improvement scan");
    checks=0;beam.stop=[&]{return ++checks==3;};auto b=cp::beam_search(5,evaluate,neighbors,beam);
    require(b.best().state==5 && b.evaluations==2 && b.depth==0,"discard partial beam layer");
    FakeClock::current={};cp::BasicTimer<FakeClock> timer(0.004);
    sa.stop=[&]{return timer.expired();};sa.progress=[&]{return timer.progress();};
    auto timed=cp::simulated_annealing(5,evaluate,[](int x,cp::Random&){FakeClock::advance(1);return x-1;},random,sa);
    require(timed.iterations==4 && timed.best_state==1 && timed.termination==cp::SearchTermination::stopped,"timer integration and time schedule");
    for(double bad:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}) {
        auto invalid=[bad](int){return bad;};
        rejects<std::invalid_argument>([&]{cp::simulated_annealing(0,invalid,propose,random);});
        rejects<std::invalid_argument>([&]{cp::local_search(0,invalid,neighbors);});
        rejects<std::invalid_argument>([&]{cp::beam_search(0,invalid,neighbors);});
        auto later=[bad](int x){return x==0?0:bad;};
        rejects<std::invalid_argument>([&]{cp::simulated_annealing(0,later,propose,random);});
        rejects<std::invalid_argument>([&]{cp::local_search(0,later,neighbors);});
        rejects<std::invalid_argument>([&]{cp::beam_search(0,later,neighbors);});
    }
    for(long double temperature:{0.0L,-1.0L,std::numeric_limits<long double>::infinity(),std::numeric_limits<long double>::quiet_NaN()}){
        cp::AnnealingOptions bad;bad.start_temperature=temperature;
        rejects<std::invalid_argument>([&]{cp::simulated_annealing(0,evaluate,propose,random,bad);});
        bad.start_temperature=1;bad.end_temperature=temperature;
        rejects<std::invalid_argument>([&]{cp::annealing_temperature(bad,0.5L);});
    }
    for(long double p:{-0.01L,1.01L,std::numeric_limits<long double>::quiet_NaN()}){
        cp::AnnealingOptions bad;bad.progress=[p]{return p;};
        rejects<std::invalid_argument>([&]{cp::simulated_annealing(0,evaluate,propose,random,bad);});
    }
    beam.width=0;rejects<std::invalid_argument>([&]{cp::beam_search(0,evaluate,neighbors,beam);});
    local.goal=static_cast<cp::SearchGoal>(99);rejects<std::invalid_argument>([&]{cp::local_search(0,evaluate,neighbors,local);});
    local.goal=cp::SearchGoal::minimize;local.mode=static_cast<cp::LocalSearchMode>(99);rejects<std::invalid_argument>([&]{cp::local_search(0,evaluate,neighbors,local);});
    sa.cooling=static_cast<cp::CoolingSchedule>(99);rejects<std::invalid_argument>([&]{cp::simulated_annealing(0,evaluate,propose,random,sa);});
    std::size_t count=std::numeric_limits<std::size_t>::max();rejects<std::overflow_error>([&]{cp::heuristic_detail::count_evaluation(count);});
    std::cout<<"zero/iteration/time budgets, partial scans/layers, nonfinite initial/candidate scores and invalid options passed\n";
}
struct NoDefault { int value; explicit NoDefault(int x):value(x){} };
void nondefault_and_large() {
    auto evaluate=[](const NoDefault& s){return s.value;};
    auto neighbors=[](const NoDefault&s){return std::vector<NoDefault>{NoDefault(s.value-1)};};
    cp::LocalSearchOptions local;local.max_iterations=3;
    require(cp::local_search(NoDefault(5),evaluate,neighbors,local).best_state.value==2,"nondefault state local");
    cp::BeamSearchOptions beam;beam.max_depth=3;
    require(cp::beam_search(NoDefault(5),evaluate,neighbors,beam).best().state.value==2,"nondefault state beam");
    cp::Random random;cp::AnnealingOptions sa;sa.max_iterations=3;
    require(cp::simulated_annealing(NoDefault(5),evaluate,[](const NoDefault&s,cp::Random&){return NoDefault(s.value-1);},random,sa).best_state.value==2,"nondefault state SA");
    local.max_iterations=200000;
    auto r=cp::local_search(200000,[](int x){return x;},[](int x){return std::vector<int>{x-1};},local);
    require(r.best_state==0&&r.accepted==200000&&r.termination==cp::SearchTermination::iteration_limit,"200000 strict local improvements without recursion");
    beam.width=200;beam.max_depth=200;
    auto b=cp::beam_search(0,[](int x){return x;},[](int x){return std::vector<int>{x+1,x+1,x+2};},beam);
    require(b.depth==200&&b.frontier.size()==200&&b.best().score==200,"bounded width, duplicate retention across 200 depths");
    std::cout<<"non-default-constructible states, 200000 local steps and width-200/depth-200 beam passed\n";
}
}
int main(){std::mt19937_64 rng(325025);randoms();timers();local_exhaustive();beams(rng);annealing();budgets_and_invalid();nondefault_and_large();}
