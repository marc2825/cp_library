#include "heuristic_search/all.hpp"
#include "parsing/all.hpp"
#include <condition_variable>
#include <iostream>
#include <random>
#include <regex>
#include <set>
#include <thread>
using namespace cp;
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        if (!(x))                                                                                            \
            throw std::runtime_error(std::string("line ") + std::to_string(__LINE__) + ": " #x);             \
    } while (false)
template <class Exception = std::exception, class F> void rejects(F f) {
    bool thrown = false;
    try {
        f();
    } catch (const Exception &) {
        thrown = true;
    }
    CHECK(thrown);
}
void close(double a, double b, double tolerance = 1e-10) {
    CHECK(std::abs(a - b) <= tolerance * (1 + std::abs(b)));
}
std::vector<std::vector<int>> words(int length) {
    std::vector<std::vector<int>> out{{}};
    for (int n = 1; n <= length; ++n)
        for (int mask = 0; mask < (1 << n); ++mask) {
            std::vector<int> w(n);
            for (int i = 0; i < n; ++i)
                w[i] = mask >> i & 1;
            out.push_back(w);
        }
    return out;
}
bool nfa_reference(const NFA &a, const std::vector<int> &w) {
    std::vector<std::vector<bool>> seen(w.size() + 1, std::vector<bool>(a.size()));
    std::vector<std::pair<int, int>> queue{{a.start(), 0}};
    seen[0][a.start()] = true;
    for (std::size_t i = 0; i < queue.size(); ++i) {
        auto [q, p] = queue[i];
        if (p == int(w.size()) && a.accepting(q))
            return true;
        for (auto e : a.edges(q)) {
            int next = p + (e.symbol != -1);
            if (e.symbol != -1 && (p == int(w.size()) || w[p] != e.symbol))
                continue;
            if (!seen[next][e.to]) {
                seen[next][e.to] = true;
                queue.push_back({e.to, next});
            }
        }
    }
    return false;
}
int main() {
    std::mt19937 rng(881726);
    Random random(771);
    UndoManager manager;
    std::vector<int> state(8);
    auto root = manager.snapshot();
    rejects<std::logic_error>([&] {
        manager.perform(
            [&] {
                state[0] = 8;
                manager.clear_history();
            },
            [&]() noexcept { state[0] = 0; });
    });
    CHECK(state[0] == 0 && manager.history_size() == 0);
    {
        Random a(91), b(91);
        const auto u = sampling_detail::open_uniform(b);
        close(sample_truncated_exponential(std::numeric_limits<double>::denorm_min(), 0, 1, a), u);
        CHECK(a() == b());
        NFA n(4, 1);
        n.add_transition(0, -1, 1);
        n.add_transition(1, 0, 2);
        n.add_transition(2, -1, 3);
        CHECK(n.advance({0}, 0) == std::vector<int>({2, 3}));
    }
    for (int i = 0; i < 300; ++i) {
        auto before = state;
        auto mark = manager.snapshot();
        manager.transaction([&](auto &h) {
            for (int j = 0; j < 5; ++j) {
                int p = rng() % 8;
                h.set(state[p], int(rng() % 100));
            }
        });
        manager.rollback(mark);
        CHECK(state == before);
    }
    rejects<std::runtime_error>([&] {
        manager.transaction([&](auto &h) {
            h.set(state[0], 20);
            h.transaction([&](auto &inner) { inner.set(state[1], 50); });
            throw std::runtime_error("cancel");
        });
    });
    CHECK(state == std::vector<int>(8));
    rejects<std::runtime_error>([&] {
        manager.perform(
            [&] {
                state[0] = 7;
                throw std::runtime_error("partial");
            },
            [&]() noexcept { state[0] = 0; });
    });
    CHECK(state[0] == 0);
    manager.set(state[0], 1);
    auto stale = manager.snapshot();
    manager.rollback(root);
    manager.set(state[0], 2);
    rejects<std::invalid_argument>([&] { manager.rollback(stale); });
    manager.undo_last();
    CHECK(state[0] == 0);
    UndoManager other;
    rejects<std::invalid_argument>([&] { other.rollback(root); });
    manager.clear_history();
    rejects<std::invalid_argument>([&] { manager.rollback(root); });
    manager.set(state[0], 1);
    auto old = manager.snapshot();
    manager.set(state[0], 2);
    rejects<std::invalid_argument>([&] { manager.transaction([&](auto &h) { h.rollback(old); }); });
    CHECK(state[0] == 2);
    manager.clear_history();
    auto sum = [](const std::vector<int> &s) { return std::accumulate(s.begin(), s.end(), 0); };
    CHECK(audit_score(state, 2, sum).consistent);
    CHECK(!audit_score(state, 3, sum).consistent);
    CHECK(audit_score(1., 1. + 1e-10, [](double x) { return x; }, {1e-9L, 0}).consistent);
    IndexSet index(25);
    std::set<int> reference;
    for (int i = 0; i < 3000; ++i) {
        int x = rng() % 25;
        if (rng() % 2)
            CHECK(index.insert(x) == reference.insert(x).second);
        else
            CHECK(index.erase(x) == bool(reference.erase(x)));
        CHECK(index.size() == reference.size());
        for (int e = 0; e < 25; ++e)
            CHECK(index.contains(e) == bool(reference.count(e)));
        for (std::size_t p = 0; p < index.size(); ++p)
            CHECK(index.position(index.at(p)) == int(p));
        auto sampled = index.sample(random);
        CHECK(bool(sampled) == !reference.empty());
        if (sampled)
            CHECK(reference.count(*sampled));
    }
    IndexSet copied = index;
    copied.clear();
    for (int i = 0; i < 25; ++i)
        CHECK(copied.insert(i));
    IndexSet moved(std::move(copied));
    CHECK(moved.size() == 25 && copied.universe_size() == 0);
    index = std::move(index);
    CHECK(index.size() == reference.size());
    rejects<std::out_of_range>([&] { index.insert(25); });
    for (int trial = 0; trial < 120; ++trial) {
        int n = 2 + rng() % 8;
        std::vector<int> tour(n);
        std::iota(tour.begin(), tour.end(), 0);
        std::shuffle(tour.begin(), tour.end(), rng);
        std::vector<std::vector<long long>> w(n, std::vector<long long>(n));
        for (auto &row : w)
            for (auto &x : row)
                x = rng() % 100;
        auto d = [&](int a, int b) { return w[a][b]; };
        for (bool symmetric : {false, true}) {
            if (symmetric)
                for (int i = 0; i < n; ++i)
                    for (int j = 0; j < i; ++j)
                        w[j][i] = w[i][j];
            auto base = tsp_tour_cost(tour, d);
            for (int l = 0; l <= n; ++l)
                for (int r = l; r <= n; ++r) {
                    TspTwoOpt move{std::size_t(l), std::size_t(r)};
                    auto cost = two_opt_delta(tour, move, d, symmetric);
                    auto copy = tour;
                    apply_two_opt(copy, move);
                    CHECK(cost == tsp_tour_cost(copy, d) - base);
                    undo_two_opt(copy, move);
                    CHECK(copy == tour);
                }
            for (int l = 0; l < n; ++l)
                for (int k = 1; k <= std::min(3, n - l); ++k)
                    if (k < n)
                        for (int after = 0; after < n; ++after)
                            if (after < l || after >= l + k) {
                                TspOrOpt move{std::size_t(l), std::size_t(k), std::size_t(after)};
                                auto cost = or_opt_delta(tour, move, d);
                                auto copy = tour;
                                apply_or_opt(copy, move);
                                CHECK(cost == tsp_tour_cost(copy, d) - base);
                                undo_or_opt(copy, move);
                                CHECK(copy == tour);
                            }
        }
    }
    FastAnnealingAcceptance table(8);
    CHECK(table.accept_index(0, std::numeric_limits<long double>::denorm_min(), 255));
    rejects<std::out_of_range>([&] { table.accept_index(0, 1, 256); });
    for (int i = 1; i <= 300; ++i) {
        long double loss = i * .031L;
        std::size_t accepted = 0;
        for (std::size_t j = 0; j < table.thresholds().size(); ++j)
            accepted += table.accept_index(loss, 1, j);
        CHECK(std::abs(static_cast<long double>(accepted) / 256 - std::exp(-loss)) <=
              table.probability_error_bound() + 1e-15L);
    }
    Random table_rng(4), table_reference(4);
    CHECK(table(0, 1, table_rng));
    CHECK(table_rng() == table_reference());
    for (int i = 1; i <= 1000; ++i) {
        auto k = table_reference() >> 56;
        CHECK(table(i * .01L, 2, table_rng) == table.accept_index(i * .01L, 2, k));
    }
    auto eval = [](const int &x) { return (x - 3) * (x - 3); };
    auto moves = [](const int &x) { return std::vector<int>{x > -10 ? -2 : 1, x < 10 ? 1 : -1}; };
    auto delta = [&](const int &x, const int &m) { return eval(x + m) - eval(x); };
    auto apply = [](int &x, const int &m) { x += m; };
    auto undo = [](int &x, const int &m) noexcept { x -= m; };
    for (int trial = 0; trial < 150; ++trial) {
        int initial = int(rng() % 11) - 5, current = initial;
        BeamSearchOptions o;
        o.width = 1 + rng() % 8;
        o.max_depth = rng() % 8;
        o.goal = rng() % 2 ? SearchGoal::minimize : SearchGoal::maximize;
        auto expected = beam_search(
            initial, eval,
            [&](int x) {
                auto m = moves(x);
                for (auto &v : m)
                    v += x;
                return m;
            },
            o);
        auto actual = delta_beam_search(current, eval, moves, delta, apply, undo, o);
        CHECK(current == initial && actual.best_state == expected.best().state &&
              actual.best_score == expected.best().score && actual.depth == expected.depth);
        for (int m : actual.moves)
            current += m;
        CHECK(current == actual.best_state);
    }
    int mutable_state = 0, calls = 0;
    BeamSearchOptions limited;
    limited.width = 3;
    limited.max_depth = 6;
    limited.stop = [&] { return ++calls == 13; };
    auto stopped = delta_beam_search(mutable_state, eval, moves, delta, apply, undo, limited);
    CHECK(stopped.termination == SearchTermination::stopped && mutable_state == 0);
    rejects<std::runtime_error>([&] {
        delta_beam_search(
            mutable_state, eval, moves,
            [&](const int &x, const int &m) {
                if (x != 0)
                    throw std::runtime_error("delta");
                return delta(x, m);
            },
            apply, undo);
    });
    CHECK(mutable_state == 0);
    // The final solution copy must unwind the reconstructed path if it throws.
    struct CopyFailure {
        int value = 0;
        CopyFailure() = default;
        CopyFailure(const CopyFailure &) { throw std::runtime_error("solution copy"); }
    } copy_failure;
    BeamSearchOptions copy_options;
    copy_options.max_depth = 3;
    copy_options.width = 1;
    rejects<std::runtime_error>([&] {
        delta_beam_search(
            copy_failure, [](const CopyFailure &x) { return -x.value; },
            [](const CopyFailure &) { return std::vector<int>{1}; },
            [](const CopyFailure &, const int &m) { return -m; },
            [](CopyFailure &x, const int &m) { x.value += m; },
            [](CopyFailure &x, const int &m) noexcept { x.value -= m; }, copy_options);
    });
    CHECK(copy_failure.value == 0);
    {
        Random custom_rng(88), reference_rng(88);
        AnnealingOptions options;
        options.max_iterations = 40;
        auto result = simulated_annealing_delta_acceptance(
            0, [](const int &x) { return x; }, [](const int &, Random &) { return 1; },
            [](const int &, const int &m) { return m; }, apply, undo, custom_rng, std::ref(table), options);
        int expected = 0;
        for (int step = 0; step < 40; ++step)
            expected +=
                table(1, annealing_temperature(options, static_cast<long double>(step) / 39), reference_rng);
        CHECK(result.current_state == expected && result.best_state == 0);
        CHECK(custom_rng() == reference_rng());
    }
    for (int depth = 0; depth <= 7; ++depth)
        for (bool maximum : {false, true}) {
            ChokudaiOptions o;
            o.max_depth = depth;
            o.goal = maximum ? SearchGoal::maximize : SearchGoal::minimize;
            auto actual = chokudai_search(0, eval, [](int x) { return std::vector<int>{x - 1, x + 1}; }, o);
            int best = maximum ? -1 : 100000;
            for (int i = 0; i <= depth; ++i) {
                int value = eval(2 * i - depth);
                best = maximum ? std::max(best, value) : std::min(best, value);
            }
            CHECK(actual.reached_target() && actual.best().score == best &&
                  actual.termination == ChokudaiTermination::exhausted);
        }
    ChokudaiOptions co;
    co.max_nodes = 1;
    CHECK(chokudai_search(
              0, eval, [](int x) { return std::vector<int>{x + 1}; }, co)
              .termination == ChokudaiTermination::node_limit);
    co.max_nodes = 100;
    co.max_expansions = 0;
    CHECK(chokudai_search(0, eval, [](int x) { return std::vector<int>{x + 1}; }, co).best().state == 0);
    AnnealingOptions ao;
    ao.max_iterations = 60;
    auto factory = [](std::size_t i, Random &r) { return int(i) + int(r.uniform(7)); };
    auto propose = [](int x, Random &r) { return x + (r.uniform(2) ? 1 : -1); };
    auto serial = multi_start_sa(20, factory, eval, propose, 55, ao);
    for (std::size_t workers : {1, 2, 4}) {
        ParallelSearchOptions po;
        po.workers = workers;
        auto parallel = parallel_multi_start_sa(20, factory, eval, propose, 55, ao, po);
        CHECK(parallel.completed == 20 && *parallel.best_run_index == serial.best_run_index);
        for (int i = 0; i < 20; ++i) {
            CHECK(parallel.runs[i]->best_state == serial.runs[i].best_state);
            CHECK(parallel.runs[i]->current_state == serial.runs[i].current_state);
        }
    }
    struct Run {
        int best_score;
    };
    std::atomic<int> alive{0};
    std::mutex lock;
    std::condition_variable condition;
    int ready = 0;
    auto together = parallel_multi_start(2, [&](std::size_t i, Random &, const std::atomic<bool> &) {
        std::unique_lock<std::mutex> held(lock);
        ++ready;
        condition.notify_all();
        condition.wait(held, [&] { return ready == 2; });
        ++alive;
        return Run{int(i)};
    });
    CHECK(together.completed == 2 && alive == 2);
    rejects<std::runtime_error>([&] {
        parallel_multi_start(8, [](std::size_t, Random &, const std::atomic<bool> &) -> Run {
            throw std::runtime_error("worker");
        });
    });
    ParallelSearchOptions po;
    po.stop = [] { return true; };
    auto cancelled = parallel_multi_start(
        3, [](std::size_t, Random &, const std::atomic<bool> &) { return Run{0}; }, 1, po);
    CHECK(cancelled.completed == 0 && !cancelled.best_run_index && cancelled.stopped);
    for (int trial = 0; trial < 120; ++trial) {
        int n = 1 + rng() % 7;
        std::vector<std::uint64_t> w(n);
        for (auto &x : w)
            x = rng() % 8;
        if (std::accumulate(w.begin(), w.end(), 0ULL) == 0)
            w[0] = 1;
        AliasSampler alias(w);
        std::vector<std::uint64_t> frequency(n);
        for (int i = 0; i < n; ++i)
            for (std::uint64_t coin = 0; coin < alias.total_weight(); ++coin)
                ++frequency[alias.choose(i, coin)];
        for (int i = 0; i < n; ++i)
            CHECK(frequency[i] == n * w[i]);
    }
    AliasSampler huge({UINT64_MAX - 1, 1});
    for (int i = 0; i < 100; ++i)
        CHECK(huge.sample(random) < 2);
    rejects<std::overflow_error>([] { AliasSampler invalid({UINT64_MAX, 1}); });
    std::vector<std::vector<int>> pair_count(3, std::vector<int>(3));
    for (int i = 0; i < 40000; ++i) {
        auto s = weighted_sample_without_replacement({1, 2, 3, 0}, 2, random);
        CHECK(s.size() == 2 && s[0] != s[1] && s[0] < 3 && s[1] < 3);
        ++pair_count[s[0]][s[1]];
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (i != j)
                close(pair_count[i][j] / 40000., (i + 1) / 6. * (j + 1) / (5 - i), .015);
    NormalSampler normal;
    double sum_normal = 0, squares = 0, sum_exp = 0;
    for (int i = 0; i < 100000; ++i) {
        double x = normal.sample(random);
        sum_normal += x;
        squares += x * x;
        sum_exp += sample_exponential(2, random);
        double y = sample_truncated_exponential(2, 1, 3, random);
        CHECK(y >= 1 && y <= 3);
    }
    close(sum_normal / 100000, 0, .015);
    close(squares / 100000, 1, .015);
    close(sum_exp / 100000, .5, .01);
    Random nr(8), nr_copy(8);
    normal.reset();
    normal.sample(nr);
    normal.sample(nr);
    nr_copy();
    nr_copy();
    CHECK(nr() == nr_copy());
    CHECK(sample_truncated_exponential(1, 2, 2, random) == 2);
    CHECK(normal.sample(random, 4, 0) == 4);
    rejects<std::invalid_argument>([&] { sample_exponential(0, random); });
    CHECK(evaluate_integer_expression(parse_expression(" -2^2 + 3 * (x+1) "), {{"x", 4}}) == 11);
    CHECK(evaluate_integer_expression(parse_expression("2^3^2")) == 512);
    CHECK(evaluate_integer_expression(parse_expression("-7/3 + -7%3")) == -3);
    CHECK(evaluate_integer_expression(parse_expression("0003")) == 3);
    auto tree = parse_expression("(2+3)*4");
    CHECK(tree.text(tree.root()) == "(2+3)*4");
    close(evaluate_expression<double>(
              tree, [](std::string_view s) { return std::stod(std::string(s)); },
              [](std::string_view) { return 0.; }, [](char op, double x) { return op == '-' ? -x : x; },
              [](char op, double x, double y) { return op == '+' ? x + y : x * y; }),
          20);
    for (int i = 0; i < 300; ++i) {
        long long a = int(rng() % 101) - 50, b = int(rng() % 101) - 50, c = int(rng() % 101) - 50;
        auto expression = "(" + std::to_string(a) + "+" + std::to_string(b) + ")*" + std::to_string(c);
        CHECK(evaluate_integer_expression(parse_expression(expression)) == BigInt((a + b) * c));
    }
    for (const auto &s : std::vector<std::string>{"", "1+", "(1", "1 2", "*2", "a(2)"})
        rejects<ParseError>([&] { parse_expression(s); });
    try {
        parse_expression("1+");
        CHECK(false);
    } catch (const ParseError &e) {
        CHECK(e.kind() == ParseErrorKind::expected_operand && e.position() == 2);
    }
    rejects<ParseError>([] { evaluate_integer_expression(parse_expression("1/0")); });
    rejects<ParseError>([] { evaluate_integer_expression(parse_expression("2^-1")); });
    rejects<ParseError>([] { parse_expression("(((1)))", {100, 1}); });
    rejects<ParseError>([] { parse_expression("1+2", {2, 20}); });
    const auto language = words(6);
    for (int trial = 0; trial < 100; ++trial) {
        int n = 1 + rng() % 5;
        NFA a(n, 2);
        for (int i = 0; i < n; ++i) {
            a.set_accepting(i, rng() % 2);
            for (int k = 0; k < 3; ++k)
                if (rng() % 2)
                    a.add_transition(i, int(rng() % 3) - 1, rng() % n);
        }
        auto d = determinize(a);
        auto minimal = minimize_dfa(d.automaton);
        auto inverse = complement_dfa(minimal.automaton);
        for (const auto &w : language) {
            bool want = nfa_reference(a, w);
            CHECK(a.accepts(w) == want && d.automaton.accepts(w) == want &&
                  minimal.automaton.accepts(w) == want && inverse.accepts(w) != want);
        }
        const auto &m = minimal.automaton;
        std::vector<std::vector<bool>> distinct(m.size(), std::vector<bool>(m.size()));
        for (int i = 0; i < m.size(); ++i)
            for (int j = 0; j < m.size(); ++j)
                distinct[i][j] = m.accepting(i) != m.accepting(j);
        bool change = true;
        while (change) {
            change = false;
            for (int i = 0; i < m.size(); ++i)
                for (int j = 0; j < i; ++j)
                    for (int c = 0; c < 2; ++c)
                        if (!distinct[i][j] && distinct[m.transition(i, c)][m.transition(j, c)]) {
                            distinct[i][j] = distinct[j][i] = true;
                            change = true;
                        }
        }
        for (int i = 0; i < m.size(); ++i)
            for (int j = 0; j < i; ++j)
                CHECK(distinct[i][j]);
        DFA b(3, 2);
        for (int i = 0; i < 3; ++i) {
            b.set_accepting(i, rng() % 2);
            for (int c = 0; c < 2; ++c)
                b.set_transition(i, c, int(rng() % 4) - 1);
        }
        for (auto op : {AutomatonBoolean::intersection, AutomatonBoolean::unite, AutomatonBoolean::difference,
                        AutomatonBoolean::symmetric_difference}) {
            auto product = dfa_product(d.automaton, b, op);
            for (const auto &w : language) {
                bool x = d.automaton.accepts(w), y = b.accepts(w);
                bool want = op == AutomatonBoolean::intersection ? x && y
                            : op == AutomatonBoolean::unite      ? x || y
                            : op == AutomatonBoolean::difference ? x && !y
                                                                 : x != y;
                CHECK(product.automaton.accepts(w) == want);
            }
        }
    }
    DFA decimal(3, 10);
    for (int q = 0; q < 3; ++q)
        for (int d = 0; d < 10; ++d)
            decimal.set_transition(q, d, (q + d) % 3);
    decimal.set_accepting(0);
    for (int upper = 0; upper < 300; ++upper)
        CHECK(dfa_digit_count(std::to_string(upper), decimal) == upper / 3 + 1);
    auto exponential = regex_to_nfa("(a|b)*a(a|b)(a|b)");
    rejects<std::length_error>([&] { determinize(exponential, 1); });
    std::vector<std::string> patterns{"",       "a",     "a|",   "(ab|ba)*",    "(a?b+)*",
                                      "[a-b]+", "[^a]*", "a.*b", "\\x61(b|a)?", "(a|b)*a(a|b)(a|b)"};
    for (int i = 0; i < 40; ++i)
        patterns.push_back("(" + std::string(rng() % 2 ? "a" : "b") + "|" +
                           std::string(rng() % 2 ? "ab" : "ba") + ")" + (rng() % 2 ? "*" : "+") +
                           std::string(rng() % 2 ? "a?" : "b?"));
    for (const auto &p : patterns) {
        auto a = regex_to_nfa(p);
        std::regex reference_regex(p);
        for (const auto &w : language) {
            std::string text;
            for (int c : w)
                text += char('a' + c);
            CHECK(regex_full_match(a, text) == std::regex_match(text, reference_regex));
        }
    }
    CHECK(regex_full_match(regex_to_nfa("\\0"), std::string(1, '\0')));
    CHECK(regex_full_match(regex_to_nfa("."), "\n"));
    for (const auto &p :
         std::vector<std::string>{"(", "[", "[]", "[z-a]", "\\", "\\xq1", "a**", "a{2}", "^a"})
        rejects<ParseError>([&] { regex_to_nfa(p); });
    std::cout
        << "E/X: undo/exception histories, 3000 IndexSet operations, 120 exhaustive TSP neighborhoods, table "
           "error bounds, 150 beam comparisons, complete chokudai trees, 1/2/4 workers, 120 exact alias "
           "tables, distribution moments, 300 expressions, 100 automata and 50 regex languages passed\n";
}
