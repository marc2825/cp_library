#include "heuristic_search/all.hpp"
#include "parsing/all.hpp"
#include <iomanip>
#include <iostream>
#include <numeric>
using namespace cp;
int main() {
    std::cout << std::setprecision(17);
    char mode;
    while (std::cin >> mode) {
        if (mode == 'P') {
            std::string expression;
            std::cin >> std::quoted(expression);
            try {
                std::cout << evaluate_integer_expression(parse_expression(expression)) << '\n';
            } catch (const ParseError &e) {
                std::cout << "error " << int(e.kind()) << ' ' << e.position() << '\n';
            }
        } else if (mode == 'T') {
            int n, kind;
            std::cin >> n >> kind;
            std::vector<int> tour(n);
            for (auto &x : tour)
                std::cin >> x;
            std::vector<std::vector<long long>> matrix(n, std::vector<long long>(n));
            for (auto &row : matrix)
                for (auto &x : row)
                    std::cin >> x;
            auto distance = [&](int a, int b) { return matrix[a][b]; };
            auto before = tour;
            long long delta;
            if (kind == 0) {
                std::size_t l, r;
                std::cin >> l >> r;
                TspTwoOpt move{l, r};
                delta = two_opt_delta(tour, move, distance, false);
                apply_two_opt(tour, move);
                std::cout << delta << ' ' << tsp_tour_cost(tour, distance) << ' ';
                undo_two_opt(tour, move);
            } else {
                std::size_t l, k, a;
                std::cin >> l >> k >> a;
                TspOrOpt move{l, k, a};
                delta = or_opt_delta(tour, move, distance);
                apply_or_opt(tour, move);
                std::cout << delta << ' ' << tsp_tour_cost(tour, distance) << ' ';
                undo_or_opt(tour, move);
            }
            std::cout << (tour == before) << '\n';
        } else if (mode == 'B' || mode == 'C') {
            int initial, target, count;
            std::size_t depth, width;
            std::cin >> initial >> target >> depth >> width >> count;
            std::vector<int> moves(count);
            for (auto &x : moves)
                std::cin >> x;
            auto eval = [&](int x) { return (x - target) * (x - target); };
            if (mode == 'B') {
                BeamSearchOptions o;
                o.max_depth = depth;
                o.width = width;
                int current = initial;
                auto result = delta_beam_search(
                    current, eval, [&](int) { return moves; },
                    [&](int x, int m) { return eval(x + m) - eval(x); }, [](int &x, const int &m) { x += m; },
                    [](int &x, const int &m) noexcept { x -= m; }, o);
                std::cout << result.best_state << ' ' << result.best_score << ' ' << current << '\n';
            } else {
                ChokudaiOptions o;
                o.max_depth = depth;
                o.expansions_per_depth = width;
                auto result = chokudai_search(
                    initial, eval,
                    [&](int x) {
                        auto out = moves;
                        for (auto &m : out)
                            m += x;
                        return out;
                    },
                    o);
                std::cout << result.best().state << ' ' << result.best().score << ' ' << result.deepest
                          << '\n';
            }
        } else if (mode == 'I') {
            int n, q;
            std::cin >> n >> q;
            IndexSet s(n);
            while (q--) {
                int op, x;
                std::cin >> op >> x;
                if (op)
                    s.insert(x);
                else
                    s.erase(x);
            }
            auto values = s.values();
            std::sort(values.begin(), values.end());
            std::cout << values.size();
            for (int x : values)
                std::cout << ' ' << x;
            std::cout << '\n';
        } else if (mode == 'A') {
            int n;
            std::cin >> n;
            std::vector<std::uint64_t> w(n);
            for (auto &x : w)
                std::cin >> x;
            AliasSampler a(w);
            std::vector<int> counts(n);
            for (int i = 0; i < n; ++i)
                for (std::uint64_t coin = 0; coin < a.total_weight(); ++coin)
                    ++counts[a.choose(i, coin)];
            for (int i = 0; i < n; ++i)
                std::cout << (i ? " " : "") << counts[i];
            std::cout << '\n';
        } else if (mode == 'N') {
            int n, sigma, m, accept_count;
            std::cin >> n >> sigma >> m >> accept_count;
            NFA a(n, sigma);
            for (int i = 0; i < accept_count; ++i) {
                int q;
                std::cin >> q;
                a.set_accepting(q);
            }
            for (int i = 0; i < m; ++i) {
                int q, c, to;
                std::cin >> q >> c >> to;
                a.add_transition(q, c, to);
            }
            int size;
            std::cin >> size;
            std::vector<int> word(size);
            for (auto &x : word)
                std::cin >> x;
            auto d = determinize(a);
            auto minimal = minimize_dfa(d.automaton);
            std::cout << a.accepts(word) << ' ' << d.automaton.accepts(word) << ' '
                      << minimal.automaton.accepts(word) << '\n';
        } else if (mode == 'R') {
            std::string pattern, text;
            std::cin >> std::quoted(pattern) >> std::quoted(text);
            auto a = regex_to_nfa(pattern);
            std::cout << regex_full_match(a, text) << '\n';
        } else if (mode == 'F') {
            unsigned bits;
            double loss, temperature;
            std::cin >> bits >> loss >> temperature;
            FastAnnealingAcceptance table(bits);
            int count = 0;
            for (std::size_t i = 0; i < table.thresholds().size(); ++i)
                count += table.accept_index(loss, temperature, i);
            std::cout << count << '\n';
        } else if (mode == 'D') {
            std::uint64_t seed;
            std::cin >> seed;
            Random random(seed);
            NormalSampler normal;
            double a = normal.sample(random), b = normal.sample(random), c = sample_exponential(2, random),
                   d = sample_truncated_exponential(2, 1, 3, random);
            std::cout << a << ' ' << b << ' ' << c << ' ' << d << '\n';
        } else if (mode == 'M') {
            std::size_t starts, workers;
            std::uint64_t seed;
            std::cin >> starts >> workers >> seed;
            struct Run {
                int best_state, best_score;
            };
            ParallelSearchOptions o;
            o.workers = workers;
            auto r = parallel_multi_start(
                starts,
                [](std::size_t, Random &rng, const std::atomic<bool> &) {
                    Run r{0, 1000};
                    for (int i = 0; i < 3; ++i) {
                        int x = int(rng.uniform(100)), score = std::abs(x - 37);
                        if (score < r.best_score)
                            r = {x, score};
                    }
                    return r;
                },
                seed, o);
            std::cout << *r.best_run_index << ' ' << r.best_run().best_state << ' ' << r.best_run().best_score
                      << '\n';
        } else
            throw std::invalid_argument("unknown query");
    }
}
