// Local microbenchmark: excludes score evaluation, includes RNG and public acceptance checks.
#include "heuristic_search/fast_annealing_acceptance.hpp"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>
int main() {
    using Clock = std::chrono::steady_clock;
    constexpr std::size_t count = 2000000;
    cp::Random input(719);
    std::vector<long double> loss(count), temperature(count);
    for (std::size_t i = 0; i < count; ++i) {
        loss[i] = .001L + 8 * input.real();
        temperature[i] = .1L + 4 * input.real();
    }
    auto start = Clock::now();
    cp::FastAnnealingAcceptance table(12);
    auto build = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    std::vector<double> exact_times, table_times;
    std::uint64_t exact_count = 0, table_count = 0;
    for (int round = 0; round < 8; ++round) {
        // Alternate ordering. The first pair warms caches and is discarded.
        for (int order = 0; order < 2; ++order) {
            const bool fast = (order ^ (round & 1)) != 0;
            cp::Random rng(9127);
            std::uint64_t accepted = 0;
            start = Clock::now();
            if (fast) {
                for (std::size_t i = 0; i < count; ++i)
                    accepted += table(loss[i], temperature[i], rng);
            } else {
                for (std::size_t i = 0; i < count; ++i)
                    accepted += rng.real() < std::exp(-loss[i] / temperature[i]);
            }
            const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            if (fast)
                table_count = accepted;
            else
                exact_count = accepted;
            if (round)
                (fast ? table_times : exact_times).push_back(ms);
        }
    }
    std::sort(exact_times.begin(), exact_times.end());
    std::sort(table_times.begin(), table_times.end());
    std::cout << std::fixed << std::setprecision(3) << "compiler=" << __VERSION__ << " trials=" << count
              << " repetitions=7\n"
              << "table_build_ms=" << build
              << " table_bytes=" << table.thresholds().size() * sizeof(long double) << '\n'
              << "exp_median_ms=" << exact_times[3] << " table_median_ms=" << table_times[3]
              << " ratio=" << exact_times[3] / table_times[3] << '\n'
              << "exp_accepted=" << exact_count << " table_accepted=" << table_count << '\n';
}
