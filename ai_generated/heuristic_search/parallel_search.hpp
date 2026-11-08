#pragma once
#include "multi_start_sa.hpp"
#include <atomic>
#include <exception>
#include <mutex>
#include <optional>
#include <thread>
namespace cp {
struct ParallelSearchOptions {
    std::size_t workers = 2;
    SearchGoal goal = SearchGoal::minimize;
    std::function<bool()> stop; // Serialized while assigning tasks. Cancellation is cooperative.
};
template <class Result> struct ParallelSearchResult {
    std::vector<std::optional<Result>> runs;
    std::optional<std::size_t> best_run_index;
    std::size_t completed = 0;
    bool stopped = false;
    const Result &best_run() const {
        if (!best_run_index)
            throw std::logic_error("no completed parallel run");
        return *runs[*best_run_index];
    }
};
// task(index,rng,cancel)->result with best_score. Each run gets a fresh task copy and fixed seed.
// User callbacks must not share unsynchronized mutable data. Join before returning/rethrowing.
template <class Task>
auto parallel_multi_start(std::size_t starts, Task task, std::uint64_t seed = 20261029,
                          ParallelSearchOptions options = {}) {
    using Result =
        std::decay_t<std::invoke_result_t<Task &, std::size_t, Random &, const std::atomic<bool> &>>;
    heuristic_detail::validate_goal(options.goal);
    if (!starts || !options.workers)
        throw std::invalid_argument("parallel search needs runs and workers");
    ParallelSearchResult<Result> out;
    out.runs.resize(starts);
    std::vector<std::uint64_t> seeds(starts);
    Random generator(seed);
    for (auto &value : seeds)
        value = generator();
    std::vector<std::exception_ptr> errors(starts);
    std::atomic<bool> cancel{false};
    std::mutex scheduler;
    std::size_t next = 0;
    const auto worker = [&] {
        for (;;) {
            std::size_t index;
            {
                std::lock_guard<std::mutex> lock(scheduler);
                if (cancel.load() || next == starts)
                    return;
                // A throwing stop callback is recorded against the next unstarted run.
                try {
                    if (options.stop && options.stop()) {
                        cancel.store(true);
                        return;
                    }
                } catch (...) {
                    errors[next] = std::current_exception();
                    cancel.store(true);
                    return;
                }
                index = next++;
            }
            try {
                auto local = task;
                Random random(seeds[index]);
                auto result = local(index, random, std::as_const(cancel));
                heuristic_detail::validate_score(result.best_score);
                out.runs[index] = std::move(result);
            } catch (...) {
                errors[index] = std::current_exception();
                cancel.store(true);
                return;
            }
        }
    };
    std::vector<std::thread> threads;
    threads.reserve(std::min(starts, options.workers));
    try {
        for (std::size_t i = 0; i < std::min(starts, options.workers); ++i)
            threads.emplace_back(worker);
    } catch (...) {
        cancel.store(true);
        for (auto &t : threads)
            t.join();
        throw;
    }
    for (auto &t : threads)
        t.join();
    for (const auto &error : errors)
        if (error)
            std::rethrow_exception(error);
    out.stopped = cancel.load();
    for (std::size_t i = 0; i < starts; ++i)
        if (out.runs[i]) {
            ++out.completed;
            if (!out.best_run_index ||
                heuristic_detail::better(out.runs[i]->best_score, out.best_run().best_score, options.goal))
                out.best_run_index = i;
        }
    return out;
}
template <class Factory, class Evaluate, class Propose>
auto parallel_multi_start_sa(std::size_t starts, Factory factory, Evaluate evaluate, Propose propose,
                             std::uint64_t seed = 20261029, AnnealingOptions annealing = {},
                             ParallelSearchOptions parallel = {}) {
    // The SA stop callback is also serialized, so Timer and stateful stop counters can be shared safely.
    std::mutex stop_mutex;
    const auto stop = annealing.stop;
    parallel.goal = annealing.goal;
    auto task = [=, &stop_mutex](std::size_t index, Random &random, const std::atomic<bool> &cancel) mutable {
        auto options = annealing;
        options.stop = [&] {
            if (cancel.load())
                return true;
            std::lock_guard<std::mutex> lock(stop_mutex);
            return stop && stop();
        };
        auto initial = factory(index, random);
        return simulated_annealing(std::move(initial), evaluate, propose, random, options);
    };
    return parallel_multi_start(starts, std::move(task), seed, std::move(parallel));
}
} // namespace cp
