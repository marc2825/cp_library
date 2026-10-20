#include "static_query/swag_deque.hpp"
#include "test_values.hpp"
#include <deque>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

using namespace static_test;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(#x) + " line " + std::to_string(__LINE__)); } while (false)
using TextDeque = cp::SWAGDeque<std::string, concatenate, empty_string>;
using AffineDeque = cp::SWAGDeque<Affine, concatenate_affine, identity>;

std::size_t exhaustive(const TextDeque& queue, const std::deque<std::string>& model, int depth) {
    std::string expected; for (const auto& x : model) expected += x;
    CHECK(queue.prod() == expected && queue.size() == int(model.size()) && queue.empty() == model.empty());
    if (!model.empty()) CHECK(queue.front() == model.front() && queue.back() == model.back());
    std::size_t checked = 1;
    if (depth == 0) return checked;
    for (int kind = 0; kind < 6; ++kind) {
        if (kind >= 4 && model.empty()) continue;
        auto next = queue; auto reference = model;
        if (kind < 4) {
            const std::string x = kind % 2 ? "b" : "a";
            if (kind < 2) { next.push_front(x); reference.push_front(x); }
            else { next.push_back(x); reference.push_back(x); }
        } else if (kind == 4) { next.pop_front(); reference.pop_front(); }
        else { next.pop_back(); reference.pop_back(); }
        checked += exhaustive(next, reference, depth - 1);
    }
    return checked;
}

void random_histories() {
    std::mt19937 rng(340101);
    for (int trial = 0; trial < 240; ++trial) {
        AffineDeque queue; std::deque<Affine> model;
        for (int step = 0; step < 1000; ++step) {
            int kind = rng() % 9;
            if (model.empty() && kind >= 2 && kind <= 3) kind -= 2;
            if (model.size() >= 64 && kind < 2) kind += 2;
            if (kind < 2) {
                Affine f{static_cast<long long>(rng() % mod), static_cast<long long>(rng() % mod)};
                if (step % 17 == 0) f.a = 0; // Noninvertible operations are valid.
                if (kind == 0) { queue.push_front(f); model.push_front(f); }
                else { queue.push_back(f); model.push_back(f); }
            } else if (kind == 2) { queue.pop_front(); model.pop_front(); }
            else if (kind == 3) { queue.pop_back(); model.pop_back(); }
            else if (kind == 8) { queue.clear(); model.clear(); }
            const auto& view = queue;
            CHECK(view.size() == int(model.size()) && view.empty() == model.empty());
            const auto combined = view.prod();
            for (long long x : {0, 1, 17}) {
                auto expected = x;
                for (const auto& f : model) expected = evaluate(f, expected);
                CHECK(evaluate(combined, x) == expected);
            }
            if (!model.empty()) {
                CHECK(view.front().a == model.front().a && view.front().b == model.front().b);
                CHECK(view.back().a == model.back().a && view.back().b == model.back().b);
            }
        }
    }
}

void boundaries() {
    TextDeque queue;
    CHECK(queue.empty() && queue.size() == 0 && queue.prod().empty());
    for (int i = 0; i < 100; ++i) {
        queue.push_back("x"); queue.pop_front();
        queue.push_front("y"); queue.pop_back();
        CHECK(queue.empty() && queue.prod().empty());
    }
    queue.push_back("a"); queue.push_front(queue.back());
    queue.push_back("b"); queue.push_back(queue.front());
    CHECK(queue.prod() == "aaba");
    auto copy = queue; copy.pop_front(); CHECK(copy.prod() == "aba" && queue.prod() == "aaba");
    queue.clear(); queue.push_front("c"); CHECK(queue.prod() == "c");
    cp::SWAGDeque<First, first, no_first> firsts;
    firsts.push_back(First(2)); firsts.push_back(First(3)); firsts.pop_front();
    CHECK(firsts.prod().value == 3);
    firsts.push_front(First(1)); firsts.pop_back(); CHECK(firsts.prod().value == 1);
    firsts.pop_back(); CHECK(firsts.prod().value == -1);
}

int remaining_operations = -1;
long long throwing_sum(long long a, long long b) {
    if (remaining_operations == 0) throw std::runtime_error("op");
    if (remaining_operations > 0) --remaining_operations;
    return a + b;
}
struct CopyValue {
    int value;
    inline static int budget = -1;
    explicit CopyValue(int v) : value(v) {}
    CopyValue(const CopyValue& other) : value(other.value) {
        if (budget == 0) throw std::runtime_error("copy");
        if (budget > 0) --budget;
    }
    CopyValue(CopyValue&&) noexcept = default;
};
CopyValue copy_sum(const CopyValue& a, const CopyValue& b) { return CopyValue(a.value + b.value); }
CopyValue copy_zero() { return CopyValue(0); }
void exceptions() {
    for (bool left : {false, true}) for (int budget = 0; budget < 6; ++budget) {
        cp::SWAGDeque<long long, throwing_sum, zero> queue;
        for (int x = 1; x <= 8; ++x) if (left) queue.push_back(x); else queue.push_front(x);
        remaining_operations = budget;
        bool threw = false;
        try { if (left) queue.pop_front(); else queue.pop_back(); }
        catch (const std::runtime_error&) { threw = true; }
        remaining_operations = -1;
        CHECK(threw && queue.size() == 8 && queue.prod() == 36);
        for (int x = 1; x <= 8; ++x) {
            CHECK((left ? queue.front() : queue.back()) == x);
            if (left) queue.pop_front(); else queue.pop_back();
        }
    }
    for (int budget = 0; budget < 20; ++budget) {
        cp::SWAGDeque<CopyValue, copy_sum, copy_zero> queue;
        for (int x = 1; x <= 8; ++x) queue.push_back(CopyValue(x));
        CopyValue::budget = budget; bool threw = false;
        try { queue.pop_front(); } catch (const std::runtime_error&) { threw = true; }
        CopyValue::budget = -1;
        CHECK(queue.size() == (threw ? 8 : 7) && queue.prod().value == (threw ? 36 : 35));
        CHECK(queue.front().value == (threw ? 1 : 2) && queue.back().value == 8);
    }
}

void linear_work() {
    using Queue = cp::SWAGDeque<long long, counted_sum, zero>;
    for (bool left : {false, true}) for (bool alternating : {false, true}) {
        operation_count = 0; int updates = 0; Queue queue;
        for (int i = 0; i < 200000; ++i) {
            if (left) queue.push_front(1); else queue.push_back(1);
            ++updates;
        }
        const int calls = operation_count;
        for (int i = 0; i < 1000; ++i) CHECK(queue.front() == 1 && queue.back() == 1);
        CHECK(calls == operation_count); // Peeking never triggers rebuilding.
        bool from_front = !left;
        while (!queue.empty()) {
            if (from_front) queue.pop_front(); else queue.pop_back();
            if (alternating) from_front = !from_front;
            ++updates;
            CHECK(queue.prod() == queue.size());
            CHECK(operation_count <= 4 * updates + 8);
        }
    }
    operation_count = 0; Queue queue; int updates = 0;
    for (int i = 0; i < 513; ++i) { queue.push_back(1); ++updates; }
    for (int i = 0; i < 100000; ++i) {
        if (i % 2) { queue.pop_front(); queue.push_back(1); }
        else { queue.pop_back(); queue.push_front(1); }
        updates += 2; CHECK(queue.prod() == 513 && operation_count <= 4 * updates + 8);
    }
}
int main() {
    const auto states = exhaustive({}, {}, 8);
    random_histories(); boundaries(); exceptions(); linear_work();
    std::cout << "SWAG Deque: " << states << " exhaustive states, 240000 random operations, exception rollback and 200000-element linear work passed\n";
}
