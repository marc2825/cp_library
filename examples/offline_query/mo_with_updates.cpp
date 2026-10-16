#include <iostream>
#include "offline_query/mo_with_updates.hpp"
int main() {
    cp::MoWithUpdates mo(3); mo.add_query(0, 3); mo.add_update(); mo.add_query(1, 3);
    int a[] = {1, 2, 3}, sum = 0, answer[2] = {};
    auto add = [&](int p) { sum += a[p]; }; auto remove = [&](int p) { sum -= a[p]; };
    auto change = [&](int value, int l, int r) { if (l <= 1 && 1 < r) sum += value - a[1]; a[1] = value; };
    mo.run(add, remove, [&](int, int l, int r) { change(7, l, r); },
        [&](int, int l, int r) { change(2, l, r); }, [&](int id) { answer[id] = sum; });
    std::cout << answer[0] << ' ' << answer[1] << '\n';
}
