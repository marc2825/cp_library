#include <iostream>
#include "heuristic_search/zobrist_hash.hpp"
int main() {
    cp::ZobristHash z(3, 4, 42);
    std::vector<int> board{0, 1, 2};
    const auto original = z.hash(board);
    const auto changed = z.change(original, 1, 1, 3);
    board[1] = 3;
    std::cout << (changed == z.hash(board)) << '\n';
    std::cout << (z.change(changed, 1, 3, 1) == original) << '\n';
}
