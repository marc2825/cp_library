#pragma once
#include <optional>
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"
#include "smawk.hpp"

namespace cp {
struct MongeMatrixProductResult {
    std::vector<std::vector<std::optional<long long>>> value;
    std::vector<std::vector<int>> argmin;
};
// C[i][j]=min_k(A[i][k]+B[k][j])。Bは有限long longのMonge行列、Aは任意。
// B[k][j]+B[k+1][j+1] <= B[k][j+1]+B[k+1][j] が前提。
// 各A行ごとにB転置へSMAWK、O(rows*(inner+cols))評価。最左のkを復元。
template<class A, class B>
MongeMatrixProductResult monge_min_plus_product(int rows, int inner, int cols, A a, B b) {
    if (rows < 0 || inner < 0 || cols < 0) throw std::invalid_argument("negative Monge dimension");
    MongeMatrixProductResult result{
        std::vector<std::vector<std::optional<long long>>>(rows, std::vector<std::optional<long long>>(cols)),
        std::vector<std::vector<int>>(rows, std::vector<int>(cols, -1))};
    if (!inner) return result;
    for (int i = 0; i < rows; ++i) {
        auto get = [&](int j, int k) { return detail::dp_add(a(i,k), b(k,j)); };
        result.argmin[i] = smawk(cols, inner, get);
        for (int j = 0; j < cols; ++j) result.value[i][j] = get(j, result.argmin[i][j]);
    }
    return result;
}
} // namespace cp
