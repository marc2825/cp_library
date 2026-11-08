---
title: TSPの2-opt・Or-opt
documentation_of: //heuristic_search/tsp_neighborhood.hpp
---

# E-03 TSPの2-opt・Or-opt

[一覧](README.md) / [ヘッダ](../../heuristic_search/tsp_neighborhood.hpp) / [実行例](../../examples/search_parsing_extensions/tsp_neighborhood.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#tsp_neighborhood) / [出典](references.md)

取り除く辺と加える辺だけで差分を求め、列の反転・回転で巡回路を変更する。

## APIと条件

tsp_tour_cost、two_opt_delta・apply_two_opt・undo_two_opt、or_opt_delta・apply_or_opt・undo_or_opt。

巡回列vector<int>と距離callback。2-optは半開区間の反転、Or-optは非巡回blockを元添字afterの後へ移す。symmetric=trueは距離の対称性を要求。undoは同じmoveを正しく適用した直後に行う。

TspTwoOpt{first,last}は[first,last)を反転します。空/1要素区間はdelta=0。全周反転は対称なら0、非対称ならすべての向きの変化を合計します。対称性は呼び出し側の条件で、距離行列全体を検査しません。

TspOrOpt{first,count,after}は元の[first,first+count)を元添字afterの直後へ移します。countは1〜n−1、afterはblockの外側。元の直前の頂点（循環添字）を指定すると変更しません。通常の1〜3頂点移動以外も扱います。先頭を動かすと巡回列の代表位置が変わります。

distance(u,v)は符号付き64bit以下の整数または浮動Scoreを返します。差分の途中の加減算もoverflowを検査し、浮動値は有限を要求します。undoは検査しないnoexcept操作です。apply前と同じmove/列長を使い、他の変更は先に逆順で戻してください。

## 計算量・用途

全コストO(n)。対称2-optとOr-optのdeltaはO(1)、非対称2-optはO(k)。2-opt適用O(k)、Or-opt適用/復元O(n)、追加領域O(1)。

問題例・用途: 巡回セールスマン問題の局所改善・焼きなまし。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "heuristic_search/tsp_neighborhood.hpp"
#include <iostream>
using namespace cp;
int main() {
    std::vector<int> tour{0, 1, 2, 3};
    const long long costs[4][4] = {{0, 8, 4, 4}, {8, 0, 4, 4}, {4, 4, 0, 8}, {4, 4, 8, 0}};
    auto distance = [&](int a, int b) { return costs[a][b]; };
    TspTwoOpt move{1, 3};
    std::cout << two_opt_delta(tour, move, distance) << " ";
    apply_two_opt(tour, move);
    std::cout << tsp_tour_cost(tour, distance) << " ";
    undo_two_opt(tour, move);
    std::cout << (tour == std::vector<int>{0, 1, 2, 3}) << "\n";
}
```

出力:

```text
-8 16 1
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
