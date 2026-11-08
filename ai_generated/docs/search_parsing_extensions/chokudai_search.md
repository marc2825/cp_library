---
title: Chokudai Search
documentation_of: //heuristic_search/chokudai_search.hpp
---

# E-06 Chokudai Search

[一覧](README.md) / [ヘッダ](../../heuristic_search/chokudai_search.hpp) / [実行例](../../examples/search_parsing_extensions/chokudai_search.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#chokudai_search) / [出典](references.md)

深さ別queueを消さずに保持し、後の周回でも見送った候補から探索を続ける。

## APIと条件

chokudai_search(initial,evaluate,expand,ChokudaiOptions)、深さ別best_at_depth。

各深さのpriority queueを巡回し、expansions_per_depth個ずつ展開する。max_expansions・max_nodes・stopで停止。目標深さの解を保持し、未達なら最深の候補。重複除去・最適性保証はしない。

ChokudaiOptionsはmax_depth/expansions_per_depth/max_expansions/max_nodes/goal/stop。expand(state)は子のvector<State>を返します。各深さから良い順に規定個数を取り出し、深いqueueへ送り、浅い深さへ戻って繰り返します。同点は生成順です。初期状態を含め、生成して評価した累積件数がmax_nodesです。

返り値はbest_at_depth（optional<State/score>）、deepest、expansions/evaluations/queued、termination。best().state/best().scoreで最深の保持解、reached_target()で目標深さ到達を確認できます。目標深さに到達しても予算まで他の枝を探索します。max_depth=0では初期状態が解です。

terminationはexhausted/expansion_limit/node_limit/stopped。stopを時計callbackにすれば時間予算になりますが、一回のexpandやevaluate内部を中断しません。全探索相当の予算を与えた小さい木と比較し、有限予算でも深さ別の最良値を保持することを検証しています。

## 計算量・用途

深さD・巡回R回・生成G個・最大queue数QでO(D(R+1)+G log(Q+1))と評価/コピー、領域O(G+D)。max_nodesは初期評価を含む累積生成予算。

問題例・用途: 時間内に探索の候補を増やし、深さ別に良い状態を保持する探索。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "heuristic_search/chokudai_search.hpp"
#include <iostream>
using namespace cp;
int main() {
    ChokudaiOptions options;
    options.max_depth = 4;
    options.max_expansions = 100;
    options.max_nodes = 1000;
    auto result = chokudai_search(
        0, [](int x) { return (x - 5) * (x - 5); }, [](int x) { return std::vector<int>{x - 1, x + 2}; },
        options);
    std::cout << result.best().state << " " << result.best().score << " " << result.reached_target() << "\n";
}
```

出力:

```text
5 0 1
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
