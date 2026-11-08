---
title: Beam Searchの重複除去
documentation_of: //heuristic_search/beam_search_unique.hpp
---

# Beam Searchの重複除去

同じ深さの候補をkeyで統合し、各keyの最良候補を残してからBeam幅を制限します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#beam_unique) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`beam_search_unique(initial,evaluate,expand,key_of, options={}, hash={}, equal={}) -> BeamSearchResult<State,Score>`。既存のBeamSearchOptionsと結果型を共有します。`beam_search`は従来どおり重複を残します。

全ての生成候補を評価し、score順に安定ソートして、同じkeyの最良1個を残します。同点は生成順（親の順・子の順）。その後widthまで選びます。異なる深さの同じkeyは除去しません。keyの同値性が探索上適切なことは利用者が保証します。

keyはコピー可能でhash/equalに対応。hash衝突はequalで比較するため、異なるkeyは潰しません。Zobrist値をkeyにする場合、Zobrist値自体の衝突は別です。全候補を評価するため、重複の評価回数もevaluationsへ含みます。stopが候補走査の途中なら旧frontierを返します。

## Usage

```cpp
#include <iostream>
#include "heuristic_search/beam_search_unique.hpp"
int main() {
    cp::BeamSearchOptions options; options.width = 3; options.max_depth = 1;
    auto r = cp::beam_search_unique(0, [](int x) { return x; },
        [](int) { return std::vector<int>{1, 1, 2, 3}; }, [](int x) { return x; }, options);
    for (std::size_t i = 0; i < r.frontier.size(); ++i) {
        if (i) std::cout << ' ';
        std::cout << r.frontier[i].state;
    }
    std::cout << '\n';
}
```

出力:

```text
1 2 3
```

[コンパイルできる使用例](../../examples/heuristic_search/beam_search_unique.cpp)。

## 原理

候補のscore順にkeyを集合へ入れると、初回に現れた候補がそのkeyの最良です。重複を飛ばしながら幅を満たすまで選ぶため、先に幅を切って重複を除く場合の候補不足を避けます。

## 計算量

1深さで候補C個に対しO(C log C)比較と平均O(C)のkey処理、O(C)候補領域。最悪のhash衝突では集合の保持数を掛けた時間。評価・状態コピー・key生成の費用は別です。

## 使用する問題と検証

同じ状態へ複数経路から到達するBeam Search。keyが同じでもscoreが異なる例、全て同点、hash関数が常に0でも異なるkeyを保持する例を検証します。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
