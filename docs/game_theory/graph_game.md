---
title: グラフゲーム（閉路・引き分け）
documentation_of: //game_theory/graph_game.hpp
---

# グラフゲーム（閉路・引き分け）

閉路を持つ有限局面グラフを通常ルールで解析し、勝ち・負け・引き分けを分類します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#graph_game) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`graph_game(vector<vector<int>> moves)`→`{status,move,remoteness}`。GraphGameStatusはlose/win/draw。

終局は手番側の負け。一手で1本の辺を移動し、同じ局面の再訪を許します。履歴依存の禁止規則は状態に含めてください。moveは勝ち手または引き分けを保つ手、敗勢は-1。remotenessは勝つ側が短縮し負け側が延長する終局距離、drawは-1。重複辺・自己辺可。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../game_theory/graph_game.hpp"

int main() {
    auto result = cp::graph_game({{1}, {}, {2}});
    std::cout << (result.status[0] == cp::GraphGameStatus::win) << " " << (result.status[2] == cp::GraphGameStatus::draw) << "\n";
}
```

出力:

```text
1 1
```

[コンパイルして動く使用例](../../examples/game_theory/graph_game_b.cpp)。

## 計算量

O(V+E)時間・領域。逆辺と未確定の出次数を使うretrograde解析。

## 使用する問題・検証

有限有向グラフの追跡ゲーム、DAG以外の勝敗分類。

4頂点までの全有向グラフを独立な固定点更新と比較し、勝ち手・drawを保つ手・距離を検査。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
