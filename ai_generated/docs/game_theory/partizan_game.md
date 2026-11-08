---
title: Partizan Gameの基本操作
documentation_of: //game_theory/partizan_game.hpp
---

# Partizan Gameの基本操作

左右で異なる選択肢を持つ有限short gameを保持し、和・負号・差・ゲーム値の部分順序を計算します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#partizan_game) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`PartizanGameArena(max_nodes=100000,max_birthday=128)`、`make(left_ids,right_ids)`、`integer/add/negate/subtract/compare/outcome`、`less_equal/node/size`。ID 0は零。

子は既存IDのみなのでDAG。makeは同じ構文を共有しますが、戦略的に等価な異なるIDを同じ正規形へ簡約しません。compareはless/equal/greater/fuzzy、outcomeはleft/right/previous/next wins。birthdayは0..256まで。予算超過はlength_error、部分的に生成したノードは残ります。nodeの参照は次の生成で無効になる場合があります。超限・loopy partizanは対象外。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../game_theory/partizan_game.hpp"

int main() {
    cp::PartizanGameArena games;
    int star = games.make({0}, {0});
    std::cout << (games.compare(star, 0) == cp::PartizanOrder::fuzzy) << " " << (games.compare(games.add(star, star), 0) == cp::PartizanOrder::equal) << "\n";
}
```

出力:

```text
1 1
```

[コンパイルして動く使用例](../../examples/game_theory/partizan_game_b.cpp)。

## 計算量

生成するゲームと比較する局面対の数に依存し、最悪指数的。memoを保存するため、上限max_nodesとbirthday予算を指定。

## 使用する問題・検証

有限完全情報の左右非対称な通常ルール、*={0|0}、↑={0|*}などの和の比較。

整数の和・負号、star/up、有限ゲーム木を左右それぞれから完全に解いたoutcomeと差の比較を照合。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
