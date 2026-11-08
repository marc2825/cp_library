---
title: Tabu Search
documentation_of: //heuristic_search/tabu_search.hpp
---

# Tabu Search

最近訪問した状態への戻りを一時的に禁止し、悪化する手も採用して探索します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#tabu) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`tabu_search(initial,evaluate,neighbors,key_of, options={}, hash={}, equal={}) -> SearchResult<State,Score>`。基本のState・Score・stopの条件は[探索の共通仕様](README.md)と同じです。key_of(const State&)はコピー可能でhash/equalに対応するkeyを返します。keyが同じ状態は同じ禁止対象です。hashの衝突はequalで区別します。Zobrist値自体をkeyにした場合の状態衝突は利用側で管理してください。

`TabuSearchOptions : SearchOptions`にtenure（既定7）。初期状態は最初のtenure走査で禁止、走査sで訪問した状態は次のtenure走査で禁止です。tenure=0は無効化。size_tの期限超過は最大値へ飽和し、残りの走査中は禁止を維持します。

禁止されていない候補、または全体最良を厳密に更新する候補（aspiration）から最良を選びます。同点は先着。候補が全て禁止・空ならexhausted。完了した走査をiterationsへ数え、その走査で選べなくても1を加えます。bestは同点で更新しません。中途stopではその走査の候補を採用しません。最適性・品質は保証しません。

## Usage

```cpp
#include <iostream>
#include <vector>
#include "heuristic_search/tabu_search.hpp"
int main() {
    cp::TabuSearchOptions options; options.tenure = 1; options.max_iterations = 3;
    auto r = cp::tabu_search(0, [](int x) { return x; },
        [](int x) { return std::vector<int>{1 - x}; }, [](int x) { return x; }, options);
    std::cout << r.best_state << ' ' << r.current_state << ' ' << r.accepted << '\n';
}
```

出力:

```text
0 1 3
```

[コンパイルできる使用例](../../examples/heuristic_search/tabu_search.cpp)。

## 原理

訪問keyと期限を表に持ち、期限順のキューで古い記録を削除します。再訪で同じkeyの期限を更新した場合は、古いキュー項目で新しい禁止を削除しないようにします。現在解の改善だけに制限せず、全体最良を別に保存します。

## 計算量

候補数B・走査数I・tenure Tとして、平均的なhash/equal費用がO(1)ならO(IB)回の評価とkey処理、O(min(I+1,T+1))保持領域。候補生成・State/keyのコピーと最良保存の費用は別。unordered_mapの最悪時には保持数を掛けた時間になり得ます。

## 使用する問題と検証

局所最適から悪化手で抜け、循環を抑える探索。固定グラフの独立履歴シミュレーション、期限切れ・再訪・aspiration・停止・極値を比較します。オンラインのスコア改善は未確認です。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
