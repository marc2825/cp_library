---
title: Mo with Updates
documentation_of: //offline_query/mo_with_updates.hpp
---

# Mo with Updates

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#mo) / [実装](../../offline_query/mo_with_updates.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
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
```

[実行できる使用例](../../examples/offline_query/mo_with_updates.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`MoWithUpdates(n)`、`add_update()->id`、`add_query(l,r)->id`、`query_order(block=0)`。更新の内容は利用側が保持します。質問にはその時点の更新件数を記録します。

`run(add,remove,apply,undo,answer,block=0)`、または左右別の `run(add_left,add_right,remove_left,remove_right,apply,undo,answer,block)`。add/removeは添字、apply/undoは `(update_id,current_l,current_r)` を受け取り、配列と集計の両方を変更します。answer(id)で結果を保存。開始前に初期配列・空の集計へ戻してください。終了時は最後の処理状態を保持します。空区間可。

## 計算量・原理

幅BでおおむねO(QB+n²/B+Un²/B²)回の位置・時間移動にソートO(Q log Q)を加算。n,Q,Uが同程度ならB≈n^(2/3)でO(n^(5/3))。移動コールバックの費用を掛ける。格納O(Q)、更新内容は利用側。

区間の左・右・更新時刻の3軸をブロック順に整列します。時間を進める更新と戻す更新を可逆にし、現在区間に入っている更新位置の寄与だけを差し替えます。

## 使用する問題・検証

[関連問題](https://www.spoj.com/problems/ADAUNIQ/)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
