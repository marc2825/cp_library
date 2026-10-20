---
title: 部分永続配列と最初の成立時刻の探索
documentation_of: //basic_utility/partial_persistent_search.hpp
---

# 部分永続配列と最初の成立時刻の探索（U-20）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#partial_persistent_search)

各添字に変更時刻と値の履歴を追加し、参照時刻以下の最後の値を二分探索で得ます。その上に版番号を探索する独立の関数を置くことで、配列だけでなく既存の部分永続DSUにも同じ時間検索を適用できます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/partial_persistent_search.hpp"
int main() {
    cp::PartiallyPersistentArray<int> values(2, 0);
    values.set(0, 2); values.set(1, 5); values.set(0, 7);
    auto time = cp::first_true_version(0, values.time() + 1,
        [&](std::int64_t t) { return values.get(t, 0) >= 4; });
    std::cout << *time << ' ' << values.get(1, 0) << '\n';
    cp::PartiallyPersistentDisjointSetUnion dsu(3);
    dsu.merge(0, 1); dsu.merge(1, 2);
    std::cout << *cp::first_connected_version(dsu, 0, 2) << '\n';
}
```

出力:

```text
3 2
2
```

## API・使用条件

U-20の具体的な対象を **最新版だけを更新する点代入配列と、過去版上の単調述語が初めてtrueになる時刻の検索** とします。既存の部分永続DSU・完全永続木・Rollbackを再実装する項目ではありません。

`PartiallyPersistentArray<T>(initial)` または `(size,value=T())` は固定長、初期版0。`set(index,value)` は最新版を更新して新時刻を返し、同値代入でも1進みます。`get(time,index)` は任意の既存版の値をコピーして返します。`size()` / `time()` で長さと現在時刻を取得。過去版からの分岐更新はできません。不正時刻・添字は `out_of_range`。時刻はint64で最大INT64_MAX−1、超過更新は `overflow_error`。Tはコピー可能な値型です。

`first_true_version(first,last,predicate)` は非負int64の半開版区間 `[first,last)` を二分探索し、最初のtrueの版をoptionalで返します。全false／空はnullopt。範囲逆転・負のfirstは `invalid_argument`。predicateは区間内でfalse→trueに単調、参照中の履歴は固定が前提です。関数は版の実在や単調性を検査しません。配列が任意の値に更新できることと、ある条件が時間に関して単調であることは別です。

`first_connected_version(dsu,u,v)` は既存のPartiallyPersistentDisjointSetUnionを参照し、初期版を含めて初めて連結した時刻を返します。未連結はnullopt、同一頂点は0、頂点範囲外は例外。DSUの「失敗したmergeでも時刻が進む」規約を保持します。

## 計算量

配列構築O(n)、更新はならしO(1)、getはその添字への更新数u_iに対しO(log(u_i+1))、記憶O(n+U)。Tのコピー費用は別です。版検索はV版にO(log(V+1))回のpredicate評価。DSU接続時刻はO(log(V+1) log(n+1))、配列の1点閾値時刻ならO(log(V+1) log(u_i+1))。

## 使用する問題・検証

単調に増えるスコアが閾値に達した最初の更新、辺追加履歴で2頂点が初めてつながる時刻。全版の配列コピー、各時刻の連結成分ラベルと比較し、初期版・未成立・空区間・最大時刻近辺も検証します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
