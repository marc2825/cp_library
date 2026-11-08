---
title: 過去の操作を編集する時刻列（U-21）
documentation_of: //basic_utility/retroactive_operations.hpp
---

# 過去の操作を編集する時刻列（U-21）

[ユーティリティ索引](README.md) / [Uの全14項目](../utility_search/README.md) / [操作できる図解](../../visualizations/utility_search/index.html#retroactive_operations)

時刻順の操作をMonoidとして合成し、過去への追加・置換・削除を任意時点の結果に反映します。例えば、時刻10の「2倍して1加える」と時刻30の「3倍する」の間に「4加える」を挿入すると、初期値1の現在値は9から21へ変わります。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/retroactive_operations.hpp"
#include "range_query/monoid_catalog.hpp"
int main() {
    cp::RetroactiveOperations<cp::AffineFunctionMonoid<long long>> history({10, 20, 30});
    history.set(10, {2, 1}); history.set(30, {3, 0});
    std::cout << history.all()(1) << '\n';
    history.set(20, {1, 4}); // Insert into the past: (2*x+1), (x+4), (3*x).
    std::cout << history.through(20)(1) << ' ' << history.all()(1) << '\n';
    history.erase(10);
    std::cout << history.all()(1) << '\n';
}
```

出力:

```text
9
7 21
15
```

## API・使用条件

U-21の対象を **事前登録したint64時刻上で、結合則を満たす操作の列を編集し、任意時刻・区間の合成を得る機能** と定めました。最新版だけを更新し過去版を残す[U-20](partial_persistent_search.md)とは、過去の操作自体を変更する点が異なります。任意のデータ構造を自動的にretroactive化する機構ではありません。[部分Retroactive Priority Queue](../../ordered_structure/partially_retroactive_priority_queue.hpp)は別の機能です。

`RetroactiveOperations<M>` のMは `using S`、`static S op(S,S)`（const参照引数も可）、`static S e()` を持つMonoidです。`op(a,b)` は **aを実行してからb**、eは何もしない操作。可換性や逆操作は不要です。既存の `AffineFunctionMonoid<T>` が利用でき、a=0による代入も含められます。固定幅型では演算の中間値も型に収まることが必要です。

| API | 意味 |
|---|---|
| `(vector<int64_t> times)` | 時刻を整列・重複除去して登録。初期状態では操作なし |
| `set(time,operation)` | その時刻の操作を追加／置換。同じ時刻に複数操作は置かない |
| `erase(time)` | 削除できればtrue、登録済みだが空ならfalse |
| `get(time)` | 操作のコピーをoptionalで取得。登録済みで空ならnullopt |
| `contains(time)` | 操作が存在するか。未登録時刻ならfalse |
| `before(time)` | time未満の操作を合成 |
| `through(time)` | time以下の操作を合成。INT64_MAXでも加算しない |
| `fold(first,last)` | 時刻区間 `[first,last)` の操作を合成 |
| `all()` | 全操作の合成 |
| `capacity()` / `size()` / `empty()` | 重複除去後の時刻数／存在する操作数／空か |

`set/erase/get`の未登録時刻は `out_of_range`。検索の境界時刻は未登録でも指定できます。逆転区間は `invalid_argument`。登録時刻数が1億を超える場合は既存Segtreeの上限に合わせて `length_error`。空の登録列・空区間ではeを返します。eと同値の操作をsetしても「存在する1操作」と数えます。入力エラーは更新前に検出します。Sのコピー・代入やMonoid演算が更新途中で例外を投げた場合、整合性の保証はありません。通常のコピーは履歴を独立に複製します。

## 原理・計算量

既存[Segtree](../range_query/segtree.md)の葉に各時刻の操作、空の葉にeを置きます。内部節点は左→右の合成で、過去を編集すると祖先だけを再計算します。

入力時刻数n、重複除去後mに対し、構築O(n log(n+1)+m)、記憶O(n+m)。`set/erase/before/through/fold`はO(log(m+1))、`get/contains`は時刻検索を含めO(log(m+1))、all/size/capacity/emptyはO(1)。Sのサイズとコピー・op/eをO(1)とした計算量です。長い文字列の連結をMonoidにするとこの前提は成立しません。

## 使用する問題・検証

過去の取引の訂正と任意時刻の残高、時刻付きaffine変換・代入の追加削除、文字列の順序を保つ操作合成など。図解ではaffine変換の編集直後に各時刻の値がどう変わるかを観察します。

[比較テスト](../../verify/utility_search/retroactive_enumeration_test.cpp)では、非可換な文字列連結をstd::mapの全再生と比較し、3000回の追加・置換・削除、空操作の存在、任意境界、INT64_MIN/MAX、非可逆な代入、コピーの独立性、20万時刻を検証します。libstdc++ debug modeではlower_boundの前提検査が線形になるため2000時刻に縮小し、同じ全時刻更新・削除・結果照合を行います。`python3 verify/run_utility_search.py`。オンライン提出ACは未確認です。

## 参考資料

[Demaine・Iacono・Langerman, Retroactive Data Structures](https://erikdemaine.org/papers/Retroactive_SODA2004/) は過去の操作列への編集と、任意時点への問合せという区別を定義しています。本APIは合成可能な操作と固定時刻集合に対象を限定した独自ラッパです。[出典と既存部品](../utility_search/references.md)。
