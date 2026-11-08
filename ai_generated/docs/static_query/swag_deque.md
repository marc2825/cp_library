---
title: SWAG Deque・両端操作と順序を保つ集約
documentation_of: //static_query/swag_deque.hpp
---

# SWAG Deque（D-01）

両端から要素を追加・削除しながら、先頭から末尾へのMonoid積を取得します。和・min/max・行列積・関数合成などに使え、逆演算・可換性・冪等性は不要です。

[実装](../../static_query/swag_deque.hpp) / [両端操作と再構築の図解](../../visualizations/static_query/index.html#swag_deque) / [カテゴリ](README.md)

## 使用例

```cpp
#include <iostream>
#include "static_query/swag_deque.hpp"
struct F { long long a, b; };
F op(F left, F right) { return {right.a * left.a, right.a * left.b + right.b}; }
F e() { return {1, 0}; }
int main() {
    cp::SWAGDeque<F, op, e> deque;
    deque.push_back({3, 4}); deque.push_front({2, 1});
    auto f = deque.prod();
    std::cout << f.a * 5 + f.b << '\n'; // (2*x+1), then (3*x+4).
    deque.pop_back();
    f = deque.prod(); std::cout << f.a * 5 + f.b << '\n';
    deque.push_front({0, 7}); // Assignment needs no inverse.
    f = deque.prod(); std::cout << f.a * 5 + f.b << '\n';
    deque.pop_front(); deque.pop_front();
    f = deque.prod(); std::cout << f.a * 5 + f.b << '\n';
}
```

出力:

```text
37
11
15
5
```

[実行できるソース](../../examples/static_query/swag_deque.cpp)。`op(left,right)`はleftを適用してからrightを適用する合成です。係数0の代入も扱えます。固定幅整数では計算途中の値も型の範囲内に収めてください。

## API・使用条件

`cp::SWAGDeque<S,op,e>`。C++17、単独includeと `static_query/all.hpp` に対応します。既存のFIFO版 [SWAG](swag.md) と同じ型・演算の指定方法です。

| API | 意味 | 計算量 |
|---|---|---|
| `SWAGDeque()` | 空で初期化 | O(1) |
| `push_front(value)` / `push_back(value)` | 先頭／末尾に追加 | ならしO(1)、最悪O(n) |
| `pop_front()` / `pop_back()` | 先頭／末尾の1要素を削除 | ならしO(1)、最悪O(n) |
| `front() const` / `back() const` | 端の値へのconst参照 | O(1) |
| `prod() const` | 左から右への全体積。空ならe() | O(1) |
| `size() const` / `empty() const` | intの要素数／空か | O(1) |
| `clear()` | 全要素を破棄 | O(n) |

Sはコピー構築可能な値型で、デフォルトコンストラクタは不要です。opは引数を書き換えず結合則を満たし、e()は左右の単位元を返す必要があります。演算やコピーをO(1)とした計算量で、文字列連結などでは時間・記憶量が増えます。

同時要素数はINT_MAX以下。空のpop/front/backは呼べず、前提をassertで検査します。NDEBUGではこの検査は消えます。返した参照は更新・clearをまたいで保持しないでください。`push_back(deque.front())`のような自己参照の追加は、引数を先にコピーするので可能です。

再構築は元のスタックを保持して新しい2本を作り、完成後に交換します。再構築途中で確保・コピー・opが例外を投げた場合、元の内容は残ります。Sのデストラクタが例外を投げないことを前提とします。

生きた要素と累積はO(n)、再構築時の追加領域もO(n)。vectorの容量はpush/pop/clearで再利用するため、確保済みメモリは過去の最大長に依存します。

## 原理とならし計算量

frontを逆順、backを順方向に並べたものが論理列です。frontの累積は `op(value,これまでの積)`、backでは `op(これまでの積,value)`。全体は `op(frontの積,backの積)` です。front/backの参照だけなら、他方の底の要素から端の値を得られるため再構築しません。

削除する側のスタックが空のときだけ、要素を半分ずつに分けて累積を作り直します。奇数長なら削除する側に1要素多く割り当てます。たとえばbackだけに `[a,b,c,d,e]` がある状態でpop_frontすると、frontの内部配列を `[c,b,a]`、backを `[d,e]` にした後、aを削除します。

全要素を反対側へ移す方式では、pop_front/pop_backを交互に行うと毎回ほぼ全体を移してしまいます。半分ずつにすると、ポテンシャル `|frontの長さ−backの長さ|` は通常更新1回につき高々1増え、n要素の再構築でnから高々1へ下がります。この減少でO(n)の再構築を支払え、vectorの拡張も含め、空からq回の両端更新の総計はO(q)です。

## 使用する問題・検証

[Library Checker: Deque Operate All Composite](https://judge.yosupo.jp/problem/deque_operate_all_composite) — [ジャッジ形式プログラム](../../verify/static_query/deque_operate_all_composite.test.cpp)。移動区間の両端を伸縮して関数を合成する問題、両端追加・削除のある列の最小値や行列積にも使えます。オンライン提出ACは未確認です。

```sh
make verify-static-query
python3 -m tools.cp_tools.suite --runners static_query
python3 -m tools.cp_tools.suite --stdlib-debug --runners static_query
python3 -m tools.cp_tools.suite --sanitize --runners static_query
node verify/visualizations/static_query_model_test.js
node verify/visualizations/static_query_browser_test.mjs --browser /path/to/chromium
```

[専用テスト](../../verify/static_query/swag_deque_test.cpp)はassertに依存しないチェックを使います。

- 2種類の値を両端に追加し、両端から削除する長さ8までの全履歴、1,092,253状態をstd::deque＋文字列の連結と比較。
- 240履歴・24万操作を剰余付きaffine関数の逐次適用と比較。非可逆な係数0、空・1要素・奇数長・clear・コピーの独立性・自己参照・非デフォルト構築型を確認。
- opとコピーに例外を注入し、再構築失敗後の元の内容を確認。
- 20万要素の片側／交互削除と513要素の窓の往復で、集約演算回数が総更新数の定数倍以下であることを検査。
- Pythonの素朴なdequeで8つのジャッジ入力を独立生成。使用例の文書との一致・出力、[実C++の13状態](../../verify/static_query/swag_deque_visualization.cpp)と図解の参照値も比較。

図解では整数の和と、整数を文字列に変換した連結を切り替えます。通算16追加まで、途中の再構築も前後移動でき、元の論理列と構築中のスタックを別々に表示します。C++の任意のMonoid・メモリアロケータを実行するものではありません。

## 参考資料

2026-10-20に [ebi-fly13のSWAG DequeのAPI・半分ずつ移す原理](https://ebi-fly13.github.io/Library/data_structure/deque_aggregation.hpp.html) と [Library Checkerの問題定義](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/data_structure/deque_operate_all_composite/task.md) を確認しました。本実装は既存SWAGの規約に合わせ、2つのvectorを一時構築する独自実装です。参考コードのコピー・改変による移植はしていません。出典の整理は[カテゴリの参考資料](references.md)を参照してください。
