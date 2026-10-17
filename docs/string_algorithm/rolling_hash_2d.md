---
title: 2D Rolling Hash
documentation_of: //string_algorithm/rolling_hash_2d.hpp
---

# 2D Rolling Hash

長方形のbyteグリッドを前計算し、部分長方形のハッシュを定数時間で取得します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#hash_2d) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`RollingHash2D(vector<string> grid={}, uint32_t row_base=911382323, uint32_t column_base=972663749)`。法は既存RollingHashと同じ1000000007/1000000009。baseは両方[257,1000000007)。高さ0なら幅0、幅0の行が複数あっても高さを保持します。全行同じ長さが必要です。

`height()` / `width()`、`row_base()` / `column_base()`、`get(top,bottom,left,right) -> Hash`。Hashはarray<uint32_t,2>。引数は行区間[top,bottom)、列区間[left,right)の順で、空の長方形は{0,0}。

`equal(other, top,bottom,left,right, other_top,other_bottom,other_left,other_right)`はbase・形・ハッシュを比較します。raw Hash自体はbaseや形を持ちません。異なるbaseはinvalid_argument、形状/baseの違反もinvalid_argument、範囲外はout_of_range。各次元はINT_MAX以下、prefix容量超過はlength_error。入力不変。

ハッシュ衝突があり、等値は厳密な証明ではありません。固定baseに対する敵対入力では衝突確率を保証しません。row_baseとcolumn_baseを同じ値にしても呼出可能ですが、方向を区別する能力が弱くなる場合があります。

## Usage

```cpp
#include <iostream>
#include "string_algorithm/rolling_hash_2d.hpp"
int main() {
    cp::RollingHash2D a({"abcd", "efgh", "ijkl"}), b({"fg", "jk"});
    std::cout << a.equal(b, 1, 3, 1, 3, 0, 2, 0, 2) << '\n';
    const auto empty = a.get(1, 1, 0, 4);
    std::cout << empty[0] << ' ' << empty[1] << '\n';
}
```

出力:

```text
1
0 0
```

[コンパイルできる使用例](../../examples/string_algorithm/rolling_hash_2d.cpp)。

## 原理

各セルをbyte+1にし、行方向と列方向の二変数多項式を保持します。prefix矩形を作り、上・左をそれぞれのbaseの冪でずらして引き、二重に引いた左上を足します。抽出後の指数は矩形の右下を基準に揃います。

## 計算量

h×wの構築O(hw+h+w)時間・領域、get/equal O(1)。空グリッドも扱います。

## 使用する問題と検証

[AOJ ALDS1_14_C Pattern Search](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_14_C)。ジャッジ形式は一致候補を実際のbyteで確認して出力し、ハッシュ衝突を正解判定へ混入させません。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
