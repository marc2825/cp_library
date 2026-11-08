---
title: Burnsideの補題
documentation_of: //combinatorics/burnside.hpp
---

# Burnsideの補題

群作用で同一視される対象の数を、固定点数の平均から求めます。

[実装](../../combinatorics/burnside.hpp) / [操作できる図解](../../visualizations/math_a/index.html#burnside) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`burnside_orbits(group_order,fixed,mod=998244353)`は群要素i=0..G-1のfixed(i)を平均します。fixedは非負のlong longの固定点数または剰余。`burnside_average(sum,G,mod)`は計算済みの総和を平均する入口です。

`burnside_orbits_exact(vector<uint64_t> fixed_counts)`は整数で平均し、mod逆元を使いません。

G>0・mod>0。mod版はgcd(G,mod)=1が必要、非可逆ならdomain_error。mod=1は0。exact版は総和がuint64内、割り切れなければdomain_error、総和超過はoverflow_error。群作用と固定点数が正しいことは呼出側の責任で、可除性だけで群作用の妥当性を証明するものではありません。

## 原理と計算量

対象とそれを固定する群要素の組を二通りに数えると、各軌道は群の位数だけ寄与します。固定点総和を群の位数で割ると軌道数です。

G回の固定点評価＋O(G)加算＋逆元とmod乗算。exact版O(G)時間/O(1)追加領域。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "combinatorics/burnside.hpp"

int main() {
    std::cout << cp::burnside_orbits_exact({8,2,2}) << "\n";
}
```

出力:

```text
4
```

[動く使用例](../../examples/combinatorics/burnside.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

使用問題: 3頂点の輪を2色で塗り、回転を同一視する彩色。一般化は[Pólya](polya.md)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [CMU K. Sutner 群作用・Pólyaの講義](https://www.cs.cmu.edu/~cdm/resources.html)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
