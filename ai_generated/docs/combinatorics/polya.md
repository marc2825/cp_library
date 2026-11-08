---
title: Pólya Enumeration / Cycle Index
documentation_of: //combinatorics/polya.hpp
---

# Pólya Enumeration / Cycle Index

群の全置換からcycle indexを構築し、色数や重みを代入して軌道を数えます。

[実装](../../combinatorics/polya.hpp) / [操作できる図解](../../visualizations/math_a/index.html#polya) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`CycleIndex(group)`のgroupはvector<vector<int>>で、各行は0..N-1の置換。`degree()`、`group_order()`、`terms()`（cycle個数ベクトル→重複数）を取得できます。`colorings(colors,mod)`は等価な彩色の個数。`evaluate(weights,mod)`は長さiのcycleへweights[i]を代入します。weightsは長さN+1、0番は未使用。

群は空でなく全要素を列挙すること。置換の長さ・範囲・全単射性を検査しますが、群の閉性と列挙の正しさは検査しません。群作用が非忠実なら同じ置換になる群要素も含めます。colors>=0、mod>0で群の位数がmodに対し可逆であること。非可逆はdomain_error、不正入力はinvalid_argument。N=0の空配置は1、colors=0かつN>0は0。

色cの重みをw_cとするとweights[i]=sum_c w_c^iを渡します。これは重み付き軌道の和で、各色の使用個数を指定する多変数係数抽出APIは含みません。

## 原理と計算量

各群要素を互いに素なcycleへ分解します。同じcycle上は同じ色でなければ固定されません。cycle長iの寄与をweights[i]とする積をcycle型ごとに重複数倍し、Burnsideの平均を共有します。

G個のN次置換から構築O(G*N log(G+1))を上界とし、T種類のcycle型で保持領域O(T*N+N)。評価はO(T*N)回の走査とmod累乗、各指数はN以下。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "combinatorics/polya.hpp"

int main() {
    cp::CycleIndex group({{0,1,2},{1,2,0},{2,0,1}});
    std::cout << group.colorings(2) << " " << group.evaluate({0,3,5,9}) << "\n";
}
```

出力:

```text
4 15
```

[動く使用例](../../examples/combinatorics/polya.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

使用問題: 回転/反転する輪の彩色と重み付きnecklace。図解で置換ごとのcycleと寄与を追えます。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [CMU K. Sutner 群作用・Pólyaの講義](https://www.cs.cmu.edu/~cdm/resources.html)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
