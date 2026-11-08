---
title: Aliens Trick / WQS Binary Search
documentation_of: //dp_optimization/aliens_trick.hpp
---

# Aliens Trick / WQS Binary Search

個数への罰則付き最適化から、ちょうどK個を使う離散凸DPの値を求めます。

[実装](../../dp_optimization/aliens_trick.hpp) / [操作できる図解](../../visualizations/math_a/index.html#aliens_trick) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`aliens_trick(k,low,high,oracle)` → optional<AliensTrickResult>。oracle(penalty)は`PenalizedDPResult {value,count}`として`min_j(F(j)+penalty*j)`とその個数を返します。結果の`value`はF(k)、`penalty`は採用した罰則、`relaxed`はoracleの結果です。

Fは実行可能な個数の連続区間上で離散凸、Fと罰則は整数です。罰則の同値解は必ず**個数が多い方**を返し、countは非負。探索端点はcount(low)>=K>=count(high)を満たす必要があり、満たさなければnullopt。凸性・oracleの最適性は検査しません。一般の非凸DPではこの値をexact-K解として使えません。

不正なK/範囲/負countはinvalid_argument。`penalty*K`と補正値を含むlong long計算の超過はoverflow_error。oracle内部の計算も呼出側の責任です。

## 原理と計算量

罰則を増やすと最適個数は非増加です。count>=Kとなる最後の整数罰則で、支持直線の値からpenalty*Kを引きます。離散凸性と整数性により、同値でKを飛び越えても正しいF(K)を得られます。

罰則幅WについてO(log(W+1)+1)回のoracle呼出し、追加領域O(1)。signed 64bit全域をまたぐ二分探索にも対応。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "dp_optimization/aliens_trick.hpp"

int main() {
    auto oracle=[](long long p) {
        cp::PenalizedDPResult best{1000000,0};
        for (int k=0;k<=6;++k) {
            long long value=(k-3)*(k-3)+p*k;
            if (value<=best.value) best={value,k};
        }
        return best;
    };
    auto result=cp::aliens_trick(2,-20,20,oracle);
    std::cout << result->value << "\n";
}
```

出力:

```text
1
```

[動く使用例](../../examples/dp_optimization/aliens_trick.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [IOI 2016 Aliens（公式タスク保管庫）](https://ioinformatics.org/page/ioi-2016/42)。図解は非負列の分割DPを罰則で最適化します。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [OI Wiki WQSの式・同値処理](https://oi-wiki.org/dp/opt/wqs-binary-search/)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
