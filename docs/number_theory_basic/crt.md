---
title: CRT / 一般化中国剰余定理
documentation_of: //number_theory_basic/crt.hpp
---

# CRT / 一般化中国剰余定理

互いに素でない正の法も含む連立合同式を解きます。

[実装](../../number_theory_basic/crt.hpp) / [操作できる図解](../../visualizations/math_a/index.html#crt) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`crt(residues,moduli)` → optional<CRTResult>、フィールド`remainder,modulus`。解は`x ≡ remainder (mod modulus)`、0<=remainder<modulus、modulusは法のlcm。空入力は{0,1}、矛盾はnullopt。

入力長一致・全法が正であること。不正入力はinvalid_argument。各prefixのlcmがLLONG_MAXを超えればoverflow_error（後続の矛盾判定より先に発生する場合があります）。負の剰余、法1、重複法に対応。固定3法の畳み込み内部CRTを変更しません。

## 原理と計算量

現在の解r mod mへb mod nを併合します。g=gcd(m,n)がb-rを割らなければ矛盾。割れば、(m/g)のmod n/g逆元で補正係数を求めます。結果は新しいlcm未満なので、lcm検査後のr+m*tは安全です。

K個の法に対しO(K log M)回程度の整数演算。安全なmod乗算は積が64bitを超えるとO(log M)加算、領域O(1)追加。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "number_theory_basic/crt.hpp"

int main() {
    auto result=cp::crt({2,8},{6,9});
    if (result) std::cout << result->remainder << " " << result->modulus << "\n";
}
```

出力:

```text
8 18
```

[動く使用例](../../examples/number_theory_basic/crt.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [AtCoder ABC193 E - Oversleeping](https://atcoder.jp/contests/abc193/tasks/abc193_e)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [ACL公式CRT仕様](https://atcoder.github.io/ac-library/production/document_ja/math.html)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
