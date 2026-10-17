---
title: Karatsuba畳み込み
documentation_of: //convolution/karatsuba.hpp
---

# Karatsuba畳み込み

原始根を使わず、一般の環上で通常の多項式積を求めます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/transform_a/index.html#karatsuba) / [補完一覧・検証](../transform_a/README.md) / [一次資料](../transform_a/references.md)

## API・条件

`convolution_karatsuba<T>(const vector<T>& a, const vector<T>& b) -> vector<T>`。係数は定数項から昇冪順。空入力を含めば空、それ以外は長さa.size()+b.size()-1で末尾の0も保持します。入力は変更しません。

TはT(0)、コピー、+=、-=、積を持つ環です。除算・原始根・素数法を要求せず、合成数法や非可換の係数でも乗算順a[i]*b[j]を保ちます。boolは未対応。標準整数は途中の加減乗算も型内に収まる必要があり、超過はoverflow_error。最終値だけ収まっても途中で拒否する場合があります。独自型の演算条件と浮動小数点誤差は呼出側が管理します。長さ・padding容量超過はlength_error。

## Usage

```cpp
#include <iostream>
#include "convolution/karatsuba.hpp"
int main() {
    const auto c = cp::convolution_karatsuba(std::vector<long long>{1, 2, 3}, std::vector<long long>{4, 5});
    for (auto x : c) std::cout << x << ' ';
    std::cout << '\n';
}
```

出力:

```text
4 13 22 15
```

[コンパイルできる使用例](../../examples/convolution/karatsuba.cpp)。

## 原理

a=a0+x^h a1、b=b0+x^h b1に分け、z0=a0b0、z2=a1b1、z1=(a0+a1)(b0+b1)-z0-z2の3積を再帰計算します。結果はz0+x^h z1+x^(2h)z2。短い側が32以下なら直接積、それ以外は長い側を次の2冪に0補完します。

## 計算量

N=max(|a|,|b|)としてO(N^log2(3))時間・O(N)追加領域。短い側が32以下の直接積はO(|a||b|)。 型の加減乗除をO(1)として数えます。独自型ではその演算費用を掛けます。

## 使用する問題と検証

[Library Checker Convolution](https://judge.yosupo.jp/problem/convolution_mod)の通常の積。ジャッジ用は法付きの係数型を使います。

`python3 verify/run_transform_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した定義との比較・境界と例外を確認します。各項目の具体的な検証範囲は[補完一覧](../transform_a/README.md)へ。オンラインACは未確認です。
