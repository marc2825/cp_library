---
title: Nim Product
documentation_of: //game_theory/nim_product.hpp
---

# Nim Product

uint64_t全域でnim積を計算します。nim加算はXORで、この積と合わせて有限体をなします。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#nim_product) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`nim_product(uint64_t a, uint64_t b) -> uint64_t`。全64ビットを使え、オーバーフローする通常の整数積とは異なります。0との積は0、1は単位元。負の整数の暗黙変換を意図せず渡さず、符号なしのnim値を指定してください。独立なゲームの直和の計算は引き続きXORです。

初回は共有の読み取り専用256×256 byte表を構築します。C++のローカルstaticにより初期化はthread-safe。外部modint・__int128には依存しません。Nimberの除算・逆元・一般の体クラスは今回のA項目に含めていません。

## Usage

```cpp
#include <iostream>
#include "game_theory/nim_product.hpp"
int main() {
    std::cout << cp::nim_product(2, 3) << ' ' << cp::nim_product(16, 16) << '\n';
    std::cout << cp::nim_product(UINT64_C(1) << 32, UINT64_C(1) << 32) << '\n';
}
```

出力:

```text
1 24
6442450944
```

[コンパイルできる使用例](../../examples/game_theory/nim_product.cpp)。

## 原理

定義はa⊗b=mex{(a′⊗b) XOR (a⊗b′) XOR (a′⊗b′) | a′<a,b′<b}。小さい表はmexから構築し、8bit表へ拡張します。大きい数はa=a0+a1 Xの二次拡大へ分けます。X=2^(w/2)に対しX²=X XOR 2^(w/2−1)を使い、下位・上位・和の積から組み立てます。

## 計算量

wビット（wは2冪、最大64）としてO(w²)時間・O(log w)再帰領域。8bit以下は表参照O(1)。共有表は64KiB、初回構築は固定の約8bit規模の処理です。

## 使用する問題と検証

[Library Checker Nim Product (64 bit)](https://judge.yosupo.jp/problem/nim_product_64)。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
