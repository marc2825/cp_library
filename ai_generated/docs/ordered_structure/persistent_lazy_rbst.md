---
title: Fully Persistent Lazy RBST
documentation_of: //ordered_structure/persistent_lazy_rbst.hpp
---

# Fully Persistent Lazy RBST（D-32）

動的列の挿入・削除・反転・monoid作用を、経路コピーで完全永続化します。通常のオブジェクトコピーから過去のどの版でも更新を分岐できます。forward/backward積と遅延作用を不変ノードで共有し、サイズ比を用いたRBSTで結合します。D-23はこの実装の恒等作用版です。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#persistent_lazy_rbst)

## API・使用条件

`PersistentLazyRBST<S,op,e,F,mapping,composition,id>(values={},seed)`。D-23と同じinsert/erase/set/get/prod/reverse/move/split/concat/to_vectorに加え `apply(l,r,f)`。`auto old=tree; tree.apply(...)` でoldは不変です。

作用は全要素に一様で `mapping(f,op(a,b))=op(mapping(f,a),mapping(f,b))` を満たす必要があります。composition(f,g)はf after g、mapping(id,x)=x。位置依存作用は対象外。Sに長さを含めれば区間加算・affineと和を扱えます。反転と作用は可換であること。全体長はintに収まること。constなprodも遅延を展開した一時ノードを生成します。

## 計算量・保存領域

コピー O(1)、構築 O(n log n)、更新・区間積・split/concatは期待 O(log n)、all_prod O(1)、to_vector O(n)。要素演算を O(1) とします。各更新の新規ノードは期待 O(log n)。保持版の総変更経路分のメモリが必要で、最後の参照の破棄は解放数に比例します。

## 使用例

[コンパイル可能な使用例](../../examples/ordered_structure/persistent_lazy_rbst.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../ordered_structure/persistent_lazy_rbst.hpp"
struct S{long long sum;int length;};
S op(S a,S b){return {a.sum+b.sum,a.length+b.length};}
S e(){return {0,0};}
S mapping(long long f,S x){return {x.sum+f*x.length,x.length};}
long long composition(long long f,long long g){return f+g;}
long long id(){return 0;}
int main(){
    cp::PersistentLazyRBST<S,op,e,long long,mapping,composition,id> tree({{1,1},{2,1},{3,1}});
    auto old=tree; tree.apply(0,2,10); tree.reverse(0,3);
    std::cout<<old.all_prod().sum<<' '<<tree.all_prod().sum<<' '<<tree.prod(0,2).sum<<'\n';
}
```

```text
6 26 15
```

## 使用する問題

Persistent Dynamic Sequence Range Affine Range Sum、過去版から分岐する列の編集。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Driscoll et al., Making Data Structures Persistent](https://doi.org/10.1016/0022-0000(89)90034-2) の経路コピーと、[Randomized Binary Search Trees](https://doi.org/10.1145/274787.274812) のサイズ比joinを組み合わせます。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
