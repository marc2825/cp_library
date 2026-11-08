# 3.1〜3.6 B項目の図解

[図解を開く](index.html) / [公開API・検証一覧](../../docs/b_1_6/README.md) / [一次資料](../../docs/b_1_6/references.md)。43掲載箇所に対する42モードです。Range Treeは3.3／3.6の同じ実体・同じモードを共有します。

入力はJSONです。アルゴリズムを選び、入力を反映してから「戻る」「次へ」「再生」で確認できます。各snapshotは説明・内部状態の表・配列または木の図を持ちます。入力を変更すると全履歴を作り直すため、処理途中の木に別の操作を適用することはありません。外部通信・CDN・npm導入は不要です。

## 表示する状態と簡略化

| モード | 表示する状態 | C++との違い |
|---|---|---|
| 整数根・Stern–Brocot・分数二分探索 | 二分探索境界、Euclidのrun、mediant境界 | 整数はBigInt。分数二分探索の教材predicateは入力p/q以下かの判定 |
| Fractional Cascading | 拡張catalog、元列／次列へのprefix対応、一つの補正 | 小さい配列を直接表示 |
| 平方分割 | ブロック範囲・和・点更新 | 教材のモノイドは整数和 |
| 非単調探索 | 優先queueの区間下界、暫定解、枝刈り | 既知の小配列からmin−slackを下界に作る。実APIでは利用者の下界oracle |
| RangeSet | 併合／分割後の半開区間 | JavaScript配列で区間列を持つ |
| Parallel／動的DSU | 長さ2^kごとのDSU、ラベル辞書、親・サイズ | 教材は小さい整数ID。経路圧縮による親配置の違いあり |
| 加算付きDSU | 成分tag、併合offset、点固有値、各点の現在値 | 教材では圧縮せず親経路のoffsetを合計 |
| Offline Dynamic Connectivity | 有効期間、時間軸Segtreeのbucket、入退場のDSU | snapshotで小配列をコピー。C++は変更履歴でrollback |
| Incremental Bridge | 橋・閉路辺・二重辺連結成分 | 教材は各辺を除いた成分探索で分類。C++の縮約森の物理配置・根変更は省略 |
| Sparse／Rollback Segtree | 作成された節点、節点値、履歴長、巻戻し | 疎版は小さい完全木に作成フラグを付けて描画 |
| Persistent Lazy Segtree | 不変root、節点の共有、lazy a,b、コピーした節点 | 教材は整数和とaffine、C++と同じcarry検索 |
| Sortable Segtree | runの分割・併合、key列、点のkey/payload更新、順／逆方向の集約 | run内の値域木をソート列で表示。整数和なので順逆の数値は一致 |
| Segtree Fractional Cascading | stable merge、左右prefixの写像、採用する節点 | 小さい配列を直接表示 |
| 2D Dual | 二軸の節点tag | 教材は可換な整数加算 |
| 重みWavelet Matrix／Wavelet Tree | partition、重みprefix、値域と子prefix、k選択 | 整数keyと整数重みだけ。C++は比較可能なkeyと可換加法群 |
| 永続Trie | 版root、path copy、各bitの子と個数 | 教材は1〜8bit、C++は全64bit |
| Priority Sum | 選択したk個と残り、各和 | 教材はソート配列。C++は二つのmultiset |
| Dynamic Bitset／Succinct Bit Vector | word内bit、word prefix、rank二分探索 | 図は8bit word、C++は64bit word。動的版の図は論理bit列 |
| Linear RMQ | 小ブロックのstack mask、block最小値、採用添字 | 教材は整数min。構築とqueryの構造はC++と共通 |
| Sqrt Tree | 各層のprefix／suffix／between、最高相違bit | 教材は整数和、C++は任意のモノイド |
| Splay／Red-Black／Weight-Balanced | 回転、色・黒高さ、部分木size、weight条件 | 小さい整数key。Splayの回転途中では祖先sizeの更新前状態を表示する場合あり |
| Lazy Balanced BST | 論理列の反転・作用・挿入と集約 | Treapの乱数優先度・節点配置は省略 |
| 永続平衡BST | 更新経路を複製したTreapと過去版 | 教材の乱数は32bit xorshift、C++は64bit SplitMix |
| Skew／Leftist Heap | merge後の左右の子、null-path rank、root | 比較は最大値優先。教材はGC所有、C++はlist所有とsplice |
| Radix Heap | bucket、last、pull時の再分配 | 教材は8bit非負key、C++は32/64bit |
| Erasable Queue | 値heap・削除予約・topの回収 | 教材のheapは順序付き小配列 |
| Range Tree | x節点ごとのy座標とBIT配列 | point IDと重複座標の契約は同じ |
| 矩形加算と和／点取得 | 四隅の差分係数、問い合わせ結果 | BITの物理配置を省略し、係数を直接走査。矩形和は整数セルの和 |
| Kd-Tree | 閉bounding box、矩形の枝刈り、最近点候補 | 小座標のNumber。C++の最近点はlong doubleで大座標の丸めを含む |
| Dynamic Rectangle Affine Sum | lazy affine、部分木sumとsize、新点の扱い | 教材は挿入時に全lazyを反映して全体を再構築。C++は経路伝播と不平衡部分だけ再構築 |
| Hilbert Mo／静的種類数 | Hilbert key順の区間移動、右端走査と最後の出現BIT | 小入力。C++のHilbert keyは31bit、教材は必要bit数だけ |

## 検証

```sh
node verify/visualizations/b_1_6_model_test.js
node verify/visualizations/b_1_6_browser_test.mjs --browser /path/to/chromium
```

モデル検証は全モードのsampleと、小配列・全候補・直接更新・set／sort・全点走査などの独立解を比較します。ブラウザでは全42モードの入力反映、前後・再生、エラー回復、説明リンク、390px表示を確認します。C++実装の検証は別の `python3 verify/run_b_1_6.py` です。図解の成功をC++のオンラインACとして扱いません。
