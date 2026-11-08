(function(root){
const meta={
  "fractional_cascading": {
    "title": "Fractional Cascading",
    "summary": "複数のソート済み列へ同じ値を検索します。次の拡張列の一つおきの要素を取り込み、各位置から元の列・次の列への境界を結びます。",
    "path": "basic_utility/fractional_cascading"
  },
  "integer_kth_root": {
    "title": "整数k乗根",
    "summary": "浮動小数点を使わず、k乗がn以下になる最大の整数を二分探索します。乗算の前にn/xと比較するため、uint64の最大値を扱えます。",
    "path": "basic_utility/integer_kth_root"
  },
  "stern_brocot_tree": {
    "title": "Stern–Brocot Tree",
    "summary": "正の既約分数と、根1/1からの左右の経路を相互変換します。連続するL/Rを回数でまとめ、Euclidの除算で一気に進めます。",
    "path": "basic_utility/stern_brocot_tree"
  },
  "fraction_binary_search": {
    "title": "分数二分探索",
    "summary": "Stern–Brocotの隣接分数を維持し、同じ方向への連続移動を指数探索と二分探索でまとめます。",
    "path": "basic_utility/fraction_binary_search"
  },
  "sqrt_decomposition": {
    "title": "平方分割テンプレート",
    "summary": "列を約√N個のブロックに分け、各ブロックにモノイドの積を保持します。境界だけ直接走査し、内部はブロックの積で飛ばします。",
    "path": "basic_utility/sqrt_decomposition"
  },
  "nonmonotone_search": {
    "title": "非単調関数の探索",
    "summary": "有限整数区間を分割し、区間の下界が暫定解を改善しない場合に枝刈りする分枝限定法です。単峰性も単調性も要求しません。",
    "path": "basic_utility/nonmonotone_search"
  },
  "range_set": {
    "title": "連続区間集合の管理",
    "summary": "3.6のRangeSetを共有し、半開区間の和集合を動的に管理します。接している区間も統合します。",
    "path": "basic_utility/range_set"
  },
  "offline_dynamic_connectivity": {
    "title": "Offline Dynamic Connectivity（共通入口）",
    "summary": "3.6の実装を共有します。辺の有効期間を時間軸のセグメント木へ置き、Rollback DSUで各時刻の状態を復元します。",
    "path": "union_find/offline_dynamic_connectivity"
  },
  "parallel_disjoint_set_union": {
    "title": "Parallel Union-Find",
    "summary": "同じ長さの二つの区間を、対応するoffset同士でまとめて併合します。長さ2^kごとのDSUで既に処理した区間対を飛ばし、初めての併合だけを下位へ伝えます。",
    "path": "union_find/parallel_disjoint_set_union"
  },
  "dynamic_disjoint_set_union": {
    "title": "動的Union-Find",
    "summary": "頂点のラベルが事前に分からない場合に、検索・併合時に頂点を追加します。ラベル辞書と配列DSUを組み合わせます。",
    "path": "union_find/dynamic_disjoint_set_union"
  },
  "additive_disjoint_set_union": {
    "title": "加算操作付きDSU",
    "summary": "成分の加算tagと、併合前の加算履歴を打ち消すoffsetを保持します。点固有の加算は別に管理し、代表頂点への加算が他の頂点へ波及しないようにします。",
    "path": "union_find/additive_disjoint_set_union"
  },
  "incremental_bridge_connectivity": {
    "title": "Incremental Bridge Connectivity",
    "summary": "橋を辺とする縮約森を維持します。異なる木を結ぶ辺は橋を一つ増やし、同じ木に閉路を作る辺は経路上の橋を縮約します。",
    "path": "union_find/incremental_bridge_connectivity"
  },
  "sparse_dynamic_segtree": {
    "title": "Sparse Dynamic Segtree",
    "summary": "既存DynamicSegtreeの疎な節点確保を共通入口から利用します。更新した根から葉への経路だけを作り、未作成区間は単位元として扱います。",
    "path": "range_query/sparse_dynamic_segtree"
  },
  "rollback_segtree": {
    "title": "Rollback Segtree",
    "summary": "点代入で変更する全節点の古い値をstackへ保存し、過去の状態へ巻き戻します。検索は履歴を増やしません。",
    "path": "range_query/rollback_segtree"
  },
  "persistent_lazy_segtree": {
    "title": "Persistent Lazy Segtree",
    "summary": "更新時に通る節点だけを複製し、任意の過去rootから区間作用の新しい版を作ります。検索は祖先のtagをcarryとして渡し、節点を変更しません。",
    "path": "range_query/persistent_lazy_segtree"
  },
  "sortable_segtree": {
    "title": "Sortable Segtree",
    "summary": "ソート済みrunをkey値域の疎な木で表し、区間境界でrank分割、sort時に破壊的meldをします。runの順／逆方向の積を持ち、外側のSegtreeで列順に集約します。",
    "path": "range_query/sortable_segtree"
  },
  "range_tree": {
    "title": "Range Tree",
    "summary": "x順のセグメント木の各節点に、y順の座標とFenwick Treeを置きます。更新点の座標を固定し、各点の重みをオンラインで変更します。",
    "path": "range_query/range_tree"
  },
  "segtree_fractional_cascading": {
    "title": "Segtree with Fractional Cascading",
    "summary": "各節点にソート列を持つMerge Sort Treeで、親prefixから左右の子prefixへの対応を保持します。根だけで二分探索し、以降の検索を定数時間で子へ伝えます。",
    "path": "range_query/segtree_fractional_cascading"
  },
  "dual_segtree_2d": {
    "title": "2D Dual Segtree",
    "summary": "矩形をx・yそれぞれのセグメント木区間へ分割し、積にtagを保持します。点取得では二軸の祖先をたどり、その点に作用した全tagを集約します。",
    "path": "range_query/dual_segtree_2d"
  },
  "weighted_wavelet_matrix": {
    "title": "Wavelet Matrixの区間和拡張",
    "summary": "keyのbitごとにstable partitionを行い、並べ替えた重みの累積和も保持します。値域内の重み和と、keyが小さいk点の重み和を求めます。",
    "path": "static_query/weighted_wavelet_matrix"
  },
  "wavelet_tree": {
    "title": "Wavelet Tree",
    "summary": "圧縮した値域を二分する木です。各節点に左の子へ進む要素数のprefixを持ち、位置区間を子の区間へ写します。",
    "path": "static_query/wavelet_tree"
  },
  "persistent_binary_trie": {
    "title": "Persistent Binary Trie",
    "summary": "uint64の各bitで分岐するTrieの根から葉までをコピーし、任意の版へ挿入・削除します。XOR後の順序統計も元の版を変えずに検索します。",
    "path": "static_query/persistent_binary_trie"
  },
  "priority_sum": {
    "title": "Priority Sum（上位・下位k個の和）",
    "summary": "比較器順の先頭k個と残りを二つのmultisetに分け、選ばれた側の和を保持します。kの変更や重複値の一個削除に対応します。",
    "path": "static_query/priority_sum"
  },
  "dynamic_bitset": {
    "title": "Dynamic Bitset",
    "summary": "実行時長のbit列をuint64の配列に格納し、word単位の集合演算とshiftを行います。最後のwordの未使用bitを常に0に保ちます。",
    "path": "static_query/dynamic_bitset"
  },
  "succinct_bit_vector": {
    "title": "Succinct Bit Vector",
    "summary": "bit本体を64bit wordに詰め、各word直前までの1の数を保持します。rankから二分探索してselectを得ます。",
    "path": "static_query/succinct_bit_vector"
  },
  "linear_rmq": {
    "title": "Linear RMQ",
    "summary": "log N/2程度の小ブロック内の単調stackをbitmaskにし、端のRMQをbit操作で答えます。ブロック最小値だけのSparse Tableは全体で線形サイズです。",
    "path": "static_query/linear_rmq"
  },
  "sqrt_tree": {
    "title": "Sqrt Tree",
    "summary": "平方分割をlog log N層に再帰し、各層でprefix・suffixと完全ブロック間の積を前計算します。左右端の最高相違bitから使う層を定数時間で選びます。",
    "path": "static_query/sqrt_tree"
  },
  "splay_tree": {
    "title": "Splay Tree",
    "summary": "検索した節点を回転で根へ持ち上げる自己調整BSTです。重複数と部分木サイズを持ち、順位検索に対応します。",
    "path": "ordered_structure/splay_tree"
  },
  "red_black_tree": {
    "title": "Red-Black Tree",
    "summary": "左傾の赤黒木です。回転・色反転で赤い右辺、連続する赤辺、黒高さの不一致を防ぎ、重複を一節点にまとめます。",
    "path": "ordered_structure/red_black_tree"
  },
  "weight_balanced_tree": {
    "title": "Weight-Balanced Tree",
    "summary": "各子の節点数が部分木全体の3/4以下になる条件を維持します。条件を破る部分木はinorder列から中央分割で再構築します。",
    "path": "ordered_structure/weight_balanced_tree"
  },
  "lazy_balanced_bst": {
    "title": "Lazy Balanced BST（共通入口）",
    "summary": "既存のLazyReversibleTreapを共有します。暗黙の添字をkeyとして挿入・削除・反転・区間作用・非可換な集約をまとめます。",
    "path": "ordered_structure/lazy_balanced_bst"
  },
  "persistent_balanced_bst": {
    "title": "永続平衡BST",
    "summary": "不変なTreap節点をshared_ptrで共有し、更新経路だけをコピーします。任意のオブジェクトを残したまま新しい版を作れます。",
    "path": "ordered_structure/persistent_balanced_bst"
  },
  "skew_heap": {
    "title": "Skew Heap",
    "summary": "右の経路を併合してから子を交換する併合可能heapです。常に左右を交換し、償却解析で短い総処理を保証します。",
    "path": "ordered_structure/skew_heap"
  },
  "leftist_heap": {
    "title": "Leftist Heap",
    "summary": "右の経路を併合してから子を交換する併合可能heapです。null-path lengthが左>=右となるよう交換し、右の経路を対数長に保ちます。",
    "path": "ordered_structure/leftist_heap"
  },
  "radix_heap": {
    "title": "Radix Heap（共通入口）",
    "summary": "既存iwiwi実装を共有します。最後に取り出したkeyとの最高相違bitでbucketを分け、単調に取り出す整数keyを扱います。",
    "path": "ordered_structure/radix_heap"
  },
  "erasable_priority_queue": {
    "title": "Erasable Priority Queue",
    "summary": "挿入heapと削除予約heapで遅延削除します。個数辞書で存在を確認し、不在の値を消して後続の同値挿入へ影響させることを防ぎます。",
    "path": "ordered_structure/erasable_priority_queue"
  },
  "rectangle_add_rectangle_sum": {
    "title": "長方形加算・長方形和",
    "summary": "矩形の四隅に差分を置きます。各差分の係数(w,-wx,-wy,wxy)を疎な2D BITで集計し、原点からの積分をwXY-wxY-wyX+wxyとして計算します。",
    "path": "offline_query/rectangle_add_rectangle_sum"
  },
  "rectangle_add_point_get": {
    "title": "長方形加算・点取得",
    "summary": "四隅の差分を座圧2D BITに追加し、点以下の差分和で値を得ます。upper_boundを使うので座標に1を足す必要がありません。",
    "path": "offline_query/rectangle_add_point_get"
  },
  "kd_tree": {
    "title": "Kd-Tree",
    "summary": "x/yを交互に中央値分割した静的Kd木です。各部分木の閉じたbounding boxで検索を枝刈りし、矩形内の点IDと最近点を求めます。",
    "path": "offline_query/kd_tree"
  },
  "hilbert_order_mo": {
    "title": "Hilbert Order Mo",
    "summary": "問い合わせの左右端を平面の点と見なし、Hilbert曲線の順に処理します。近い区間を続けて処理し、現在区間の移動量を抑えます。",
    "path": "offline_query/hilbert_order_mo"
  },
  "static_range_count_distinct": {
    "title": "Static Range Count Distinct",
    "summary": "問い合わせを右端順に並べ、各値の最後の出現位置だけをBITへ1として残します。区間和はその区間に現れる値の種類数になります。",
    "path": "offline_query/static_range_count_distinct"
  },
  "dynamic_rectangle_affine_sum": {
    "title": "Dynamic Rectangle Affine Sum",
    "summary": "lazy作用付きのKd木へオンラインで点を追加します。部分木のbounding boxが矩形に含まれるときは、和と要素数から一括してw←aw+bを適用します。",
    "path": "offline_query/dynamic_rectangle_affine_sum"
  }
};
if(typeof module!=="undefined")module.exports=meta;else root.BMetadata=meta;
})(typeof window!=="undefined"?window:globalThis);
