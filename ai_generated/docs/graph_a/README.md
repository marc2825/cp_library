# 3.7〜3.11 A項目の補完

未完了15掲載箇所を補完しました。Block-Cut Treeは既存の`block_cut_forest`を共有し、Functional Graphは3.7/3.8で共用します。3.10は既存実装を保持し、回帰検証済みです。

- [3.7 最短路・探索](../graph_basic/README.md)
- [3.8 構造分解](../graph_connectivity/README.md)
- [3.9 フロー・最適化](../graph_flow/README.md)
- [3.10 組合せグラフ](../graph_combinatorics/README.md)
- [3.11 木](../tree/README.md)
- [14モードの操作できる図解](../../visualizations/graph_a/index.html) / [一次資料・問題例](references.md)

```sh
python3 verify/run_graph_a.py
python3 verify/run_graph_a.py --sanitize
node verify/visualizations/graph_a_model_test.js
node verify/visualizations/graph_a_browser_test.mjs --browser /path/to/chromium
```

16公開ヘッダ（共通結果型・共有入口2本を含む）の単独/重複include（debug/release）、逆順/全26カテゴリ混在、複数翻訳単位、16使用例とMarkdown usageの一致、6ジャッジ形式×24ケースを確認します。差分検証5本をdebug/releaseで実行し、`--sanitize`はASan/UBSanを有効にします。

- 最短路: 700非負多重グラフでFloydとの全始点比較、500potential付きグラフ/DAGをBellman緩和と比較。全ての復元辺、補グラフの明示BFS、負閉路、64bit境界、20万頂点。
- K-shortest: 650多重グラフの全単純経路と、別実装のuniform-cost walk列挙。頂点/辺の復元・重複排除、0閉路・自己ループ・多重辺、20万頂点と2000個のwalk。
- 構造: n<=5の全Functional Graph、1024種類の2変数SAT式、750多重グラフのEuler道を辺順列の全探索と比較。Block-Cutは全頂点部分集合の頂点連結性で独立に確認。20万変数/頂点。
- フロー: 700個の小さな下限制約網で全流量割当を列挙。負のs-t流量・循環・復元、700個のProject Selectionで全真偽値割当、submodular費用・必須条件・整数極値・20万プロジェクト。
- 木ハッシュ: n<=6の全1442ラベル付き木を全根で括弧列の正規形と比較。頂点名/辺順変更、森/不正入力、20万頂点。ハッシュ一致を同型性の数学的保証とは扱いません。

全てローカル検証です。オンラインACは未確認。Yen法は頂点重複なし、Eppstein法は繰り返しを許すwalkです。木ハッシュは衝突し得ます。各APIの計算量と算術条件は個別説明を参照してください。

今回の確認結果：通常・ASan/UBSan/LeakSanitizer、全26カテゴリの混在/複数翻訳単位、既存5カテゴリ＋graph/treeの回帰が成功。図解モデルは2,319入力・20,682状態、ブラウザは全14モード・390px表示が成功。関連索引を含むローカルリンク1,093件も確認済みです。
