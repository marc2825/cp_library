# S追加項目の一次資料・設計判断

2026-10-27確認。数学的な手順・入出力仕様を参照し、既存部品と独立した実装を組み合わせています。今回、外部実装のソースは新たに取り込んでいません。

| 対象 | 一次資料 | 採用した規約・範囲 |
|---|---|---|
| BWT | [Burrows–Wheeler原報告](https://www.mit.edu/~6.454/www_fall_2002/bwt/burrowswheeler94.pdf) | 終端を0、全byteを1〜256にする可逆変換。元文字列の行とLF巡回を検査 |
| FM-index | [Ferragina–Manzini原論文](https://people.unipmn.it/manzini/papers/focs00draft.pdf) | 後ろからの区間縮小と位置samplingを参照。今回は9段のpacked rankで、原論文のentropy依存の空間上界は主張しない |
| Semi-local LCS | [Tiskinの原著](https://arxiv.org/abs/0707.3619) | 一致・二重交差を解くstrand交換から終点順の開始labelを得る。4種類の境界queryを明示。Prefix-Substringはprefixごとのkernelを保存し、追加領域の予算を公開 |
| 両端回文木 | [Wang–Yang–Zhu原論文](https://arxiv.org/html/2210.02292v3) | surfaceと間接出現数を使用。direct linkのbyte辞書は永続二分木、過去nodeは再利用する。公開実装のコードは移入していない |
| 既存のStatic Range LIS | [既存の出典記録](../static_query/static_range_lis.md) | 既に取り込まれたCC0コードの分割統治は保持し、範囲計数を共通helperへ接続。変更後hashと変更内容を出典台帳へ更新 |
| ジャッジ仕様 | [Number of Substringsの公式仕様](https://github.com/yosupo06/library-checker-problems/blob/master/string/number_of_substrings/task.md) | 入力1文字列、相異なる非空部分文字列数を出力。ローカル比較と提出ACを区別 |

LZ77は自己参照を許す最長既出因子をSA/LCPと既出順位集合から求めます。SAM utilitiesは遷移DAGの経路数とsuffix link木のEuler区間から構成し、複数文書版と共有します。Online Zは「LCPがk以上」と「長さi+kのprefixのborderにkがある」の同値性を使う実装です。S-23はLCEによるperiod判定と区間長の素因数除去を組み合わせています。
