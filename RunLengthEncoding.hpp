/*
★ Name        ：Run Length Encoding
 ■ Snipet      : Yes
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc019/submissions/44849943
                 https://atcoder.jp/contests/abc136/submissions/44850214
 ■ References  : https://nyaannyaan.github.io/library/string/run-length-encoding.hpp

 ■ TODO        :
*/


#pragma once
#include <vector>

using namespace std;

/// Run Length Encoding
// 型Cの配列の連長圧縮を(文字、数)の配列として返す
template <typename C>
vector<pair<typename C::value_type, int> > RunLengthEncoding(C& S) {
  using T = typename C::value_type;
  if (S.empty()) return {};
  
  vector<pair<T, int> > ret;
  T prev = S[0];
  int cnt = 1;
  for (int i = 1; i < (int)S.size(); i++) {
    if (S[i] == prev) cnt++;
    else {
      ret.emplace_back(prev, cnt);
      prev = S[i], cnt = 1;
    }
  }
  ret.emplace_back(prev, cnt);
  return ret;
}