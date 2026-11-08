#pragma once
#include <numeric>
#include <stdexcept>
#include <vector>

namespace cp {
// 全単調行列の最左行argmin。Mongeなら適用可。O(rows+cols)回の評価。
template<class Get> std::vector<int> smawk(int rows,int cols,Get get) {
    if(rows<0 || cols<0) throw std::invalid_argument("negative matrix dimension");
    std::vector<int> answer(rows,-1),rs(rows),cs(cols); if(!rows || !cols) return answer;
    std::iota(rs.begin(),rs.end(),0); std::iota(cs.begin(),cs.end(),0);
    auto solve=[&](auto&& self,const std::vector<int>& r,const std::vector<int>& c)->void {
        if(r.empty()) return;
        std::vector<int> reduced;
        for(int col:c) {
            while(!reduced.empty() && get(r[reduced.size()-1],col)<get(r[reduced.size()-1],reduced.back())) reduced.pop_back();
            if(reduced.size()<r.size()) reduced.push_back(col);
        }
        std::vector<int> odd; for(std::size_t i=1;i<r.size();i+=2) odd.push_back(r[i]); self(self,odd,reduced);
        std::size_t start=0;
        for(std::size_t i=0;i<r.size();i+=2) {
            std::size_t end=reduced.size()-1;
            if(i+1<r.size()) { end=start; while(reduced[end]!=answer[r[i+1]]) ++end; }
            int best=reduced[start];
            for(std::size_t j=start+1;j<=end;++j) if(get(r[i],reduced[j])<get(r[i],best)) best=reduced[j];
            answer[r[i]]=best; start=end;
        }
    };
    solve(solve,rs,cs); return answer;
}
} // namespace cp
