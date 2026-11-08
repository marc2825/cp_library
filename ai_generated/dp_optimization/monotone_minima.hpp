#pragma once
#include <stdexcept>
#include <vector>

namespace cp {
// 行の最左argminが単調非減少である行列。get(row,col)、値の<を使用。
// O(rows+cols log(rows+1))回の評価。0列の各行は-1。
template<class Get> std::vector<int> monotone_minima(int rows,int cols,Get get) {
    if(rows<0 || cols<0) throw std::invalid_argument("negative matrix dimension");
    std::vector<int> answer(rows,-1); if(!rows || !cols) return answer;
    auto solve=[&](auto&& self,int top,int bottom,int left,int right)->void {
        if(top>=bottom) return;
        int row=top+(bottom-top)/2,best=left;
        for(int col=left+1;col<right;++col) if(get(row,col)<get(row,best)) best=col;
        answer[row]=best; self(self,top,row,left,best+1); self(self,row+1,bottom,best,right);
    };
    solve(solve,0,rows,0,cols); return answer;
}
} // namespace cp
