#pragma once
// 説明: docs/range_query/dual_segtree_2d.md
#include <cassert>
#include <vector>

namespace cp {
// 可換モノイドFの矩形作用を累積。getはその点へ作用した全Fの積。
template<class F,auto composition,auto id> class DualSegtree2D {
    int h_,w_,x_=1,y_=1; std::vector<F> tags_;
    F& at(int x,int y) { return tags_[std::size_t(x)*2*y_+y]; }
    const F& at(int x,int y) const { return tags_[std::size_t(x)*2*y_+y]; }
    void row_apply(int x,int l,int r,F f) {
        for(l+=y_,r+=y_;l<r;l/=2,r/=2) {
            if(l&1) { auto& t=at(x,l++); t=composition(f,t); }
            if(r&1) { auto& t=at(x,--r); t=composition(f,t); }
        }
    }
public:
    DualSegtree2D(int height,int width) : h_(height),w_(width) {
        assert(height>=0 && width>=0);
        while(x_<h_) x_*=2;
        while(y_<w_) y_*=2;
        tags_.assign(std::size_t(2*x_)*2*y_,id());
    }
    void apply(int top,int bottom,int left,int right,F f) {
        assert(0<=top && top<=bottom && bottom<=h_ && 0<=left && left<=right && right<=w_);
        for(top+=x_,bottom+=x_;top<bottom;top/=2,bottom/=2) {
            if(top&1) row_apply(top++,left,right,f);
            if(bottom&1) row_apply(--bottom,left,right,f);
        }
    }
    F get(int row,int column) const {
        assert(0<=row && row<h_ && 0<=column && column<w_); F result=id();
        for(int x=row+x_;x;x/=2) for(int y=column+y_;y;y/=2) result=composition(at(x,y),result);
        return result;
    }
};
} // namespace cp
