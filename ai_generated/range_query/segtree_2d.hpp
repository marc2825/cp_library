#pragma once
#include <cassert>
#include <vector>

namespace cp {
// 密な二次元格子。可換モノイドが条件。空長方形はe()。
template<class S,auto op,auto e> class Segtree2D {
    int h_,w_,hbase_=1,wbase_=1; std::vector<std::vector<S>> data_;
    S row_prod(int x,int l,int r) const {
        S a=e(),b=e();
        for (l+=wbase_,r+=wbase_;l<r;l/=2,r/=2) {
            if (l&1) a=op(a,data_[x][l++]);
            if (r&1) b=op(data_[x][--r],b);
        }
        return op(a,b);
    }
public:
    Segtree2D(int h,int w):h_(h),w_(w) {
        assert(0<=h && h<=100000000 && 0<=w && w<=100000000);
        while (hbase_<h) hbase_*=2;
        while (wbase_<w) wbase_*=2;
        data_.assign(2*hbase_,std::vector<S>(2*wbase_,e()));
    }
    explicit Segtree2D(const std::vector<std::vector<S>>& a):Segtree2D(int(a.size()),a.empty()?0:int(a[0].size())) {
        for (int i=0;i<h_;++i) {
            assert(int(a[i].size())==w_);
            for (int j=0;j<w_;++j) data_[i+hbase_][j+wbase_]=a[i][j];
            for (int j=wbase_-1;j;--j) data_[i+hbase_][j]=op(data_[i+hbase_][j*2],data_[i+hbase_][j*2+1]);
        }
        for (int i=hbase_-1;i;--i) for (int j=1;j<2*wbase_;++j) data_[i][j]=op(data_[i*2][j],data_[i*2+1][j]);
    }
    void set(int x,int y,S v) {
        assert(0<=x && x<h_ && 0<=y && y<w_);
        int i=x+hbase_,j=y+wbase_; data_[i][j]=v;
        for (int k=j/2;k;k/=2) data_[i][k]=op(data_[i][k*2],data_[i][k*2+1]);
        for (i/=2;i;i/=2) {
            data_[i][j]=op(data_[i*2][j],data_[i*2+1][j]);
            for (int k=j/2;k;k/=2) data_[i][k]=op(data_[i][k*2],data_[i][k*2+1]);
        }
    }
    S prod(int left,int down,int right,int up) const {
        assert(0<=left && left<=right && right<=h_ && 0<=down && down<=up && up<=w_);
        S a=e(),b=e();
        for (left+=hbase_,right+=hbase_;left<right;left/=2,right/=2) {
            if (left&1) a=op(a,row_prod(left++,down,up));
            if (right&1) b=op(row_prod(--right,down,up),b);
        }
        return op(a,b);
    }
    S get(int x,int y) const { assert(0<=x && x<h_ && 0<=y && y<w_); return data_[x+hbase_][y+wbase_]; }
};
} // namespace cp
