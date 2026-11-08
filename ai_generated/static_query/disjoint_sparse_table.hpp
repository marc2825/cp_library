#pragma once
#include <algorithm>
#include <cassert>
#include <vector>

namespace cp {
// 結合則のみで可。順序を保つ区間積、空区間はe()。
template<class S,auto op,auto e> class DisjointSparseTable {
    std::vector<S> a_; std::vector<std::vector<S>> table_;
public:
    DisjointSparseTable()=default;
    explicit DisjointSparseTable(const std::vector<S>& a):a_(a) {
        int n=int(a.size());
        for (long long half=1;half<n;half*=2) {
            table_.push_back(a); auto& row=table_.back();
            for (long long l=0;l<n;l+=2*half) {
                int mid=int(std::min(l+half,static_cast<long long>(n)));
                int r=int(std::min(l+2*half,static_cast<long long>(n)));
                for (int i=mid-2;i>=l;--i) row[i]=op(a[i],row[i+1]);
                for (int i=mid+1;i<r;++i) row[i]=op(row[i-1],a[i]);
            }
        }
    }
    int size() const { return int(a_.size()); }
    S prod(int l,int r) const {
        assert(0<=l && l<=r && r<=size());
        if (l==r) return e();
        if (l+1==r) return a_[l];
        unsigned x=unsigned(l^(r-1)); int b=0;
        while (x>>=1) ++b;
        return op(table_[b][l],table_[b][r-1]);
    }
};
} // namespace cp
