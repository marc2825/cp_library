#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>

namespace cp {
// 削除の代わりにsnapshot/rollbackを使うMo。resetは集計を空にする。
class RollbackMo {
    int n_; std::vector<std::pair<int,int>> queries_;
public:
    explicit RollbackMo(int n):n_(n) { assert(n>=0); }
    int add_query(int l,int r) { assert(0<=l && l<=r && r<=n_); queries_.emplace_back(l,r); return int(queries_.size())-1; }
    template<class Reset,class AddLeft,class AddRight,class Snapshot,class Rollback,class Answer>
    void run(Reset reset,AddLeft add_left,AddRight add_right,Snapshot snapshot,Rollback rollback,
             Answer answer,int block_size=0) const {
        assert(block_size>=0);
        int b=block_size?block_size:std::max(1,int(std::sqrt(std::max(1,n_))));
        std::vector<std::vector<int>> groups(n_/b+1);
        for(int id=0;id<int(queries_.size());++id) groups[queries_[id].first/b].push_back(id);
        for(int block=0;block<int(groups.size());++block) {
            auto& ids=groups[block]; if(ids.empty()) continue;
            int base=int(std::min(static_cast<long long>(n_),static_cast<long long>(block+1)*b));
            std::vector<int> long_queries;
            for(int id:ids) {
                auto [l,r]=queries_[id];
                if(r>base) { long_queries.push_back(id); continue; }
                reset(); for(int p=l;p<r;++p) add_right(p); answer(id);
            }
            reset(); int right=base;
            std::sort(long_queries.begin(),long_queries.end(),[&](int i,int j){return queries_[i].second<queries_[j].second;});
            for(int id:long_queries) {
                auto [l,r]=queries_[id]; while(right<r) add_right(right++);
                auto token=snapshot();
                for(int p=base;p>l;) add_left(--p);
                answer(id); rollback(token);
            }
        }
        reset();
    }
};
} // namespace cp
