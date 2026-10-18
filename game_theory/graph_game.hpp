#pragma once
#include <algorithm>
#include <climits>
#include <queue>
#include <stdexcept>
#include <vector>
namespace cp {
enum class GraphGameStatus {lose,win,draw};
struct GraphGameResult {std::vector<GraphGameStatus> status;std::vector<int> move,remoteness;};
// 終点で手番側が負け。閉路を許す有限・公平な通常ルールのretrograde解析。
// moveは勝ち手/引き分けを保つ手、敗勢は-1。drawのremotenessは-1。
inline GraphGameResult graph_game(const std::vector<std::vector<int>>& moves){
    if(moves.size()>static_cast<std::size_t>(INT_MAX))throw std::length_error("too many graph-game states");
    const int n=int(moves.size());std::vector<std::vector<int>> reverse(n);std::vector<std::size_t> remaining(n);
    GraphGameResult result{std::vector<GraphGameStatus>(n,GraphGameStatus::draw),std::vector<int>(n,-1),std::vector<int>(n,-1)};
    std::vector<int> largest(n);std::queue<int> queue;
    for(int u=0;u<n;++u){remaining[u]=moves[u].size();for(int v:moves[u]){if(v<0||v>=n)throw std::out_of_range("graph-game transition");reverse[v].push_back(u);}if(moves[u].empty()){result.status[u]=GraphGameStatus::lose;result.remoteness[u]=0;queue.push(u);}}
    while(!queue.empty()){
        const int v=queue.front();queue.pop();
        for(int u:reverse[v])if(result.status[u]==GraphGameStatus::draw){
            if(result.status[v]==GraphGameStatus::lose){result.status[u]=GraphGameStatus::win;result.move[u]=v;result.remoteness[u]=result.remoteness[v]+1;queue.push(u);}
            else{largest[u]=std::max(largest[u],result.remoteness[v]);if(!--remaining[u]){result.status[u]=GraphGameStatus::lose;result.remoteness[u]=largest[u]+1;queue.push(u);}}
        }
    }
    for(int u=0;u<n;++u)if(result.status[u]==GraphGameStatus::draw)for(int v:moves[u])if(result.status[v]==GraphGameStatus::draw){result.move[u]=v;break;}
    return result;
}
} // namespace cp
