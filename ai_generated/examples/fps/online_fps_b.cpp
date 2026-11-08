#include <iostream>
#include <vector>
#include "../../fps/online_fps.hpp"

int main() {
    cp::OnlineFPSExp<> series(4);
    series.append(0); series.append(1); series.append(0);
    std::cout << series.append(0) << "\n";
}
