#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
namespace cp {
// 正の悪化幅の平均採用率をtargetに合わせる。log温度上の二分探索。
inline long double calibrate_annealing_temperature(const std::vector<long double>&losses,long double target){
    if(losses.empty()||!std::isfinite(target)||target<=0||target>=1)throw std::invalid_argument("temperature calibration needs losses and target in (0,1)");
    for(auto loss:losses)if(!std::isfinite(loss)||loss<=0)throw std::invalid_argument("calibration losses must be positive and finite");
    const auto rate=[&](long double logarithm){const auto temperature=std::exp(logarithm);long double mean=0;for(std::size_t i=0;i<losses.size();++i){const auto probability=std::exp(-losses[i]/temperature);mean+=(probability-mean)/static_cast<long double>(i+1);}return mean;};
    long double low=std::log(std::numeric_limits<long double>::min()),high=std::log(std::numeric_limits<long double>::max())-1;
    if(rate(high)<target||rate(low)>target)throw std::overflow_error("calibrated temperature is outside the supported positive range");
    for(int i=0;i<160;++i){const auto mid=(low+high)/2;if(rate(mid)<target)low=mid;else high=mid;}
    return std::exp((low+high)/2);
}
class AcceptanceTemperatureController {
    long double temperature_,target_,gain_,minimum_,maximum_;
    std::size_t attempts_=0,accepted_=0;
public:
    AcceptanceTemperatureController(long double temperature,long double target=0.3L,long double gain=1,long double minimum=1e-12L,long double maximum=1e12L)
        :temperature_(temperature),target_(target),gain_(gain),minimum_(minimum),maximum_(maximum){
        if(!std::isfinite(temperature)||!std::isfinite(target)||!std::isfinite(gain)||!std::isfinite(minimum)||!std::isfinite(maximum)||temperature<=0||target<=0||target>=1||gain<0||minimum<=0||maximum<minimum||temperature<minimum||temperature>maximum)throw std::invalid_argument("invalid acceptance-temperature controller");
    }
    long double temperature()const{return temperature_;}
    std::size_t observations()const{return attempts_;}
    void observe(bool worsening,bool accepted){if(!worsening)return;if(attempts_==std::numeric_limits<std::size_t>::max())throw std::overflow_error("temperature observation count");++attempts_;if(accepted)++accepted_;}
    long double update(){if(!attempts_)return temperature_;const auto error=target_-static_cast<long double>(accepted_)/attempts_;const auto shift=gain_*error;const auto logt=std::clamp(std::log(temperature_)+shift,std::log(minimum_),std::log(maximum_));temperature_=std::clamp(std::exp(logt),minimum_,maximum_);attempts_=accepted_=0;return temperature_;}
};
} // namespace cp
