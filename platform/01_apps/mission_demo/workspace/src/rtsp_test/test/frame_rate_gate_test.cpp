#include "frame_rate_gate.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
void require(bool value, const char* message) {if (!value) throw std::runtime_error(message);}
int main() {
    using C = FrameRateGate::Clock;
    auto at=[](double seconds) {return C::time_point(std::chrono::duration_cast<C::duration>(std::chrono::duration<double>(seconds)));};
    FrameRateGate jitter(15); int count=0;
    for (int i=0;i<300;++i) count+=jitter.ready(at(i/15.0 + (i%2 ? .002 : 0)));
    require(count==300,"15Hz jitter causes sustained frame loss");
    FrameRateGate varying(15); count=0;
    for (int i=0;i<300;++i) count+=varying.ready(at(i/15.0 + (i%7)*.003));
    require(count==300,"bounded variable jitter drops 15Hz frames");
    FrameRateGate high(15);count=0;
    for (int i=0;i<600;++i) count+=high.ready(at(i/30.0));
    require(count>=299 && count<=301,"30Hz source not limited to 15Hz");
    FrameRateGate slow(15);count=0;
    for (int i=0;i<100;++i) count+=slow.ready(at(i/5.0));
    require(count==100,"slow source frames dropped");
    FrameRateGate stall(15);
    require(stall.ready(at(0)) && stall.ready(at(100)),"stalled stream fails to recover");
    for (int i=0;i<100;++i) require(!stall.ready(at(100)),"stall generated catch-up burst");
    require(stall.ready(at(100.07)),"post-stall cadence did not recover");
    FrameRateGate disabled(0);require(!disabled.ready(at(1)),"disabled color still publishes");
    for (double invalid : {-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected=false;try {FrameRateGate gate(invalid);} catch (const std::invalid_argument&) {rejected=true;}
        require(rejected,"invalid rate accepted");
    }
    std::cout<<"jitter, rate cap, slow source, stall, disabled and invalid input passed\n";
}
