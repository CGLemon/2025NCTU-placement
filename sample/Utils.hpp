#pragma once
#include <random>
#include <chrono>

inline uint64_t nowUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(
            high_resolution_clock::now().time_since_epoch()).count();
}
static std::mt19937 rng((uint32_t)time(nullptr));
inline int randint(int l,int r){ std::uniform_int_distribution<int> d(l,r); return d(rng); }
inline double rand01(){ std::uniform_real_distribution<double>d(0,1); return d(rng); }
