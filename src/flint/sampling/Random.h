
#pragma once 

#include <random>
#include <chrono>

namespace sampling {

class RandomGenerator : public std::mt19937_64 {
    public:
        RandomGenerator() : std::mt19937_64() {
            // initialize the random number generator with time-dependent seed
            uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
            std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
            this->seed(ss);
        }

        RandomGenerator(std::seed_seq& q) : std::mt19937_64(q) { }

        RandomGenerator(std::mt19937_64::result_type val) : std::mt19937_64(val) { }
};

}