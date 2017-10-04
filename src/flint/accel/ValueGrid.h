
#pragma once
/*
#include <Eigen/Dense>

namespace accel {

template <int N, typename T>
class ValueGrid {
    public:
        ValueGrid(const Eigen::Array<unsigned int, N, 1> &dimension) : dimension(dimension) {
            pitch(0, 0) = 1;
            for (int i = 1; i < N; ++i) {
                pitch(i, 0) = pitch(i - 1, 0) * dimension(i - 1, 1);
            }
            values = decltype(values)::Zero(pitch(N - 1, 1) * dimension(N - 1, 0), 1);
        }

        ValueGrid(const Eigen::Array<unsigned int, N, 1> &dimension, T init) : dimension(dimension) {
            pitch(0, 0) = 1;
            for (int i = 1; i < N; ++i) {
                pitch(i, 0) = pitch(i - 1, 0) * dimension(i - 1, 0);
            }
            values = decltype(values)::Constant(pitch(N - 1, 0) * dimension(N - 1, 0), 1, init);
        }

        const T& operator[](unsigned int index) const {
            return values(index, 0);
        }

        T& operator[](unsigned int index) {
            return values(index, 0);
        }

        const T& operator[](const Eigen::Array<unsigned int, N, 1> &indices) const {
            unsigned int index = 0;
            for (int i = 0; i < N; ++i) {
                index += indices(i, 0) * pitch(i, 0);
            }
            return values(index, 0);
        }

        T& operator[](const Eigen::Array<unsigned int, N, 1> &indices) {
            unsigned int index = 0;
            for (int i = 0; i < N; ++i) {
                index += indices(i, 0) * pitch(i, 0);
            }
            return values(index, 0);
        }

        void Fill(T value) {
            values = decltype(values)::Constant(pitch(N - 1, 0) * dimension(N - 1, 0), 1, value);
        }

    private:
        Eigen::Array<unsigned int, N, 1> dimension;
        Eigen::Array<unsigned int, N, 1> pitch;
        Eigen::Array<T, Eigen::Dynamic, Eigen::Dynamic> values;
};

}*/
