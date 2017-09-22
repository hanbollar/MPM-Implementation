
#pragma once

#include <Eigen/Dense>
#include "Optional.h"

namespace core {

template <int N, typename T>
class AxisAlignedBox {

    Eigen::Array<T, N, 2> _data;

    public:

        AxisAlignedBox() = delete;
        AxisAlignedBox(const Eigen::Array<T, N, 1> &min, const Eigen::Array<T, N, 1> &max) {
            this->min() = min;
            this->max() = max;
        }

        typename Eigen::Array<T, N, 2>::ConstColXpr min() const {
            return _data.col(0);
        }

        typename Eigen::Array<T, N, 2>::ConstColXpr max() const {
            return _data.col(1);
        }

        typename Eigen::Array<T, N, 2>::ColXpr min() {
            return _data.col(0);
        }

        typename Eigen::Array<T, N, 2>::ColXpr max() {
            return _data.col(1);
        }

        T& min(unsigned int i) {
            return _data(i, 0);
        }

        T& max(unsigned int i) {
            return _data(i, 1);
        }

        T min(unsigned int i) const {
            return _data(i, 0);
        }

        T max(unsigned int i) const {
            return _data(i, 1);
        }

        T Extent(unsigned int i) const {
            return max(i) - min(i);
        }

        unsigned int GreatestExtent() const {
            unsigned int i = 0;
            for (unsigned int d = 0; d < N; ++d) {
                if (Extent(d) > Extent(i)) {
                    i = d;
                }
            }
            return i;
        }

        T SurfaceArea() const {
            T sa = 0;
            for (unsigned int d1 = 0; d1 < N; ++d1) {
                for (unsigned int d2 = 0; d2 < N; ++d2) {
                    if (d1 != d2) {
                        sa += Extent(d1) + Extent(d2);
                    }
                }
            }
            return sa;
        }

        AxisAlignedBox& Merge(const AxisAlignedBox &other) {
            auto lower = (other.min() < min()).template cast<float>();
            auto upper = (other.max() > max()).template cast<float>();

            min() = (lower * other.min() + (1 - lower) * min());
            max() = (upper * other.max() + (1 - upper) * max());

            return *this;
        }

        AxisAlignedBox& Merge(const Eigen::Array<T, N, 1> &other) {
            auto lower = (other < min()).template cast<float>();
            auto upper = (other > max()).template cast<float>();

            min() = (lower * other + (1 - lower) * min());
            max() = (upper * other + (1 - upper) * max());

            return *this;
        }

};

template <int N, typename T>
AxisAlignedBox<N, T>& Merge(AxisAlignedBox<N, T> &a, const AxisAlignedBox<N, T> &b) {
    return a.Merge(b);
}

template <int N, typename T>
AxisAlignedBox<N, T>& Merge(AxisAlignedBox<N, T> &a, Optional<const AxisAlignedBox<N, T>> &b) {
    if (b.hasValue()) {
        a.Merge(b.value());
    }
    return a;
}

template <int N, typename T>
AxisAlignedBox<N, T>& Merge(AxisAlignedBox<N, T> &a, Optional<AxisAlignedBox<N, T>> &&b) {
    if (b.hasValue()) {
        a.Merge(b.value());
    }
    return a;
}

template <int N, typename T>
AxisAlignedBox<N, T>& Merge(AxisAlignedBox<N, T> &a, const Eigen::Matrix<T, N, 1> &b) {
    a.Merge(b);
    return a;
}

template <int N, typename T>
AxisAlignedBox<N, T>& Merge(Optional<AxisAlignedBox<N, T>> &a, const AxisAlignedBox<N, T> &b) {
    if (a.hasValue()) {
        a.value().Merge(b);
    } else {
        a.set(b);
    }
    return a.value();
}

template <int N, typename T>
AxisAlignedBox<N, T>& Merge(Optional<AxisAlignedBox<N, T>> &a, const Eigen::Matrix<T, N, 1> &b) {
    if (a.hasValue()) {
        a.value().Merge(b);
    } else {
        a.set(AxisAlignedBox<N, T> {b, b});
    }
    return a.value();
}

template <int N, typename T>
AxisAlignedBox<N, T>& Merge(Optional<AxisAlignedBox<N, T>> &&a, const Eigen::Matrix<T, N, 1> &b) {
    if (a.hasValue()) {
        a.value().Merge(b);
    } else {
        a.set(AxisAlignedBox<N, T> {b, b});
    }
    return a.value();
}

template <int N, typename T>
Optional<AxisAlignedBox<N, T>>& Merge(Optional<AxisAlignedBox<N, T>> &a, Optional<AxisAlignedBox<N, T>> &b) {
    if (a.hasValue()) {
        if (b.hasValue()) {
            a.value().Merge(b.value());
        }
    } else {
        if (b.hasValue()) {
            a.set(b.value());
        }
    }
    return a;
}

template <int N, typename T>
Optional<AxisAlignedBox<N, T>>& Merge(Optional<AxisAlignedBox<N, T>> &a, Optional<const AxisAlignedBox<N, T>> &b) {
    if (a.hasValue()) {
        if (b.hasValue()) {
            a.value().Merge(b.value());
        }
    } else {
        if (b.hasValue()) {
            a.set(b.value());
        }
    }
    return a;
}

template <int N, typename T>
Optional<AxisAlignedBox<N, T>>& Merge(Optional<AxisAlignedBox<N, T>> &a, const Optional<AxisAlignedBox<N, T>> &b) {
    if (a.hasValue()) {
        if (b.hasValue()) {
            a.value().Merge(b.value());
        }
    } else {
        if (b.hasValue()) {
            a.set(b.value());
        }
    }
    return a;
}

template <int N, typename T>
Optional<AxisAlignedBox<N, T>>& Merge(Optional<AxisAlignedBox<N, T>> &a, const Optional<const AxisAlignedBox<N, T>> &b) {
    if (a.hasValue()) {
        if (b.hasValue()) {
            a.value().Merge(b.value());
        }
    } else {
        if (b.hasValue()) {
            a.set(b.value());
        }
    }
    return a;
}

template <int N, typename T>
Optional<AxisAlignedBox<N, T>>& Merge(Optional<AxisAlignedBox<N, T>> &a, Optional<Eigen::Matrix<T, N, 1>> &b) {
    if (a.hasValue()) {
        if (b.hasValue()) {
            a.value().Merge(b);
        }
    } else {
        if (b.hasValue()) {
            a.set(AxisAlignedBox<N, T> {b, b});
        }
    }
    return a;
}

template <int N, typename T>
Optional<AxisAlignedBox<N, T>>& Merge(Optional<AxisAlignedBox<N, T>> &a, Optional<const Eigen::Matrix<T, N, 1>> &b) {
    if (a.hasValue()) {
        if (b.hasValue()) {
            a.value().Merge(b);
        }
    } else {
        if (b.hasValue()) {
            a.set(AxisAlignedBox<N, T> {b, b});
        }
    }
    return a;
}

}
