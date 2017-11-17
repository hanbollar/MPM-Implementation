
#pragma once
#include <functional>
#include <vector>

namespace core {
namespace VectorUtils {

template <typename Vector>
void ApplyOverIndices(const Vector& vector, const std::function<void(unsigned int)> &func) {
    for (unsigned int i = 0; i < vector.size(); ++i) {
        func(i);
    }
}

template <typename Vector>
void ApplyOverElements(Vector& vector, const std::function<void(typename Vector::value_type&)> &func) {
    for (auto& el : vector) {
        func(el);
    }
}

}
}