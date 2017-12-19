
#pragma once
#include <functional>
#include <vector>

namespace core {
namespace VectorUtils {

template <typename Vector, typename Function>
void ApplyOverIndices(const Vector& vector, const Function &func) {
    for (unsigned int i = 0; i < vector.size(); ++i) {
        func(i);
    }
}

template <typename Vector, typename Function>
void ApplyOverElements(Vector& vector, const Function &func) {
    for (auto& el : vector) {
        func(el);
    }
}

}
}
