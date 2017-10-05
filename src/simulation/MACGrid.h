
#pragma once

#include "flint/core/MultiGrid.h"

namespace simulation {

    namespace MAC {
        enum class Attribute {
            Grid,
            Interpolated,
        };
    }

    template <MAC::Attribute MACType, typename _type>
    struct GridAttributeInfo {
        static constexpr MAC::Attribute kMACType = MACType;
        using type = _type;
    };

template <unsigned int Dimension, typename Attribute, typename AttributeDefinitions, Attribute... GridAttributes>
class MACGrid {
    using Storage = std::tuple<core::MultiGrid<typename AttributeDefinitions::template AttributeInfo<GridAttributes>::info::type, Dimension>...>;
    Storage storage;

    static constexpr bool IsValidIndex(unsigned int i) {
        return i < sizeof...(GridAttributes);
    }

    template <unsigned int I>
    static constexpr Attribute IndexToAttribute() {
        static_assert(IsValidIndex(I), "Attribute not found on MAC grid");
        return std::get<I, Attribute, sizeof...(GridAttributes)>({ GridAttributes... });
    }

    template <Attribute A, unsigned int I = 0>
    struct AttributeToIndex {
        struct ValueHolder { static constexpr unsigned int value = I; };
        static constexpr unsigned int value = std::conditional_t<IsValidIndex(I), ValueHolder, AttributeToIndex<A, I + 1>>::value;
    };

    template <size_t... I, typename F>
    void ForEachGridImpl(std::index_sequence<I...>, F&& f) {
        int result[] = { 0, ((void)f(std::get<I>(this->storage)), 0)... };
    }

    template <typename F>
    void ForEachGrid(F&& f) {
        ForEachGridImpl(std::make_index_sequence<sizeof...(GridAttributes)>(), std::forward<F>(f));
    }

    public:
        MACGrid() { }

        template <typename T, typename... Dimensions>
        MACGrid(T cellSize, Dimensions... dimensions) {
            ForEachGrid([&](auto& grid) {
                using Grid = typename std::remove_reference<decltype(grid)>::type;
                typename Grid::Index index;
                std::array<T, Dimension> dims = { dimensions... };
                for (unsigned int i = 0; i < Dimension; ++i) {
                    index[i] = dims[i] / cellSize;
                }
                grid = Grid(index);
            });
        }

        template <Attribute A>
        constexpr decltype(auto) GetGrid() {
            return std::get<AttributeToIndex<A>::value>(storage);
        }
};

}
