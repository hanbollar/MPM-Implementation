
#pragma once

#include <tuple>
#include "flint/core/MultiGrid.h"
#include "AttributeStorage.h"

namespace simulation {

namespace MAC {
    enum class Attribute {
        Grid,
        Interpolated,
    };
}

template <MAC::Attribute _MACType, typename _Type>
struct GridAttributeInfo {
    static constexpr MAC::Attribute MACType = _MACType;
    using Type = _Type;
};

template <unsigned int Dimension, typename Attribute, typename AttributeDefinitions, Attribute... Attributes>
class MACGrid {
    using Storage = std::tuple<typename std::conditional_t<
        AttributeDefinitions::template AttributeInfo<Attributes>::Info::MACType == MAC::Attribute::Interpolated,
        std::array<core::MultiGrid<typename AttributeDefinitions::template AttributeInfo<Attributes>::Info::Type, Dimension>, Dimension>,
        core::MultiGrid<typename AttributeDefinitions::template AttributeInfo<Attributes>::Info::Type, Dimension>
    >...>;
    
    using AttributeStorage = AttributeStorage<Attribute, Attributes...>;

    Storage storage;
    
    public:
        MACGrid() { }

        template <typename T, typename... Dimensions>
        MACGrid(T cellSize, Dimensions... dimensions) {
            AttributeStorage::ForEach(this->storage, [&](auto& grid, unsigned int index) {
                /*using Grid = typename std::remove_reference<decltype(grid)>::type;
                typename Grid::Index index;
                std::array<T, Dimension> dims = { dimensions... };
                for (unsigned int i = 0; i < Dimension; ++i) {
                    index[i] = dims[i] / cellSize;
                }
                grid = Grid(index);*/
            });
        }

        template <Attribute A>
        decltype(auto) GetGrid() {
            return std::get<AttributeStorage::AttributeToIndex<A>::value>(storage);
        }

        template <Attribute A>
        decltype(auto) GetGrid() const {
            return std::get<AttributeStorage::AttributeToIndex<A>::value>(storage);
        }
};

}
