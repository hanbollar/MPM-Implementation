
#pragma once

#include <vector>
#include "AttributeStorage.h"

namespace simulation {

template <typename _Type>
struct ParticleAttributeInfo {
    using Type = _Type;
};

template <typename Attribute, typename AttributeDefinitions, Attribute... Attributes>
class ParticleSet {
    using Storage = std::tuple<std::vector<typename AttributeDefinitions::template AttributeInfo<Attributes>::Info::Type>...>;
    using AttributeStorage = AttributeStorage<Attribute, Attributes...>;

    Storage storage;
    unsigned int particleCount;

    public:
        ParticleSet() : particleCount(0) { }

        ParticleSet(unsigned int particleCount) : particleCount(particleCount){
            AttributeStorage::ForEach(this->storage, [&](auto& attributeList, unsigned int index) {
                attributeList.resize(particleCount);
            });
        }

        template <Attribute A>
        decltype(auto) GetAttributeList() {
            return std::get<AttributeStorage::AttributeToIndex<A>::value>(storage);
        }

        template <Attribute A>
        decltype(auto) GetAttributeList() const {
            return std::get<AttributeStorage::AttributeToIndex<A>::value>(storage);
        }

        unsigned int Size() const {
            return particleCount;
        }
};

}