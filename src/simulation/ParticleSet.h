
#pragma once

#include <tuple>
#include <vector>
#include "AttributeStorage.h"

namespace simulation {

template <typename Attribute, typename AttributeDefinitions, Attribute... Attributes>
class ParticleSet {
    using AttributeStorage = AttributeStorage<Attribute, Attributes...>;
    using Storage = std::tuple<std::vector<typename AttributeDefinitions::template AttributeInfo<Attributes>::type>...>;
    
    public:
        ParticleSet() : particleCount(0) { }

        ParticleSet(unsigned int particleCount) : particleCount(particleCount){
            AttributeStorage::ForEach(this->storage, [&](auto& attributeList, unsigned int index) {
                attributeList.resize(particleCount);
            });
        }

        void Resize(unsigned int particleCount) {
            this->particleCount = particleCount;
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

    private:
        unsigned int particleCount;
        Storage storage;

};

};
