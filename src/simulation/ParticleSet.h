
#pragma once

#include <tuple>
#include <vector>
#include "AttributeStorage.h"

namespace simulation {

template <typename Attribute, typename AttributeDefinitions, Attribute... Attributes>
class ParticleSet {
    using AttributeStorage_ = AttributeStorage<Attribute, Attributes...>;
    using Storage = std::tuple<std::vector<typename AttributeDefinitions::template AttributeInfo<Attributes>::type>...>;
    
    public:
        ParticleSet() : particleCount(0) { }

        ParticleSet(unsigned int particleCount) : particleCount(particleCount){
            AttributeStorage_::ForEach(this->storage, [&](auto& attributeList, unsigned int index) {
                attributeList.resize(particleCount);
            });
        }

        void Resize(unsigned int particleCount) {
            this->particleCount = particleCount;
            AttributeStorage_::ForEach(this->storage, [&](auto& attributeList, unsigned int index) {
                attributeList.resize(particleCount);
            });

            int result[] = {0, ((void)std::fill(GetAttributeList<Attributes>().begin(), GetAttributeList<Attributes>().end(), AttributeDefinitions::template AttributeInfo<Attributes>::Default()), 0)... };
        }

        void Append(const ParticleSet& other) {
            int result[] = { 0, ((void)GetAttributeList<Attributes>().insert(GetAttributeList<Attributes>().end(), other.GetAttributeList<Attributes>().begin(), other.GetAttributeList<Attributes>().end()), 0)... };
            particleCount += other.Size();
        }

        template <Attribute A>
        decltype(auto) Get() {
            return std::get<AttributeStorage_::template AttributeToIndex<A>::value>(storage);
        }

        template <Attribute A>
        decltype(auto) Get() const {
            return std::get<AttributeStorage_::template AttributeToIndex<A>::value>(storage);
        }

        template <Attribute A>
        decltype(auto) GetAttributeList() {
            return std::get<AttributeStorage_::template AttributeToIndex<A>::value>(storage);
        }

        template <Attribute A>
        decltype(auto) GetAttributeList() const {
            return std::get<AttributeStorage_::template AttributeToIndex<A>::value>(storage);
        }

        unsigned int Size() const {
            return particleCount;
        }

    private:
        unsigned int particleCount;
        Storage storage;

};

};
