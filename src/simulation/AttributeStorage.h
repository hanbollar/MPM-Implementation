#pragma once

namespace simulation {

// Compile-time helpers to help iterate over and map particle attributes
template <typename Attribute, Attribute... Attributes>
class AttributeStorage {
    public:
        static constexpr bool IsValidIndex(unsigned int i) {
            return i < sizeof...(Attributes);
        }

        template <unsigned int I>
        static constexpr Attribute IndexToAttribute() {
            static_assert(IsValidIndex(I), "Attribute not found on MAC grid");
            return std::get<I, Attribute, sizeof...(Attributes)>({ Attributes... });
        }

        template <Attribute A, unsigned int I = 0>
        struct AttributeToIndex {
            struct ValueHolder { static constexpr unsigned int value = I; };
            static constexpr unsigned int value = std::conditional_t<IndexToAttribute<I>() == A, ValueHolder, AttributeToIndex<A, I + 1>>::value;
        };

        template <typename Storage, typename F>
        static void ForEach(Storage&& storage, F&& f) {
            ForEachImpl(std::make_index_sequence<sizeof...(Attributes)>(), std::forward<Storage>(storage), std::forward<F>(f));
        }

    private:
        template <typename Storage, size_t... I, typename F>
        static void ForEachImpl(std::index_sequence<I...>, Storage&& storage, F&& f) {
            int result[] = { 0, ((void)f(std::get<I>(storage), I), 0)... };
        }
};

}
