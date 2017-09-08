
#pragma once

#include <functional>

namespace core {
namespace iterator {

    namespace polymorphic {

        template <typename T>
        class IteratorBase {
            public:
                virtual IteratorBase& operator++() = 0;
                virtual ~IteratorBase() {}
                virtual T operator*() const = 0;
                virtual bool operator==(const IteratorBase& other) const = 0;
                virtual bool operator!=(const IteratorBase& other) const = 0;
        };

        template <typename T>
        class Iterator {
            public:
                Iterator(IteratorBase<T>* impl) : impl(impl) { }
                Iterator(const Iterator& other) = delete;
                Iterator& operator=(const Iterator&) = delete;
                Iterator(Iterator&& other) {
                    impl = other.impl;
                    other.impl = nullptr;
                }
                Iterator& operator=(Iterator&& other) {
                    impl = other.impl;
                    other.impl = nullptr;
                    return *this;
                }
                ~Iterator() { delete impl; }

                Iterator& operator++() {
                    impl->operator++();
                    return *this;
                }

                T operator*() const {
                    return impl->operator*();
                }

                bool operator==(const Iterator& other) const {
                    return impl->operator==(*other.impl);
                }

                bool operator!=(const Iterator& other) const {
                    return impl->operator!=(*other.impl);
                }

            private:
                IteratorBase<T>* impl = nullptr;
        };

        template <typename T>
        class RangeIteratorBase {
            public:
                virtual ~RangeIteratorBase() {}
                virtual Iterator<T> begin() const = 0;
                virtual Iterator<T> end() const = 0;
        };

        template <typename T>
        class RangeIterator {
            public:
                RangeIterator(RangeIteratorBase<T>* impl) : impl(impl) { }
                RangeIterator(const RangeIterator& other) = delete;
                RangeIterator& operator=(const RangeIterator&) = delete;
                RangeIterator(RangeIterator&& other) {
                    impl = other.impl;
                    other.impl = nullptr;
                }
                RangeIterator& operator=(RangeIterator&& other) {
                    impl = other.impl;
                    other.impl = nullptr;
                    return *this;
                }
                ~RangeIterator() { delete impl; }

                Iterator<T> begin() const {
                    return impl->begin();
                }

                Iterator<T> end() const {
                    return impl->end();
                }

            private:
                RangeIteratorBase<T>* impl = nullptr;
        };

        template <typename Base, typename T, unsigned int N>
        class FixedArrayIteratorImpl : public RangeIteratorBase<Base> {

            const std::array<T, N> &array;
            using filter_arg_t = typename std::conditional<std::is_pointer<T>::value, T, typename std::add_lvalue_reference<T>::type>::type;
            using filter_func_t = bool(*)(filter_arg_t);
            const filter_func_t filter;
            using iterator_t = decltype(std::declval<const std::array<T, N>>().begin());

            public:
                FixedArrayIteratorImpl(const std::array<T, N> &array, const filter_func_t filter = nullptr) : array(array), filter(filter) { }

                class IteratorImpl : public IteratorBase<Base> {

                    iterator_t iterator;
                    iterator_t end;
                    filter_func_t filter;

                    public:
                        IteratorImpl(iterator_t iterator, iterator_t end, filter_func_t filter = nullptr)
                            : iterator(iterator), end(end), filter(filter) {
                        }

                        IteratorBase<Base>& operator++() {
                            iterator = filter ? std::find_if(iterator + 1, end, filter) : iterator + 1;
                            return *this;
                        }

                        bool operator==(const IteratorBase<Base>& other) const {
                            return iterator == reinterpret_cast<const IteratorImpl*>(&other)->iterator;
                        }

                        bool operator!=(const IteratorBase<Base>& other) const {
                            return iterator != reinterpret_cast<const IteratorImpl*>(&other)->iterator;
                        }

                        T operator*() const {
                            return *iterator;
                        }
                };

                Iterator<Base> begin() const {
                    if (filter) {
                        auto start = std::find_if(array.begin(), array.end(), filter);
                        return Iterator<Base>(new IteratorImpl(start, array.end(), filter));
                    } else {
                        return Iterator<Base>(new IteratorImpl(array.begin(), array.end()));
                    }
                }

                Iterator<Base> end() const {
                    return Iterator<Base>(new IteratorImpl(array.end(), array.end()));
                }

        };

    }

}
}
