
#pragma once

#include <type_traits>
#include <utility>

namespace core {

template <typename Value>
class Optional {

public:

  using value_t = Value;

  Optional() noexcept { }

  Optional(const Optional &other) {
    if (other) {
      construct(std::move(other.value()));
    }
  }

  Optional(Optional&& other) {
    if (other) {
      construct(std::move(other.value()));
      other.clear();
    }
  }

  template<class... Args>
  void construct(Args&&... args) {
    const void* ptr = &storage.value;
    new(const_cast<void*>(ptr)) Value(std::forward<Args>(args)...);
    storage.hasValue = true;
  }

  bool hasValue() const { return storage.hasValue; }

  void clear() { storage.clear(); }

  void set(Value&& value) { construct(std::move(value)); }
  void set(const Value& value) { construct(value); }

  const Value& value() const& { return storage.value; }
  Value& value() & { return storage.value; }
  const Value&& value() const&& { return std::move(storage.value); }

  explicit operator bool() const { return hasValue(); }

  const Value& operator*()  const&  { return value(); }
        Value& operator*()       &  { return value(); }
  const Value&& operator*() const&& { return std::move(value()); }
        Value&& operator*()      && { return std::move(value()); }

  const Value* operator->() const { return &value(); }
        Value* operator->()       { return &value(); }

private:

  struct TrivialStorage {
    union {
      bool hasValue = false;
      struct {
        bool _hasValue;
        Value value;
      };
    };

    TrivialStorage() : hasValue(false) { }

    void clear() {
      hasValue = false;
    }
  };

  struct NonTrivialStorage {
    union {
      bool hasValue;
      struct {
        bool _hasValue;
        Value value;
      };
    };

    NonTrivialStorage() : hasValue(false) { }
    ~NonTrivialStorage() {
      clear();
    }

    void clear() {
      if (hasValue) {
        hasValue = false;
        value.~Value();
      }
    }
  };

  using Storage = typename std::conditional<
    std::is_trivially_destructible<Value>::value,
    TrivialStorage,
    NonTrivialStorage
  >::type;

  Storage storage;


};

}
