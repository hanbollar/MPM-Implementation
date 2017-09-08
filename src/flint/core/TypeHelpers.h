
#pragma once

#define DEFAULT_TYPE(T1, T2, op) decltype(std::declval<T1>() op std::declval<T2>())
