#ifndef VK_RAY_TRACING_TUTORIAL_COMMON_TYPES_H
#define VK_RAY_TRACING_TUTORIAL_COMMON_TYPES_H

#include <stdint.h>
#include <type_traits>

using i8  = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;
using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using f32 = float;
using f64 = double;

#define CHECK_TYPE_SIZE(type, bit_width)                                                           \
    static_assert(sizeof(type) == bit_width / 8, #type " doesn't have a correct size")
#define DEMON_CHECK_TYPE_SIZE(prefix, bit_width) CHECK_TYPE_SIZE(prefix##bit_width, bit_width)

DEMON_CHECK_TYPE_SIZE(u, 8);
DEMON_CHECK_TYPE_SIZE(u, 16);
DEMON_CHECK_TYPE_SIZE(u, 32);
DEMON_CHECK_TYPE_SIZE(u, 64);
DEMON_CHECK_TYPE_SIZE(i, 8);
DEMON_CHECK_TYPE_SIZE(i, 16);
DEMON_CHECK_TYPE_SIZE(i, 32);
DEMON_CHECK_TYPE_SIZE(i, 64);
DEMON_CHECK_TYPE_SIZE(f, 32);
DEMON_CHECK_TYPE_SIZE(f, 64);

#undef DEMON_CHECK_TYPE_SIZE
#undef CHECK_TYPE_SIZE

//#define CHECK_TYPE_SIZE(prefix, bit_width) \
//    static_assert(sizeof(prefix##bit_width) == bit_width / 8,                                      \
//                  #prefix #bit_width"doesn't have a correct size")
//CHECK_TYPE_SIZE(u, 16);
//#undef CHECK_TYPE_SIZE

// Shorthands for static casts.
template <typename dst_type, typename src_type>
dst_type cast(src_type v)
{
    static_assert(std::is_arithmetic_v<dst_type>, "Can't cast to a non-arithmetic type");
    return static_cast<dst_type>(v);
}

#define DECLARE_CAST_FUNCTION(dst_type)                                                            \
    template <typename src_type>                                                                   \
    dst_type cast_##dst_type(src_type v) { return cast<dst_type, src_type>(v); }

// Declares mostly used functions cast fundamental arithmetic types to 32-bit and 64-bit float and
// signed/unsigned integers. The functions are named cast_[type] where type is [i|u|f][32|64].
// Use them instead of static_cast<int>() for shorter code:
// 
//     u32 length =          cast_i32( message.size() )
//     unsigned int length = static_cast<unsigned int>( message.size() )
DECLARE_CAST_FUNCTION(i32)
DECLARE_CAST_FUNCTION(i64)
DECLARE_CAST_FUNCTION(u32)
DECLARE_CAST_FUNCTION(u64)
DECLARE_CAST_FUNCTION(f32)
DECLARE_CAST_FUNCTION(f64)



#endif  // VK_RAY_TRACING_TUTORIAL_COMMON_TYPES_H