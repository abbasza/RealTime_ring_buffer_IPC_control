// common.hpp
//
// The two data layouts under comparison for the Brain -> Bridge hand-off.
//
//   JointStateAoS : "the usual way" — Array of Structs. Natural to write,
//                   natural to get wrong: adjacent joints share cache lines,
//                   so a writer touching joint[0] and a reader touching
//                   joint[1] can be fighting over the same 64-byte line.
//
//   JointStateSoA : Struct of Arrays, with each array's *access pattern*
//                   given its own cache-line-aligned home. Iterating
//                   positions only touches position cache lines: the
//                   prefetcher sees a linear stride of doubles and stays
//                   saturated, and Bridge readers no longer share lines
//                   with unrelated fields (velocity, torque) they're not
//                   even touching this tick.
//
#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

inline constexpr std::size_t kNumJoints = 7;
inline constexpr std::size_t kCacheLine = 64;

// ---------------------------------------------------------------------
// "Usual way": Array of Structs, no alignment control.
// ---------------------------------------------------------------------
struct JointStateAoS {
    double position;
    double velocity;
    double torque;
    // sizeof(JointStateAoS) == 24 bytes on a common ABI -> the runtime
    // packs multiple joints into a single 64-byte line with no say from
    // you about which joints land together. Joint 0 and Joint 1 (and
    // part of Joint 2) are very likely on the SAME cache line.
};

using JointArrayAoS = JointStateAoS[kNumJoints];

// ---------------------------------------------------------------------
// Cache-aware layout: Struct of Arrays, each field its own contiguous,
// cache-line-aligned block.
// ---------------------------------------------------------------------
struct alignas(kCacheLine) JointStateSoA {
    // alignas(64) on the *struct* pins its start to a cache-line boundary;
    // that only matters here because each member below is itself a
    // contiguous, tightly packed array — the SoA property — so scanning
    // `position[]` alone never pulls in a `velocity` or `torque` byte.
    double position[kNumJoints];
    double velocity[kNumJoints];
    double torque[kNumJoints];
    // 7 doubles = 56 bytes per field: this happens to leave `velocity[]`
    // and `torque[]` NOT starting on their own cache line boundary. For a
    // production layout you'd round each array up to a multiple of 64
    // bytes (e.g. pad kNumJoints to 8 doubles = 64 bytes) so no two
    // fields ever share a line. See the padded variant below.
};

// Padded-to-cache-line-multiple version: this is what you'd actually ship.
// Rounding kNumJoints (7) up to 8 doubles = exactly 64 bytes per field, so
// position[], velocity[], and torque[] each start AND end on a cache-line
// boundary — zero possibility of false sharing between fields, and the
// unused 8th slot costs 24 bytes total to buy that guarantee.
inline constexpr std::size_t kPaddedJoints = 8; // 8 * 8 bytes = 64 bytes

struct alignas(kCacheLine) JointStateSoAPadded {
    double position[kPaddedJoints];
    double velocity[kPaddedJoints];
    double torque[kPaddedJoints];
};
static_assert(sizeof(JointStateSoAPadded::position) == kCacheLine,
              "position[] must be exactly one cache line");
static_assert(sizeof(JointStateSoAPadded::velocity) == kCacheLine,
              "velocity[] must be exactly one cache line");
