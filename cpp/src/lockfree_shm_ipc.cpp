// lockfree_shm_ipc.cpp
//
// "My implementation": Brain and Bridge as two real, separate processes
// (via fork(), same as the naive version, so the comparison is fair —
// the only variable that changes is the IPC mechanism) exchanging joint
// targets through a lock-free SPSC ring buffer living in POSIX shared
// memory (`shm_open` + `mmap`). Zero syscalls per message after setup,
// zero copies through the kernel, zero locks.
//
// This is the same class of construct as `rtrb`/`heapless::spsc` from the
// Rust side of this exercise, hand-rolled in C++ so the head/tail atomics
// and cache-line padding are fully explicit and inspectable — worth having
// both versions (library + hand-rolled) ready to talk through in an
// interview, since "implement the primitive yourself" is a common
// follow-up to "which crate/library would you use."
//
// Build: g++ -O2 -std=c++20 -pthread lockfree_shm_ipc.cpp -lrt -o lockfree_shm_ipc
// Run:   ./lockfree_shm_ipc

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <new>
#include <thread>
#include <vector>

#include "common.hpp"

constexpr const char* kShmName = "/tier1_joint_ring";
constexpr std::size_t kCapacity = 8; // power of two
constexpr int kTicks = 3000;

// A single slot: the SoA joint targets plus a timestamp for latency
// measurement. In production you'd send just JointStateSoAPadded and let
// the Bridge stamp its own receive time; the send timestamp here exists
// purely to measure hand-off latency for this benchmark.
struct Slot {
    uint64_t seq;
    uint64_t sent_at_ns;
    JointStateSoAPadded joints;
};

// ---------------------------------------------------------------------
// Lock-free SPSC ring buffer, hand-rolled. Head/tail atomics are each
// alone on their own cache line (`alignas(64)`) specifically so the
// producer spinning on/writing `head` never invalidates the cache line
// the consumer is reading `tail` from, and vice versa — the same
// false-sharing hazard from `false_sharing_bench.cpp`, applied to the
// ring buffer's own control state rather than user data.
// ---------------------------------------------------------------------
struct RingBuffer {
    alignas(kCacheLine) std::atomic<uint64_t> head{0}; // next slot producer writes
    alignas(kCacheLine) std::atomic<uint64_t> tail{0}; // next slot consumer reads
    alignas(kCacheLine) Slot slots[kCapacity];

    static constexpr uint64_t mask = kCapacity - 1;
    static_assert((kCapacity & mask) == 0, "capacity must be power of two");

    // Producer side (Brain). Wait-free: bounded work, never blocks on the
    // consumer. Returns false if the buffer is full (Bridge fell behind).
    bool try_push(const Slot& item) {
        uint64_t h = head.load(std::memory_order_relaxed);
        uint64_t t = tail.load(std::memory_order_acquire);
        if (h - t >= kCapacity) return false; // full
        slots[h & mask] = item;
        head.store(h + 1, std::memory_order_release);
        return true;
    }

    // Consumer side (Bridge). Wait-free: bounded work, never blocks on the
    // producer. Returns false if empty.
    bool try_pop(Slot& out) {
        uint64_t t = tail.load(std::memory_order_relaxed);
        uint64_t h = head.load(std::memory_order_acquire);
        if (t == h) return false; // empty
        out = slots[t & mask];
        tail.store(t + 1, std::memory_order_release);
        return true;
    }
};

static RingBuffer* map_shared_ring(bool create) {
    int flags = create ? (O_CREAT | O_RDWR) : O_RDWR;
    int fd = shm_open(kShmName, flags, 0666);
    if (fd < 0) {
        std::perror("shm_open");
        _exit(1);
    }
    if (create) {
        if (ftruncate(fd, sizeof(RingBuffer)) != 0) {
            std::perror("ftruncate");
            _exit(1);
        }
    }
    void* addr = mmap(nullptr, sizeof(RingBuffer), PROT_READ | PROT_WRITE,
                       MAP_SHARED, fd, 0);
    close(fd);
    if (addr == MAP_FAILED) {
        std::perror("mmap");
        _exit(1);
    }
    return static_cast<RingBuffer*>(addr);
}

static uint64_t now_ns(const std::chrono::steady_clock::time_point& epoch) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now() - epoch)
        .count();
}

static void run_bridge_consumer(RingBuffer* ring,
                                 const std::chrono::steady_clock::time_point& epoch) {
    std::vector<int64_t> latencies_ns;
    latencies_ns.reserve(kTicks);
    Slot slot;
    JointStateSoAPadded last_known{};

    // Bridge RT loop: fixed 1kHz cadence, non-blocking drain each tick —
    // deliberately paced the same way `rtrb_bridge.rs`'s consumer is, so
    // this is a real periodic control loop and not a CPU-bound busy-spin.
    // (A raw `while(true) try_pop()` spin loop is only a correct design
    // once you've actually pinned each side to its own isolated physical
    // core — on a shared/oversubscribed CPU it just starves the other
    // process of scheduler time, which produces meaningless multi-
    // millisecond "latencies" that are really scheduling artifacts, not
    // IPC cost.)
    using clock = std::chrono::steady_clock;
    auto period = std::chrono::milliseconds(1);
    auto next_tick = clock::now();

    for (int tick = 0; tick < kTicks; ++tick) {
        next_tick += period;
        while (ring->try_pop(slot)) {
            int64_t latency = static_cast<int64_t>(now_ns(epoch)) -
                               static_cast<int64_t>(slot.sent_at_ns);
            latencies_ns.push_back(latency);
            last_known = slot.joints;
        }
        // --- real Bridge would build/transmit the EtherCAT frame from
        // `last_known` right here. ---
        std::atomic_signal_fence(std::memory_order_seq_cst);

        auto now = clock::now();
        if (next_tick > now) {
            std::this_thread::sleep_until(next_tick);
        } else {
            next_tick = now; // missed a deadline; resync instead of busy-catch-up
        }
    }
    (void)last_known;

    std::sort(latencies_ns.begin(), latencies_ns.end());
    size_t n = latencies_ns.size();
    int64_t sum = 0;
    for (auto v : latencies_ns) sum += v;
    std::printf("=== lock-free SPSC over shared memory (SoA payload) ===\n");
    std::printf("samples: %zu\n", n);
    std::printf("min:     %8ld ns\n", latencies_ns.front());
    std::printf("mean:    %8.1f ns\n", static_cast<double>(sum) / n);
    std::printf("p50:     %8ld ns\n", latencies_ns[n / 2]);
    std::printf("p99:     %8ld ns\n", latencies_ns[(n * 99) / 100]);
    std::printf("max:     %8ld ns  <-- worst-case hand-off\n", latencies_ns.back());
}

static void run_brain_producer(RingBuffer* ring,
                                const std::chrono::steady_clock::time_point& epoch) {
    Slot slot{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        slot.joints.position[i] = static_cast<double>(i) * 0.1;
    }

    // Brain paced at 50Hz, same as `rtrb_bridge.rs`'s producer — matches
    // the actual IK/planner output rate this tier is modeling, rather than
    // pushing as fast as possible.
    using clock = std::chrono::steady_clock;
    auto period = std::chrono::milliseconds(20);
    auto next_tick = clock::now();
    // Bridge runs ~20x more ticks than Brain at these rates; stop once the
    // Bridge's tick budget (kTicks at 1kHz) would have elapsed.
    int brain_ticks = kTicks / 20;

    for (int i = 0; i < brain_ticks; ++i) {
        next_tick += period;
        slot.seq = static_cast<uint64_t>(i);
        slot.joints.position[0] = static_cast<double>(i) * 0.001; // "IK output"
        slot.sent_at_ns = now_ns(epoch);
        if (!ring->try_push(slot)) {
            // Full: Bridge fell behind. try_push is still wait-free — we
            // just count/skip rather than spin-retrying, exactly like the
            // rtrb demo's overrun handling.
            std::fprintf(stderr, "[brain] seq=%d overrun: consumer buffer full\n", i);
        }
        auto now = clock::now();
        if (next_tick > now) {
            std::this_thread::sleep_until(next_tick);
        } else {
            next_tick = now;
        }
    }
}

int main() {
    // Clean up any stale segment from a previous crashed run.
    shm_unlink(kShmName);

    RingBuffer* ring = map_shared_ring(/*create=*/true);
    new (ring) RingBuffer(); // placement-new: reset atomics/slots to zero state

    auto epoch = std::chrono::steady_clock::now();

    pid_t pid = fork();
    if (pid == 0) {
        // Child = Brain (producer). Re-map the same segment (already
        // inherited via fork, but map_shared_ring keeps the two paths
        // symmetric with a "real" two-independent-processes deployment).
        run_brain_producer(ring, epoch);
        _exit(0);
    } else {
        run_bridge_consumer(ring, epoch);
        int status = 0;
        waitpid(pid, &status, 0);
        munmap(ring, sizeof(RingBuffer));
        shm_unlink(kShmName);
    }
    return 0;
}
