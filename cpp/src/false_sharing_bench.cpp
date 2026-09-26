// false_sharing_bench.cpp
//
// Isolates the false-sharing effect described in the prompt: two threads,
// each hammering its OWN counter, but the counters are adjacent in memory.
// If they land on the same 64-byte cache line, every write from thread A
// invalidates the line in thread B's core (MESI protocol), forcing a
// coherence round-trip on the interconnect on every single increment even
// though the threads never touch each other's data.
//
// Build: g++ -O2 -std=c++20 -pthread false_sharing_bench.cpp -o false_sharing_bench
//
// IMPORTANT: this effect is a CROSS-CORE cache-coherence cost. It only
// shows up when the two threads are actually scheduled on two different
// physical cores. On a single-core box (e.g. a constrained container) the
// OS just time-slices one core between the threads and you will NOT see
// the gap this benchmark is built to demonstrate — that's a property of
// the test environment, not of the phenomenon. On the real Bridge box,
// where the EtherCAT thread is pinned to an isolated core via `isolcpus`
// + `taskset`/`sched_setaffinity`, this is exactly the class of hazard
// that isolation-without-cache-awareness still leaves on the table if two
// *different* threads end up sharing a cache line across two *different*
// isolated cores.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

#include "common.hpp"

constexpr long kIncrements = 200'000'000;

// --- Unpadded: two counters back-to-back, almost certainly one line ---
struct UnpaddedCounters {
    std::atomic<uint64_t> a{0};
    std::atomic<uint64_t> b{0};
};

// --- Padded: each counter gets its own cache line, guaranteed ---
struct PaddedCounters {
    alignas(kCacheLine) std::atomic<uint64_t> a{0};
    alignas(kCacheLine) std::atomic<uint64_t> b{0};
};

template <typename Counters>
long long run_bench(const char* label) {
    static_assert(sizeof(Counters) >= 8, "sanity");
    auto counters = std::make_unique<Counters>();

    auto worker = [](std::atomic<uint64_t>& counter) {
        for (long i = 0; i < kIncrements; ++i) {
            counter.fetch_add(1, std::memory_order_relaxed);
        }
    };

    auto start = std::chrono::steady_clock::now();
    std::thread t1(worker, std::ref(counters->a));
    std::thread t2(worker, std::ref(counters->b));
    t1.join();
    t2.join();
    auto end = std::chrono::steady_clock::now();

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::printf("%-32s %8lld ms  (a=%lu, b=%lu)  [sizeof=%zu, offsetof(b)=%zu]\n",
                label, static_cast<long long>(ms),
                static_cast<unsigned long>(counters->a.load()),
                static_cast<unsigned long>(counters->b.load()),
                sizeof(Counters),
                offsetof(Counters, b));
    return ms;
}

int main() {
    std::printf("hardware_concurrency() reports %u logical CPUs\n",
                std::thread::hardware_concurrency());
    std::printf("(false sharing needs >=2 physical cores to manifest; on a "
                "1-core sandbox both runs will look similar)\n\n");

    auto unpadded_ms = run_bench<UnpaddedCounters>("unpadded (false-shared)");
    auto padded_ms = run_bench<PaddedCounters>("padded (alignas(64))");

    if (padded_ms > 0) {
        std::printf("\nunpadded/padded ratio: %.2fx\n",
                    static_cast<double>(unpadded_ms) / static_cast<double>(padded_ms));
    }
    return 0;
}
