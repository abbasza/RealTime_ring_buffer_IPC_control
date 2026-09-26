// naive_tcp_ipc.cpp
//
// "The usual way": Brain and Bridge as two separate processes talking over
// a TCP socket (the same class of mechanism as a REST call — user space ->
// kernel -> socket buffer -> kernel -> user space, with a context switch on
// each hop). Payload is the naive Array-of-Structs `JointArrayAoS`.
//
// This file forks: the child is "the Brain" (client), the parent is
// "the Bridge" (server). Every tick: Bridge asks for the latest joint
// targets, Brain sends the AoS blob back, Bridge measures round-trip time.
// This is deliberately the boring, correct, "how most people would first
// wire this up" implementation — it's the baseline the optimized version
// in `lockfree_shm_ipc.cpp` is measured against.
//
// Build: g++ -O2 -std=c++20 naive_tcp_ipc.cpp -o naive_tcp_ipc
// Run:   ./naive_tcp_ipc

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include "common.hpp"

constexpr int kPort = 47821;
constexpr int kTicks = 3000; // 3000 request/response round-trips

static void run_bridge_server() {
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(kPort);
    bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    listen(listen_fd, 1);

    int conn_fd = accept(listen_fd, nullptr, nullptr);
    // Disable Nagle's algorithm — otherwise the kernel batches small writes
    // and adds up to ~40ms of its own latency, which would make this
    // benchmark measure Nagle instead of the syscall/context-switch cost
    // we actually care about.
    int one = 1;
    setsockopt(conn_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    JointArrayAoS joints{};
    std::vector<int64_t> latencies_ns;
    latencies_ns.reserve(kTicks);

    for (int tick = 0; tick < kTicks; ++tick) {
        auto t0 = std::chrono::steady_clock::now();

        // "request the latest targets" — one syscall, one kernel copy in
        uint8_t ping = 1;
        if (write(conn_fd, &ping, sizeof(ping)) < 0) break;

        // Brain replies with the full AoS blob — another syscall, another
        // kernel copy out on the Brain side and copy in here.
        ssize_t received = 0;
        auto* buf = reinterpret_cast<uint8_t*>(&joints);
        while (received < static_cast<ssize_t>(sizeof(joints))) {
            ssize_t n = read(conn_fd, buf + received, sizeof(joints) - received);
            if (n <= 0) break;
            received += n;
        }

        auto t1 = std::chrono::steady_clock::now();
        latencies_ns.push_back(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    }

    close(conn_fd);
    close(listen_fd);

    std::sort(latencies_ns.begin(), latencies_ns.end());
    size_t n = latencies_ns.size();
    if (n == 0) {
        std::printf("no samples\n");
        return;
    }
    int64_t sum = 0;
    for (auto v : latencies_ns) sum += v;
    std::printf("=== naive TCP loopback (AoS payload) ===\n");
    std::printf("samples: %zu\n", n);
    std::printf("min:     %8ld ns\n", latencies_ns.front());
    std::printf("mean:    %8.1f ns\n", static_cast<double>(sum) / n);
    std::printf("p50:     %8ld ns\n", latencies_ns[n / 2]);
    std::printf("p99:     %8ld ns\n", latencies_ns[(n * 99) / 100]);
    std::printf("max:     %8ld ns  <-- worst-case round trip\n", latencies_ns.back());
}

static void run_brain_client() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(kPort);

    // Bridge's listen socket may not be up yet; retry briefly.
    for (int attempt = 0; attempt < 200; ++attempt) {
        if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) break;
        usleep(5000);
    }
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    JointArrayAoS joints{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        joints[i] = {static_cast<double>(i) * 0.1, 0.0, 0.0};
    }

    for (int tick = 0; tick < kTicks; ++tick) {
        uint8_t ping;
        ssize_t n = read(fd, &ping, sizeof(ping));
        if (n <= 0) break;

        joints[0].position = tick * 0.001; // pretend the IK result changed
        if (write(fd, &joints, sizeof(joints)) < 0) break;
    }
    close(fd);
}

int main() {
    pid_t pid = fork();
    if (pid == 0) {
        // Give the parent a head start to bind/listen.
        usleep(20000);
        run_brain_client();
        _exit(0);
    } else {
        run_bridge_server();
        int status = 0;
        waitpid(pid, &status, 0);
    }
    return 0;
}
