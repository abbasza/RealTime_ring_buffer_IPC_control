# Reproducible build/test environment for this repo — not a running
# service (there's no single "app" here, just a set of comparison demos),
# but a container that builds and runs everything the same way the CI
# workflow (.github/workflows/ci.yml) does, so "works in CI" and "works
# for me locally" mean the same thing.
#
# Build: docker build -t rt-motion-ipc .
# Run:   docker run --rm rt-motion-ipc
#   (runs every demo once and exits; see CMD below)
# Or drop into a shell to run things individually:
#   docker run --rm -it rt-motion-ipc bash

FROM rust:1-slim-bookworm AS base

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . .

# --- build both halves ---
RUN cd rust && cargo build --release --all-targets
RUN cd rust && cargo test --release
RUN cd cpp && make all

# Note on results from inside this container: like the CI runner, this is
# a shared, non-isolated environment — not a PREEMPT_RT box with pinned
# cores. The relative comparisons (lock-free floor vs. lock-based floor)
# hold up; absolute tail-latency numbers will look worse here than on real
# target hardware. docs/interview-notes.md explains which numbers to trust
# and why.
CMD ["bash", "-c", "\
    echo '=== Rust: rtrb_bridge ===' && ./rust/target/release/rtrb_bridge; \
    echo; echo '=== Rust: channel_baseline ===' && ./rust/target/release/channel_baseline; \
    echo; echo '=== Rust: heapless_spsc_mcu ===' && ./rust/target/release/heapless_spsc_mcu; \
    echo; echo '=== C++: naive_tcp_ipc ===' && ./cpp/bin/naive_tcp_ipc; \
    echo; echo '=== C++: lockfree_shm_ipc ===' && ./cpp/bin/lockfree_shm_ipc; \
    echo; echo '=== C++: false_sharing_bench ===' && ./cpp/bin/false_sharing_bench \
    "]
