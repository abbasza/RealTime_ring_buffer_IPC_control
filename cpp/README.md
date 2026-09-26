# C++ — Tier 1 cache & IPC optimization

See the [top-level README](../README.md) for the full comparison, diagrams,
and measured results. This folder:

```
make all   # builds all 3 demos into bin/
make run   # builds and runs all 3 in sequence
```

| File | What it demonstrates |
|---|---|
| `src/naive_tcp_ipc.cpp` | "usual way": TCP loopback IPC, Array-of-Structs payload |
| `src/lockfree_shm_ipc.cpp` | hand-rolled lock-free SPSC ring buffer in POSIX shared memory, Struct-of-Arrays payload |
| `src/false_sharing_bench.cpp` | isolates the false-sharing effect: padded vs. unpadded adjacent atomics |
| `src/common.hpp` | the `JointStateAoS` / `JointStateSoAPadded` layouts both demos share |
