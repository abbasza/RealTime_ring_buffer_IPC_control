# Rust — lock-free IPC + CAN-FD framing

See the [top-level README](../README.md) for the full comparison, diagrams,
and measured results. This folder:

```
cargo build --release --all-targets   # builds all 3 demo binaries + the canfd lib
cargo test --release                  # canfd round-trip / CRC / DLC-size tests
cargo run --release --bin rtrb_bridge
cargo run --release --bin channel_baseline
cargo run --release --bin heapless_spsc_mcu
```

| File | What it demonstrates |
|---|---|
| `src/bin/rtrb_bridge.rs` | Brain→Bridge hand-off over `rtrb`, wait-free SPSC |
| `src/bin/channel_baseline.rs` | same experiment with `std::sync::mpsc`, for comparison |
| `src/bin/heapless_spsc_mcu.rs` | Joint MCU tier: `no_std`-compatible ISR↔main-loop queue |
| `src/canfd.rs` | CAN-FD wire framing for joint commands (unit-tested, `cargo test`) |
