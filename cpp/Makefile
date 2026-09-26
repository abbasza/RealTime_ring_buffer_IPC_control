CXX ?= g++
CXXFLAGS ?= -O2 -std=c++20 -Wall -Wextra
LDFLAGS_SHM = -lrt -pthread
LDFLAGS_BENCH = -pthread

SRC := src
BIN := bin

.PHONY: all clean run

all: $(BIN)/naive_tcp_ipc $(BIN)/lockfree_shm_ipc $(BIN)/false_sharing_bench

$(BIN):
	mkdir -p $(BIN)

$(BIN)/naive_tcp_ipc: $(SRC)/naive_tcp_ipc.cpp $(SRC)/common.hpp | $(BIN)
	$(CXX) $(CXXFLAGS) $< -o $@

$(BIN)/lockfree_shm_ipc: $(SRC)/lockfree_shm_ipc.cpp $(SRC)/common.hpp | $(BIN)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS_SHM)

$(BIN)/false_sharing_bench: $(SRC)/false_sharing_bench.cpp $(SRC)/common.hpp | $(BIN)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS_BENCH)

run: all
	@echo "--- naive TCP loopback (AoS) ---"; ./$(BIN)/naive_tcp_ipc
	@echo; echo "--- lock-free shared-memory ring (SoA) ---"; ./$(BIN)/lockfree_shm_ipc
	@echo; echo "--- false sharing benchmark ---"; ./$(BIN)/false_sharing_bench

clean:
	rm -rf $(BIN)
