# Makefile for AP_TiltHexa HAL-free math core
# Standalone build with g++ (no waf, no HAL dependencies)
# Sources now live in the parent directory; tests/ and core/ are subdirectories.
#
# Usage:
#   make            Build all test targets + shared library (float mode default)
#   make test       Build and run all tests
#   make lib        Build libthx_core.so only
#   make double     Build in double-precision mode
#   make clean

CXX      := g++
CXXFLAGS := -std=gnu++11 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti
LDFLAGS  := -lm
INC      := -I.. -I.

CORE_SRCS := ../AP_TiltHexa_Effectiveness.cpp \
             ../AP_TiltHexa_Constraints.cpp \
             ../AP_TiltHexa_QP.cpp \
             ../AP_TiltHexa_PI.cpp \
             ../AP_TiltHexa_INDI.cpp \
             ../AP_TiltHexa_LowPass.cpp \
             ../AP_TiltHexa_Pipeline.cpp \
             ../AP_TiltHexa_Trajectory.cpp

CAPI_SRCS := ../AP_TiltHexa_CAPI.cpp ../AP_TiltHexa_CAPI_QP.cpp

BIN_DIR := build
CORE_OBJS := $(addprefix $(BIN_DIR)/,$(notdir $(CORE_SRCS:.cpp=.o)))
CAPI_OBJS := $(addprefix $(BIN_DIR)/,$(notdir $(CAPI_SRCS:.cpp=.o)))

.PHONY: all test lib clean

all: $(BIN_DIR)/test_effectiveness $(BIN_DIR)/test_constraints \
     $(BIN_DIR)/test_qp_solver $(BIN_DIR)/test_pi_allocator \
     $(BIN_DIR)/test_indi $(BIN_DIR)/e0_boundary $(BIN_DIR)/libthx_core.so

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Pattern rule for core objects (source in ../, object in $(BIN_DIR)/)
# Placing .o in $(BIN_DIR) prevents precision contamination between
# float and double builds (verifier blocker).
$(BIN_DIR)/%.o: ../%.cpp | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INC) -fPIC -c -o $@ $<

# Shared library from all core sources (HAL-free, -fPIC)
$(BIN_DIR)/libthx_core.so: $(CORE_OBJS) $(CAPI_OBJS) | $(BIN_DIR)
	$(CXX) -shared -o $@ $(CORE_OBJS) $(CAPI_OBJS) $(LDFLAGS)

lib: $(BIN_DIR)/libthx_core.so

$(BIN_DIR)/test_effectiveness: ../tests/test_effectiveness.cpp $(CORE_OBJS) ../tests/test_harness.h | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INC) -o $@ $< $(CORE_OBJS) $(LDFLAGS)

$(BIN_DIR)/test_constraints: ../tests/test_constraints.cpp $(CORE_OBJS) ../tests/test_harness.h | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INC) -o $@ $< $(CORE_OBJS) $(LDFLAGS)

$(BIN_DIR)/test_qp_solver: ../tests/test_qp_solver.cpp $(CORE_OBJS) ../tests/test_harness.h | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INC) -o $@ $< $(CORE_OBJS) $(LDFLAGS)

$(BIN_DIR)/test_pi_allocator: ../tests/test_pi_allocator.cpp $(CORE_OBJS) ../tests/test_harness.h | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INC) -o $@ $< $(CORE_OBJS) $(LDFLAGS)

$(BIN_DIR)/test_indi: ../tests/test_indi.cpp $(CORE_OBJS) ../tests/test_harness.h | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INC) -o $@ $< $(CORE_OBJS) $(LDFLAGS)

$(BIN_DIR)/e0_boundary: ../tests/e0_boundary.cpp $(CORE_OBJS) ../tests/test_harness.h | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INC) -o $@ $< $(CORE_OBJS) $(LDFLAGS)

test: all
	@echo "=== Running tests (float mode) ==="
	@$(BIN_DIR)/test_effectiveness
	@echo ""
	@$(BIN_DIR)/test_constraints
	@echo ""
	@$(BIN_DIR)/test_qp_solver
	@echo ""
	@$(BIN_DIR)/test_pi_allocator
	@echo ""
	@$(BIN_DIR)/test_indi
	@echo ""
	@echo "=== All tests passed (float) ==="

e0_test: $(BIN_DIR)/e0_boundary
	@echo "=== E0 Boundary Tests ==="
	@$(BIN_DIR)/e0_boundary ../tests/E0_boundary_tests.csv
	@cat ../tests/E0_boundary_tests.csv

double:
	$(MAKE) CXXFLAGS="$(CXXFLAGS) -DTHX_USE_DOUBLE" BIN_DIR=build_double all

test_double:
	$(MAKE) CXXFLAGS="$(CXXFLAGS) -DTHX_USE_DOUBLE" BIN_DIR=build_double test

clean:
	rm -rf build build_double