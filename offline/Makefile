# Desktop build of the REFFT stamp: the board's Stamp.cpp and Limiter.cpp,
# compiled against a portable stand-in for Bela's Fft (include/).
# `make` builds the tools, `make check` runs the self-checks, `make kit`
# writes the test signals to kit/.

CXX ?= c++
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++14 -Wall -Wextra -Wno-unused-parameter -Iinclude -I../cpp -I.

STAMP = ../cpp/Stamp.cpp ../cpp/Limiter.cpp
HEADERS = engine.h knobs.h signals.h wav.h include/libraries/Fft/Fft.h ../cpp/Stamp.h ../cpp/Limiter.h
TOOLS = build/refft-render build/refft-check build/refft-gen

all: $(TOOLS)

build/%: %.cpp $(STAMP) $(HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $< $(STAMP)

check: build/refft-check
	./build/refft-check

kit: build/refft-gen
	./build/refft-gen kit

clean:
	rm -rf build

.PHONY: all check kit clean
