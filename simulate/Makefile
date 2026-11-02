# Host simulation of the audio pipeline and fuzzing of the protocol parsers
#   make run     scenarios
#   make check   scenarios and fuzzer with AddressSanitizer/UBSan (slower, catches memory bugs)

CC       ?= cc
FIRMWARE := ../main
INCLUDES := -Ishim -I. -I$(FIRMWARE) -I$(FIRMWARE)/audio -I$(FIRMWARE)/plist -I$(FIRMWARE)/rtsp -I$(FIRMWARE)/hap
CFLAGS   := -std=gnu11 -g -Wall -Wno-unused-function $(INCLUDES)
SANITIZE := -O1 -fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer

SIMULATE_SOURCES := simulate.c fakes.c shim/esp_shim.c $(FIRMWARE)/audio/audio_buffer.c $(FIRMWARE)/audio/audio_stream.c
FUZZ_SOURCES     := fuzz.c shim/esp_shim.c $(FIRMWARE)/plist/bplist_parser.c $(FIRMWARE)/rtsp/rtsp_message.c $(FIRMWARE)/hap/tlv8.c
HEADERS := $(wildcard *.h shim/*.h shim/freertos/*.h $(FIRMWARE)/audio/*.h $(FIRMWARE)/config.h)
# simulate.c includes these firmware sources directly
INCLUDED := $(FIRMWARE)/audio/audio_receiver.c $(FIRMWARE)/audio/audio_stream_realtime.c $(FIRMWARE)/audio/audio_stream_buffered.c

build/simulate: $(SIMULATE_SOURCES) $(INCLUDED) $(HEADERS)
	@mkdir -p build
	$(CC) $(CFLAGS) -O2 -o $@ $(SIMULATE_SOURCES) -lm

build/simulate-asan: $(SIMULATE_SOURCES) $(INCLUDED) $(HEADERS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) -o $@ $(SIMULATE_SOURCES) -lm

build/fuzz: $(FUZZ_SOURCES) $(HEADERS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) -o $@ $(FUZZ_SOURCES)

.PHONY: run check clean
run: build/simulate
	./build/simulate

check: build/simulate-asan build/fuzz
	./build/fuzz 300000
	./build/simulate-asan

clean:
	rm -rf build
