# Audio pipeline simulation

Runs the real receiver code from `main/audio` on the Mac against a simulated phone,
WiFi link and I2S output, so buffering, skips, pauses and clock drift can be tested
in seconds instead of hours in the car.

```
make run                     # build and run every scenario (~10 s)
make check                   # fuzzer + every scenario under AddressSanitizer/UBSan
./build/simulate -l          # list scenarios
./build/simulate -v realtime-skip   # one scenario with firmware logs, gaps and jumps
make clean
```

Scenarios cover steady play, clock drift (±300 ppm for 2 h), skips, skip spam, pauses,
WiFi stalls and outages, loss bursts, duplicates, malformed packets, session restarts,
format changes mid-play, lossless-size packets, the "up next" change and timestamp /
sequence number wraparound.

## Fuzzer

`fuzz.c` feeds generated and corrupted binary plists (the SETUP, FLUSHBUFFERED,
SETRATEANCHORTIME bodies), RTSP requests and TLV8 to the firmware parsers, each in an
exactly sized buffer so AddressSanitizer catches any read past the end.
`./build/fuzz 1000000` runs a longer pass.

## What is real and what is faked

Compiled from the firmware unchanged: `audio_receiver.c`, `audio_buffer.c`,
`audio_stream.c`, `audio_stream_realtime.c` (reorder buffer, retransmits, decode
queue) and `audio_stream_buffered.c` (per-packet handling); for the fuzzer
`bplist_parser.c`, `rtsp_message.c` and `tlv8.c`. `audio_output.c` is not, the
simulator reads the receiver the way its playback task does. The real AAC/ALAC
decoders and task stacks can't run here, so stack overflows inside the decoder only
show on the device (the stats line logs the least free stack of the decoding tasks).

Faked:

- **FreeRTOS / ESP-IDF** (`shim/`): ring buffer, critical sections, timer, logging.
  Single threaded, nothing blocks; tasks are never started, the simulator calls
  the same per-packet functions the tasks call.
- **Codec** (`fakes.c`): every decoded sample carries its source timestamp and track,
  so the checker knows exactly which audio reached the speaker and when.
- **Phone and network** (`simulate.c`): realtime sends one ALAC packet per 352
  samples at the phone's clock rate and answers retransmit requests; buffered sends
  AAC packets as fast as the receiver has room, in order (TCP). Delay, jitter, loss,
  stalls (packets held back, then a burst) and outages are configurable per scenario.
- **Speaker**: reads 512 samples at a time at the I2S clock rate, plus the fixed
  186 ms DMA queue when reporting start times and delay.

## What a scenario checks

- **gaps**: sound stopped while the phone was playing (underrun)
- **stale**: audio from before a skip/pause reached the speaker afterwards
- **jumps**: audio skipped or repeated without a reason (drift corrections of one
  sample and concealed packet loss are counted separately)
- **start**: time from play/skip/resume to sound
- **resume**: audio lost at the start of a resume
- **delay** (realtime): phone send to speaker; its spread over the run shows whether
  the phone and car drift apart
- **up-next**: the playing track plays to its end when the next track is replaced

## Assumptions about the phone

The skip and pause behaviour of iOS is modelled from shairport-sync, not measured:

- Realtime skip: `FLUSH` with `RTP-Info: rtptime` of the new audio, sequence
  numbers continue. Pause: `SETRATEANCHORTIME` rate 0, resume carries on where it
  stopped.
- Buffered skip: `FLUSHBUFFERED` with `flushUntilSeq`/`flushUntilTS`. Pause: rate 0,
  resume resends from the pause point with new sequence numbers. Up-next change:
  `FLUSHBUFFERED` with `flushFromSeq`/`flushFromTS` while playing.

If the logs from the car show something different, change the scenario to match.
