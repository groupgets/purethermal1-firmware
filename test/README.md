# Host-side tests

Tests for the Lepton acquisition loop (`Src/lepton_task.c`) that run on a PC in
under a second. You don't need hardware or an ARM toolchain, only `gcc`,
`make` and `python3`.

```sh
make test                # from the repo root
make -C test run T=wedge # only tests whose name contains "wedge"
make -C test mutants     # check the tests still catch known past bugs
```

Run `make test` before flashing anything that touches the acquisition path. If
a test fails, either the change broke behaviour that was deliberately put
there, or the behaviour was meant to change. In the second case, update the
test in the same commit and say why.

## How it works

`Src/lepton_task.c` is compiled **unmodified** and linked against a simulated
board:

| File | Role |
|------|------|
| `fakes/` | Stand-ins for the STM32 HAL and ST USB headers. They come first on the include path, so firmware sources build on a PC. Everything else (`Inc/`, the Lepton SDK, the real `usbd_uvc.h`) is the real header. |
| `sim.c` / `sim.h` | The simulated board. It replaces `Src/lepton.c` (SPI/DMA) and `Src/lepton_i2c.c` (CCI), models the Lepton's VoSPI output and VSYNC, and runs `lepton_task()` once per 100 µs of simulated time. A stand-in for `usb_task` takes each finished frame and checks it is a complete, in-order segment. |
| `test_lepton_task.c` | The tests. |
| `test_main.c` / `test.h` | A small runner. Each test runs in its own process, because the firmware keeps its state in function-level `static`s, and each one times out after 30 s, so a firmware loop that stops yielding fails instead of hanging. |
| `mutants.py` | Puts past bugs back into a scratch copy of `lepton_task.c` and confirms some test fails for each. Run it after editing tests, to make sure they still catch regressions. |

The tests build with AddressSanitizer and UBSan, so an out-of-bounds write
into a frame buffer fails the run even if the result looks right.

### The simulated sensor

- Every segment period (27 Hz for Lepton 2.x, 106 Hz for 3.x) the sensor starts
  a new segment at packet 0 and raises VSYNC. Reads past the last packet get
  discard packets.
- A VSYNC edge latches while the EXTI line is masked and fires when it is
  unmasked, the same way the NVIC does.
- Lepton 3 segment numbers go in packet 20. The sequence is configurable, so
  repeated frames (segment 0) can be simulated.

Faults the tests can inject (fields of `struct sim`):

| Field | Simulates |
|-------|-----------|
| `skip_next` | The next read starts N packets into a segment (lost alignment) |
| `gap_at_packet` / `gap_len` | Packets lost partway through a segment |
| `garble_next_packet0` | A bit error in the first packet's header |
| `silent_packets` | Nothing driving MISO: reads come back 0x0000 |
| `wedged`, `wedge_resets_needed` | The field wedge: discard packets and silence forever, while VSYNC keeps firing, until N hardware resets (`-1` means never) |
| `dma_hang` | A transfer that never completes |
| `restore_vsync_result`, `reinit_result` | CCI calls failing |
| `consume = 0` | USB never takes frames (ring buffer fills) |

### What the simulator does not model

It checks the firmware's logic, not the electrical layer. SPI clock rates,
real DMA, signal integrity, and the *cause* of the wedge are out of scope.
Those still need the bench work in `plan.md`. Timing is approximate: a read
completes in one step, whatever its length.

## Adding a test

```c
TEST(my_new_case)
{
  sim_init();          /* Lepton 2.5, Y16; call sim_use_lepton3() for 3.x */
  sim_start_stream();
  sim_run_ms(1000);

  sim.skip_next = 10;  /* inject a fault */
  sim_run_ms(1000);

  CHECK_EQ(g_dbg.desync_events, 1);
  CHECK_EQ(sim.frames_corrupt, 0);   /* nothing corrupt reached USB */
}
```

Any new `test_*.c` file is picked up automatically. To test another firmware
source, add it to `FW_SRCS` in the Makefile and fake whatever it calls in
`sim.c`.

If you fix a bug, add a mutant for it to `mutants.py` as well, so the test for
it can't quietly stop catching it.
