# sdram

SDR SDRAM controller.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0%20OR%20MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. The controller drives one x16 SDR SDRAM chip and presents it to the bus as a stalling memory target: each 32-bit bus word is two neighbouring 16-bit words in the same row, and the byte strobes become the DQM of each word. It runs the power-up sequence, refreshes one row at a time often enough to cover the whole chip every 64 ms, and serves one word per ACTIVE.

The timing follows three datasheets: Winbond W9825G6KH, Micron's 256Mb SDRAM (MT48LC16M16A2) and Alliance Memory AS4C4M16SA. They agree on the command truth table and the mode register, and differ on the power-up pause (100 or 200 µs), the number of AUTO REFRESH cycles (2 or 8) and a few minimum intervals. The controller takes the strictest value each time, so it works with parts from all three.

The chip is described in `SdramChip.bs`, written in Bluespec Haskell. Row and column widths are numeric types, so the address split is a packed struct. There is one equation per command, and the datasheet times in nanoseconds are turned into clock cycles at compile time. A refresh interval too short for an access, or CAS latency 2 above 100 MHz, stops the build. `Sdram.bsv` is the engine: one rule that steps through initialisation, refresh and access.

The testbench drives a chip model that recomputes every limit from the datasheet numbers rather than from `SdramChip.bs`. It checks the power-up order and pause, tRCD, tRP, tRC, tWR and the mode register value, and returns read data at the loaded CAS latency. It then writes and reads the first and last words of every bank, checks that a half-strobed write keeps the other bytes, runs 160 busy rounds and an idle stretch while measuring the longest gap between refreshes, and checks that an address past the chip returns an error without touching the bus.

## Parameters

| Parameter | Range | Meaning |
| :--: | :--: | :-- |
| `rowBits` | 12 to 13 | row address width: 13 for 256Mb, 12 for 64Mb |
| `colBits` | 8 to 9 | column address width: 9 for 256Mb, 8 for 64Mb |
| `cl` | 2 to 3 | CAS latency written to the mode register |
| `mhz` | 10 to 100 | clock frequency, used to turn datasheet times into cycles |

## Pins

`sdr_cke`, `sdr_cs_n`, `sdr_ras_n`, `sdr_cas_n`, `sdr_we_n`, `sdr_addr[12:0]`, `sdr_ba[1:0]` and `sdr_dqm[1:0]` go to the chip. `sdr_dq_o`, `sdr_dq_oe` and `sdr_dq_i` make up the bidirectional data bus, and the pad or board ties them together.

Burst access, self refresh, power down, clock suspend, more than one chip and x8 or x4 parts are not implemented. Board-level timing such as output delay and setup and hold at the chip is left to static timing analysis and the board.

## Specification sources

The specifications this IP is implemented against, with their links, digests and the clause-by-clause comparison, are kept on the [`spec` branch](https://github.com/Tape-Out/sdram/tree/spec).

## License

任选其一：

- [MIT](LICENSE-MIT)
- [Apache 2.0](LICENSE-APACHE)
- [木兰宽松许可证 第2版](LICENSE-MULAN)

`SPDX-License-Identifier: MIT OR Apache-2.0 OR MulanPSL-2.0`

除非另行说明，你提交的贡献按上述三者同时授权，不附加其他条件。
