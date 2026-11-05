# ESP32-S3 Triple-FOC Mainboard: Reference Design Package

## Limitations

- **I don't have context from previous "cycles."** This is a self-contained design. If Cycle 29 fixed pin choices, a power topology, or a form factor, tell me and I'll reconcile it.
- **No one can hand you a verified "production-ready" design in a chat reply.** Production readiness comes from ERC/DRC, datasheet cross-checks, a prototype spin, and bring-up. What follows is a complete, internally consistent design you can capture in KiCad 8. Items I'm less certain of are marked **[VERIFY]**.
- **KiCad 8 netlists aren't normally hand-written.** You capture the schematic, and Eeschema generates the netlist and pushes it to the PCB with "Update PCB from Schematic." So the connectivity below is organized net-by-net per sheet, which maps one-to-one onto schematic capture. Module/IC pin numbers are given where I'm confident. The TMC6300 is specified by pin *name*, because you must map names to your library symbol's numbers yourself.

---

## 1. Architecture and Key Design Decisions

| Decision | Choice | Reason |
|---|---|---|
| MCU module | **ESP32-S3-WROOM-1-N16R2** (quad PSRAM) | Octal-PSRAM variants (R8) consume GPIO35–37, and this design needs 28 GPIOs. |
| Motor supply (VM) | 2S Li-ion/LiPo, 6.0