# ESP32-S3 Motion Controller: 3× TMC6300 FOC, 3× AS5600, 2× ICM-42688-P

## Read this first

I don't have the earlier design cycles, so this is a fresh, self-consistent design. It isn't a drop-in continuation of "Cycle 14." Before fabrication, check these items:

1. **Pin numbers.** Every pin number below must be verified against the KiCad library symbol and footprint you actually use. Pin numbers for the ESP32-S3-WROOM-1, AS5600, TCA9548A and AP63203 are standard. Treat the TMC6300 (QFN-20) and ICM-42688-P (LGA-14) numbers as **to-verify**, because they are easy to get wrong and are fatal if wrong.
2. **Use the netlist as a reference, not as the source of truth.** In KiCad 8, the schematic should be the source of truth. Use this netlist to enter or check the schematic, or import it in Pcbnew (File → Import → Netlist) for early placement.
3. **Board-level decisions you should consciously accept:**
   - **AS5600 addressing.** The AS5600 has a fixed I²C address of 0x36, so three of them cannot share one bus. This design adds a **TCA9548A I²C mux**.
   - **GPIO budget.** Three 6-PWM drivers use 18 pins. With the N16 module (no octal PSRAM) there are about 30 usable GPIOs. The PWM, SPI, I²C and f