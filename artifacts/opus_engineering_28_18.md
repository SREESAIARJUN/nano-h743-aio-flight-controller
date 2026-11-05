# Pan-Axis Yoke, Bearing Seats & 6.5 mm Slip-Ring Channel: Parametric Design

I don't have your Cycle 27 parameters, so every value below that comes from a specific vendor part is a **named, flagged default**. That covers the motor bolt circles, the hollow-shaft bore and the slip-ring length. Replace those values with your datasheet values before cutting metal. Everything else is derived from them.

---

## 1. Architecture

The parts stack along the pan axis, bottom to top:

```
 z ↑
   ┌───────────── YOKE (hub + 2 arms, tilt seats) ─────────────┐
   │   slip-ring channel Ø6.5 (body rotates with yoke)         │
   │   ┌─ hub inner-race boss ─┐                               │
 ══╪═══╡ 6704 bearing (upper) ╞══╗                            │
   │   │  housing shoulder     │ ║  BASE HOUSING              │
 ══╪═══╡ 6704 bearing (lower) ╞══╝                            │
   │   └─ spigot Ø20 h6 ──────┘                               │
   │   ROTOR COUPLER (clamps lower inner race, registers spigot)│
   │   HOLLOW-SHAFT BLDC (stator → base floor, rotor → coupler) │
   │   slip-ring tail wires ↓ through motor bore → floor slot   │
   └────────────────────────────────────────────────────────────┘
```

**How the parts work together:**

- **Bearings.** A duplex pair of thin-section 6704 bearings (20×27×4) carries the yoke's moment lo