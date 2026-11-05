# Thermal Model of a 4-in-1 ESC at 80 A Total Burst (Cycle 5)

I don't have the earlier cycles' parameters, so this uses explicitly stated assumptions. Each one is a named variable you can swap out.

---

## 0. Assumed Configuration

| Parameter | Symbol | Value |
|---|---|---|
| Battery | V_bus | 6S, 25.2 V max, 22.2 V nominal |
| Current | I_ph | 80 A total, so 20 A per channel × 4 |
| MOSFETs | | 24 × DFN 3×3 (6 per channel) |
| On-resistance | R_ds,on(25 °C) | 1.5 mΩ, temperature coefficient α_R ≈ 0.005 K⁻¹ |
| Switching | f_sw, t_r + t_f | 48 kHz, 40 ns |
| Dead time, diode drop | t_dt, V_f | 100 ns, 0.8 V |
| Shunt | R_sh | 0.5 mΩ per channel |
| PCB | | 40 × 40 mm, 1.6 mm thick, 6 layers of 2 oz Cu (70 µm), ~70 % copper fill |
| Ambient | T_∞ | 35 °C |
| Airflow | h_conv | ≈ 80 W/m²K (prop wash). Use ≈ 10 W/m²K for still air in a stack |
| Emissivity | ε | 0.9 (solder mask) |

---

## 1. Heat-Source Physics (volumetric source q‴)

Assume trapezoidal or synchronous six-step commutation. At any instant, one high-side and one low-side FET conduct per channel. Rds,on rises with temperature as R_ds(T) = R_ds,25 · [1 + α_R(T − 25)].

**Conduction loss**
P_cond = I² · 2 · R_ds(T_j)
At T_j ≈ 100 °C: 400 × 2 × 2.06 mΩ ≈ **1.65 W**

**Switching loss**
P_sw = ½ · V_bus · I · (t_r + t_f) · f_sw
= ½ × 25 × 20 × 40 ns × 48 kHz ≈ **0.48 W**

**Dead-time body-diode loss**
P_dt = V_f · I · 2 t_dt · f_sw ≈ **0.15 W**

**Gate drive loss**
P_g = Q_g · V_gs · f_sw · N_sw ≈ 30 nC × 10 V × 48 kHz × 2 ≈ **0.03 W**
This heat lands in the driver IC, not the FETs.

**Shunt loss**
P_sh = I² · R_sh ≈ **0.20 W**

**Trace / pour I²R loss**
P_Cu = I² · ρ_Cu(T) · L / (w · t) ≈ **0.3 W**

**Per-channel total:** ≈ 2.8 W, so four channels give ≈ 11.2 W.
Adding MCU, BEC, and drivers (~0.8 W) gives **Q_tot ≈ 12 W**.

The sources are mapped onto the mesh as q‴ = P_k / V_k over each FET die, shunt, and trace volume.

---

## 2. Governing Equations

**Strong form (anisotropic steady conduction):**

∇·(**K**∇T) + q‴(**x**, T) = 0  in Ω

The conductivity tensor is **K** = diag(k_xy, k_xy, k_z).

**Effective laminate conductivities** (rule of mixtures; series for k_z):

- In-plane: k_xy = Σ k_i t_i / Σ t_i = (385·0.294 + 0.3·1.306)/1.6 ≈ **71 W/mK**
- Through-plane: k_z = Σ t_i / Σ(t_i / k_i) ≈ **0.37 W/mK**
  - Thermal vias raise this locally under the FET pads. Each via conducts
    G_via = k_Cu π(r_o² − r_i²) / t,
    which is about 0.5 mm diameter with 25 µm plating ≈ 0.009 W/K per via, so an array of 20 gives ≈ 0.18 W/K.

**Boundary conditions on Γ:**

- Robin (convection plus radiation) on exposed faces:
  −**n**·**K**∇T = h_conv(T − T_∞) + εσ(T⁴ − T_∞⁴)
- Contact at the stack standoffs:
  −**n**·**K**∇T = h_c(T − T_FC), with h_c ≈ 1000–3000 W/m²K through nylon or metal standoffs.

**Linearized radiation coefficient:**
h_rad ≈ 4εσT_m³ ≈ 7 W/m²K at 330 K, giving h_eff ≈ 87 W/m²K.

---

## 3. Finite-Element Formulation

**Weak form.** Find T ∈ H¹(Ω) such that, for all test functions v:

∫_Ω ∇v·**K**∇T dΩ + ∫_Γ v · h (T − T_∞) dΓ + ∫_Γ v · εσ(T⁴ − T_∞⁴) dΓ = ∫_Ω v · q‴ dΩ

**Galerkin discretization.** Let T ≈ Σ N_a T_a and **B** = ∇**N**.

- Element conductivity matrix:
  **K**ᵉ = ∫_Ωe **B**ᵀ**K B** dΩ + ∫_Γe h **N**ᵀ**N** dΓ
- Element load vector:
  **f**ᵉ = ∫_Ωe **N**ᵀ q‴ dΩ + ∫_Γe h T_∞ **N**ᵀ dΓ

After assembly:

**K**_g **T** = **F**(**T**)

**Nonlinear solve (Newton–Raphson).** The nonlinearity comes from radiation and from q‴(T) via R_ds(T):

- Residual: **R**(**T**) = **K**_g**T** + **R**_rad(**T**) − **F**(**T**)
- Jacobian: **J** = **K**_g + ∫ 4εσT³ **N**ᵀ**N** dΓ − ∂**F**/∂**T**
- Update: **T**ⁿ⁺¹ = **T**ⁿ − **J**⁻¹**R**

The term ∂**F**/∂**T** contributes −α_R P_cond per FET node. This term is the electrothermal coupling.

**Efficient 2.5D layer-shell alternative.** Model each copper layer i as a shell coupled to its neighbors:

t_i k_i ∇²_xy T_i + Σ_j g_ij (T_j − T_i) + q″_i − δ_i,surf h(T_i − T_∞) = 0

The interlayer coupling is g_ij = k_FR4 / t_dielectric, plus via conductance per unit area. Use quad shell elements at ~0.5 mm in-plane resolution, refined to 0.1 mm under the FET pads.

---

## 4. Steady-State Temperature Distribution (Analytical Check)

**(a) Board-average rise.** Two-sided exposed area A ≈ 3.2×10⁻³ m².

- R_board = 1/(h_eff A) ≈ 3.6 K/W
- ΔT_avg = 12 × 3.6 ≈ **43 K**, so T_board ≈ **78 °C**

**(b) Fin parameter (how isothermal the board is).**

- m = √(2h_eff / (k_xy t)) ≈ 39 m⁻¹
- 1/m ≈ 25 mm, which is comparable to the board size, so the board is moderately isothermal.

**(c) Local hotspot per channel.** Treat the channel as a disk source of radius a ≈ 4 mm on a fin:

- R_sp = K₀(ma) / [2π k_xy t · ma · K₁(ma)] ≈ 1.97 / 0.714 ≈ 2.8 K/W (infinite-fin value)
- Subtract the share already counted in the board average to get the local excess:
  ΔT_local ≈ P_ch(R_sp − R_board,ch) ≈ 2.8 × ~2.3 ≈ **+6 K**

**(d) Junction rise** for the hottest FET (~0.65 W):

- R_jc ≈ 1.2 K/W plus pad/via spreading ≈ 4 K/W
- ΔT_j ≈ **+3.5 K**

**Resulting steady-state profile (h = 80 W/m²K):**

| Location | Temperature |
|---|---|
| Board edges and corners | ~70 °C |
| Board centre (MCU) | ~78 °C |
| FET copper pour, each channel | ~84 °C |
| Hottest FET junction | **~88–92 °C** |

The four channels appear as four lobes, hottest at the inner corners where their fields superpose.

**Still-air case (h ≈ 17 W/m²K effective):** R_board ≈ 18 K/W, which predicts ΔT ≈ 200 K. Continuous 80 A without airflow is therefore not surv