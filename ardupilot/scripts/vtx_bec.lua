--[[
  Nano H743 AIO - digital VTX BEC 9 V / 12 V interlock (Revision 13.0)
  Sreesai Arjun Engineering

  Hardware (ardupilot/hwdef.dat, Rev 13.0 BEC block):
    U6 TPS62933DRLR buck, 500 kHz, VBAT -> +VTX (J1 pin 1 / J2 pads).
    VTX_PWR     PD10 GPIO(81) = RELAY2 -> R39 1k -> U6 EN (internal pull-up:
                high-Z = BEC on).  Lua relay instance 1.
    VTX_12V_SEL PD11 GPIO(82) = RELAY3 -> R37 1k -> Q1 2N7002 gate (R38 100k
                pull-down: high-Z / LOW = 8.96 V, HIGH = 12.02 V).
                Lua relay instance 2.
    VTX_VSENSE  PC5 ADC12_INP8 <- R40 10k / R41 1k (x11.0) = battery monitor
                2 (BATT2_MONITOR 1, BATT2_VOLT_PIN 8, BATT2_VOLT_MULT 11.0).
                Lua battery instance 1.

  Relay / battery instance numbering: the scripting bindings pass the
  instance straight to AP_Relay::on/off/enabled/get(uint8_t instance) and
  AP_BattMonitor::voltage/healthy(uint8_t instance), which index _params[] /
  state[] from 0 (AP_Scripting/generator/description/bindings.desc:306-310,
  :114-115; AP_Relay.cpp:420-431).  RELAY1 = 0, so RELAY2 = 1, RELAY3 = 2;
  BATT = 0, BATT2 = 1.

  Policy (identical to firmware/vtx_power, frozen spec D2):
    * 12 V is requested by VTXB_12V_REQ = 1, or by an RC switch in the high
      position on the auxiliary function VTXB_RC_FUNC (300..307 =
      Scripting1..8; 0 = parameter only).
    * 12 V is granted only after battery 0 (VBAT) has been >= VTXB_V12_MIN
      (13.6 V, 4S: 3.4 V/cell) continuously for 2 s AND battery 1 (+VTX
      sense) agrees with the 9 V rail (8.96 V +/- VTXB_RAIL_TOL) over the
      same 2 s.  Below ~13.3 V at 2 A the TPS62933 cannot regulate 12.02 V
      (Dmax 0.93, analysis/osd_vtx_bec_analysis.py dropout table), so 12 V
      is a 4S-only mode.
    * In 12 V: VBAT < VTXB_V12_DROP (13.0 V) for 0.3 s, request removed, a
      battery monitor going unhealthy, or +VTX outside the 12 V window
      (after a 0.5 s settle) for 0.3 s -> back to 9 V.  A rail fault latches
      the 12 V refusal until the request is removed and re-asserted.
    * VTX_PWR is never switched off while armed.  VTXB_PWR = 0 switches the
      VTX off only while disarmed (bench cooling); arming turns it back on.
    * Any external change of RELAY2/RELAY3 (GCS, RC "Relay2/3 On/Off" aux
      functions) that violates the policy is reverted on the next 100 ms
      tick and reported.

  Install: copy to APM/scripts/ on the microSD card, set SCR_ENABLE 1,
  reboot.  Requires RELAY2_FUNCTION / RELAY3_FUNCTION = 1 (Relay); AP_Relay
  converts them automatically on first boot because hwdef.dat sets
  RELAY2/3_PIN_DEFAULT (AP_Relay.cpp convert_params()).
--]]

local SCRIPT = "VTXB"
local UPDATE_MS = 100

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- hardware constants (analysis/osd_vtx_bec_analysis.py, frozen spec B)
local RELAY_VTX_PWR = 1        -- RELAY2 (0-based binding index)
local RELAY_VTX_12V = 2        -- RELAY3
local GPIO_VTX_PWR = 81        -- hwdef.dat PD10 GPIO(81) = RELAY2_PIN_DEFAULT
local GPIO_VTX_12V = 82        -- hwdef.dat PD11 GPIO(82) = RELAY3_PIN_DEFAULT
local BATT_VBAT = 0            -- BATT  (main battery, PC0)
local BATT_VTX = 1             -- BATT2 (+VTX sense, PC5)
local V_RAIL_9 = 8.96          -- 0.8 V * (1 + 102k / 10k)
local V_RAIL_12 = 12.02        -- 0.8 V * (1 + 102k / (10k || 26.7k))
local DROPOUT_HEADROOM = 0.4   -- VBAT - Vout at 2 A in dropout (firmware/vtx_power)

-- timing constants (frozen spec D2)
local QUALIFY_MS = 2000        -- VBAT >= V12_MIN and rail OK this long before 12 V
local DROP_MS = 300            -- VBAT < V12_DROP this long -> 9 V
local SETTLE_MS = 500          -- after 9 -> 12 V, before the rail window applies
local RAIL_FAULT_MS = 300      -- rail outside window this long -> 9 V + latch

local PARAM_TABLE_KEY = 143
local PARAM_TABLE_PREFIX = "VTXB_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("%s: could not add param %s", SCRIPT, name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7),
       string.format("%s: could not add param table", SCRIPT))

--[[
  // @Param: VTXB_ENABLE
  // @DisplayName: VTX BEC interlock enable
  // @Description: Enables the 9 V / 12 V VTX BEC interlock script
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local VTXB_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: VTXB_12V_REQ
  // @DisplayName: VTX 12 V request
  // @Description: Requests the 12 V VTX rail. Granted only on 4S (see VTXB_V12_MIN). HDZero VTXs are rated 12 V max: leave at 0 for them.
  // @Values: 0:9V,1:12V
  // @User: Standard
--]]
local VTXB_12V_REQ = bind_add_param("12V_REQ", 2, 0)

--[[
  // @Param: VTXB_RC_FUNC
  // @DisplayName: VTX 12 V request RC aux function
  // @Description: RCx_OPTION scripting function whose high position requests 12 V (OR-ed with VTXB_12V_REQ). 0 disables the RC request.
  // @Values: 0:Disabled,300:Scripting1,301:Scripting2,302:Scripting3,303:Scripting4,304:Scripting5,305:Scripting6,306:Scripting7,307:Scripting8
  // @User: Standard
--]]
local VTXB_RC_FUNC = bind_add_param("RC_FUNC", 3, 0)

--[[
  // @Param: VTXB_V12_MIN
  // @DisplayName: VBAT required for 12 V
  // @Description: Battery voltage that must be held for 2 s before 12 V is granted
  // @Units: V
  // @Range: 13.3 25.2
  // @User: Advanced
--]]
local VTXB_V12_MIN = bind_add_param("V12_MIN", 4, 13.6)

--[[
  // @Param: VTXB_V12_DROP
  // @DisplayName: VBAT that forces 9 V
  // @Description: Battery voltage below which (for 0.3 s) the rail returns to 9 V
  // @Units: V
  // @Range: 12.5 25.2
  // @User: Advanced
--]]
local VTXB_V12_DROP = bind_add_param("V12_DROP", 5, 13.0)

--[[
  // @Param: VTXB_RAIL_TOL
  // @DisplayName: VTX rail tolerance
  // @Description: Allowed relative error of the BATT2 (+VTX) reading against the expected rail voltage
  // @Range: 0.03 0.2
  // @User: Advanced
--]]
local VTXB_RAIL_TOL = bind_add_param("RAIL_TOL", 6, 0.06)

--[[
  // @Param: VTXB_PWR
  // @DisplayName: VTX power while disarmed
  // @Description: 0 switches the VTX off while disarmed (bench cooling). The VTX is always powered while armed.
  // @Values: 0:Off when disarmed,1:On
  // @User: Standard
--]]
local VTXB_PWR = bind_add_param("PWR", 7, 1)

-- state ---------------------------------------------------------------------
local mode_12v = false          -- commanded rail
local fault_latched = false     -- rail fault: 12 V refused until request cycles
local qualify_start = nil       -- millis() when the 12 V preconditions became true
local drop_start = nil          -- millis() when VBAT first fell below V12_DROP
local rail_bad_start = nil      -- millis() when +VTX first left the 12 V window
local mode_change_ms = nil      -- millis() of the last 9 <-> 12 V change
local last_request = false
local init_done = false

local function elapsed_ms(t0)
    return (millis() - t0):toint()
end

local function say(sev, msg)
    gcs:send_text(sev, string.format("%s: %s", SCRIPT, msg))
end

local function battery_ok(instance)
    return battery:num_instances() > instance and battery:healthy(instance)
end

-- expected +VTX for the commanded rail, including buck dropout at low VBAT
local function expected_rail(vbat, want_12v)
    local target = want_12v and V_RAIL_12 or V_RAIL_9
    return math.min(target, vbat - DROPOUT_HEADROOM)
end

local function rail_in_window(vrail, expected)
    return math.abs(vrail - expected) <= VTXB_RAIL_TOL:get() * expected
end

local function rc_request()
    local fn = math.floor(VTXB_RC_FUNC:get())
    if fn < 300 or fn > 307 then
        return false
    end
    local pos = rc:get_aux_cached(fn)
    return pos ~= nil and pos == 2
end

local function set_12v(on, why)
    if on then
        relay:on(RELAY_VTX_12V)
    else
        relay:off(RELAY_VTX_12V)
    end
    if on ~= mode_12v then
        mode_12v = on
        mode_change_ms = millis()
        rail_bad_start = nil
        say(on and MAV_SEVERITY.NOTICE or MAV_SEVERITY.WARNING,
            string.format("VTX rail %s (%s)", on and "12V" or "9V", why))
    end
end

local function init()
    if not relay:enabled(RELAY_VTX_PWR) or not relay:enabled(RELAY_VTX_12V)
       or param:get("RELAY2_PIN") ~= GPIO_VTX_PWR or param:get("RELAY3_PIN") ~= GPIO_VTX_12V then
        say(MAV_SEVERITY.ERROR, "RELAY2/RELAY3 must be FUNCTION 1 on PIN 81/82")
        return false
    end
    -- boot state: the BEC must be on (RELAY2_DEFAULT 1) and at 9 V (RELAY3_DEFAULT 0)
    relay:on(RELAY_VTX_PWR)
    relay:off(RELAY_VTX_12V)
    mode_12v = false
    say(MAV_SEVERITY.INFO, "VTX BEC interlock active, rail 9V")
    return true
end

local function update()
    if VTXB_ENABLE:get() < 1 then
        return update, 1000
    end
    if not init_done then
        init_done = init()
        if not init_done then
            return update, 5000
        end
    end

    local armed = arming:is_armed()

    -- VTX_PWR: never off while armed
    local want_pwr = armed or VTXB_PWR:get() >= 1
    if want_pwr then
        if relay:get(RELAY_VTX_PWR) ~= 1 then
            relay:on(RELAY_VTX_PWR)
            say(armed and MAV_SEVERITY.WARNING or MAV_SEVERITY.INFO,
                armed and "VTX power forced on (armed)" or "VTX power on")
        end
    elseif relay:get(RELAY_VTX_PWR) ~= 0 then
        relay:off(RELAY_VTX_PWR)
        say(MAV_SEVERITY.INFO, "VTX power off (disarmed, VTXB_PWR 0)")
    end

    local request = (VTXB_12V_REQ:get() >= 1) or rc_request()
    if not request and fault_latched then
        fault_latched = false
        say(MAV_SEVERITY.INFO, "12V fault latch cleared")
    end
    if request and not last_request and fault_latched then
        say(MAV_SEVERITY.WARNING, "12V refused: rail fault latched, cycle the request")
    end
    last_request = request

    local vbat_ok = battery_ok(BATT_VBAT)
    local sense_ok = battery_ok(BATT_VTX)
    local vbat = vbat_ok and battery:voltage(BATT_VBAT) or 0
    local vrail = sense_ok and battery:voltage(BATT_VTX) or 0
    local powered = want_pwr

    if not mode_12v then
        -- 9 V: qualify the 12 V preconditions continuously for QUALIFY_MS
        local pre = request and powered and not fault_latched and vbat_ok and sense_ok
                    and vbat >= VTXB_V12_MIN:get()
                    and rail_in_window(vrail, expected_rail(vbat, false))
        if pre then
            if qualify_start == nil then
                qualify_start = millis()
            elseif elapsed_ms(qualify_start) >= QUALIFY_MS then
                qualify_start = nil
                drop_start = nil
                set_12v(true, string.format("VBAT %.2fV", vbat))
            end
        else
            qualify_start = nil
        end
        -- revert anything else that raised RELAY3 (GCS / RC relay switch)
        if not mode_12v and relay:get(RELAY_VTX_12V) ~= 0 then
            relay:off(RELAY_VTX_12V)
            say(MAV_SEVERITY.WARNING, "external RELAY3 on reverted: 12V interlock not met")
        end
    else
        -- 12 V: hold conditions
        if not request then
            set_12v(false, "request removed")
        elseif not powered then
            set_12v(false, "VTX power off")
        elseif not vbat_ok or not sense_ok then
            set_12v(false, "battery monitor unhealthy")
        else
            if vbat < VTXB_V12_DROP:get() then
                if drop_start == nil then
                    drop_start = millis()
                elseif elapsed_ms(drop_start) >= DROP_MS then
                    drop_start = nil
                    set_12v(false, string.format("VBAT %.2fV low", vbat))
                end
            else
                drop_start = nil
            end
            if mode_12v and elapsed_ms(mode_change_ms) >= SETTLE_MS then
                local exp = expected_rail(vbat, true)
                if not rail_in_window(vrail, exp) then
                    if rail_bad_start == nil then
                        rail_bad_start = millis()
                    elseif elapsed_ms(rail_bad_start) >= RAIL_FAULT_MS then
                        fault_latched = true
                        set_12v(false, string.format("rail fault %.2fV exp %.2fV", vrail, exp))
                    end
                else
                    rail_bad_start = nil
                end
            end
        end
        -- re-assert RELAY3 if something else lowered it
        if mode_12v and relay:get(RELAY_VTX_12V) ~= 1 then
            relay:on(RELAY_VTX_12V)
        end
    end

    return update, UPDATE_MS
end

return update, 1000
