--[[
  meshtastic-geofence.lua
  Meshtastic Geofence Tracker - Core Firmware Script

  Runs on: SenseCAP MeshTracker X1, T-Beam v1.1, WiseMesh Board One B1
  Purpose: Poll GPS position every 30s, check if inside polygon geofence,
           control 12V relay via GPIO, send/receive mesh messages to/from
           Home Assistant via the official Meshtastic HA integration.

  Message format received from HA (geofence_config):
    {
      "type": "geofence_config",
      "tracker_id": "1234",
      "person_name": "Alice",
      "geofence_name": "home_yard",
      "polygon": [[lat,lon], ...],
      "relay_gpio": 10,
      "enable_beeper": true,
      "version": 1
    }

  Message format sent to HA (status):
    {
      "device": "Tracker-1234",
      "status": "in_geofence",
      "latitude": 40.7128,
      "longitude": -74.0060,
      "relay_enabled": true,
      "battery_percent": 85,
      "gps_fix": true,
      "timestamp": 1234567890
    }
]]

-- ---------------------------------------------------------------------------
-- CONFIG DEFAULTS (overridden by geofence_config messages from HA)
-- ---------------------------------------------------------------------------

local CONFIG = {
    -- Device identity (last 4 hex digits of device MAC / node ID)
    tracker_id      = "0000",
    person_name     = "unknown",
    geofence_name   = "none",

    -- Polygon vertices [[lat, lon], ...].  Empty = geofencing disabled.
    polygon         = {},

    -- GPIO pin number for the 12V relay (active-HIGH).
    -- Override via geofence_config message or set here for static config.
    relay_gpio      = 10,

    -- Buzzer/beeper support (SenseCAP X1 has built-in buzzer)
    enable_beeper   = false,

    -- Config version – used to detect stale configs
    version         = 0,
}

-- ---------------------------------------------------------------------------
-- RUNTIME STATE
-- ---------------------------------------------------------------------------

local STATE = {
    -- Current GPS fix
    latitude        = 0.0,
    longitude       = 0.0,
    gps_fix         = false,

    -- Geofence status
    inside_geofence = false,

    -- Relay status
    relay_enabled   = false,

    -- Battery
    battery_percent = 0,

    -- Uptime counter (incremented each periodic tick)
    tick_count      = 0,

    -- Status report interval in ticks (1 tick = ~30s → 10 ticks = 5 min)
    report_interval = 10,
}

-- ---------------------------------------------------------------------------
-- CONSTANTS
-- ---------------------------------------------------------------------------

local POLL_INTERVAL_MS   = 30000   -- 30 seconds
local LOW_BATTERY_PCT    = 10      -- percent – relay OFF below this level
local RELAY_ON           = 1       -- GPIO HIGH = relay energised
local RELAY_OFF          = 0       -- GPIO LOW  = relay de-energised
local EPSILON            = 1e-9

-- ---------------------------------------------------------------------------
-- LOGGING HELPERS
-- ---------------------------------------------------------------------------

local function log_info(msg)
    print("[GeoFence][INFO ] " .. tostring(msg))
end

local function log_warn(msg)
    print("[GeoFence][WARN ] " .. tostring(msg))
end

local function log_err(msg)
    print("[GeoFence][ERROR] " .. tostring(msg))
end

local function log_debug(msg)
    print("[GeoFence][DEBUG] " .. tostring(msg))
end

-- ---------------------------------------------------------------------------
-- RELAY CONTROL
-- ---------------------------------------------------------------------------

local function relay_set(on)
    local level = on and RELAY_ON or RELAY_OFF
    local ok, err = pcall(function()
        -- Meshtastic Lua: gpio.write(pin, level)
        gpio.write(CONFIG.relay_gpio, level)
    end)
    if ok then
        STATE.relay_enabled = on
        log_info("Relay GPIO " .. CONFIG.relay_gpio .. " → " .. (on and "ON" or "OFF"))
    else
        log_err("gpio.write failed: " .. tostring(err))
    end
end

local function relay_safe_off()
    relay_set(false)
end

-- ---------------------------------------------------------------------------
-- BEEPER CONTROL  (SenseCAP X1 built-in buzzer)
-- ---------------------------------------------------------------------------

local function beep(duration_ms)
    if not CONFIG.enable_beeper then return end
    local ok, err = pcall(function()
        -- Meshtastic Lua: buzzer.beep(duration_ms)
        buzzer.beep(duration_ms)
    end)
    if not ok then
        log_debug("buzzer.beep not available: " .. tostring(err))
    end
end

-- Single short confirmation beep
local function beep_confirm()
    beep(100)
end

-- Double beep for geofence entry/exit events
local function beep_event()
    beep(200)
    -- small pause then second beep
    local t0 = os.time()
    while os.time() - t0 < 1 do end
    beep(200)
end

-- ---------------------------------------------------------------------------
-- POINT-IN-POLYGON  (Ray-casting algorithm)
-- ---------------------------------------------------------------------------
--
-- Returns true if point (px, py) is strictly inside the polygon defined by
-- the array of {lat, lon} vertex pairs.
--
-- Algorithm: cast a horizontal ray to the right from (px, py) and count how
-- many polygon edges it crosses.  Odd count → inside; even count → outside.
--
-- Edge cases handled:
--   • Polygon with fewer than 3 vertices → always outside.
--   • Point exactly on an edge → treated as outside (conservative / safe).
--

local function point_on_segment(px, py, xi, yi, xj, yj)
    local cross = (px - xi) * (yj - yi) - (py - yi) * (xj - xi)
    if math.abs(cross) > EPSILON then
        return false
    end

    local dot = (px - xi) * (px - xj) + (py - yi) * (py - yj)
    return dot <= EPSILON
end

local function point_in_polygon(px, py, polygon)
    local n = #polygon
    if n < 3 then
        log_warn("Polygon has fewer than 3 vertices – geofencing disabled")
        return false
    end

    local inside = false
    local j = n  -- previous vertex index (wraps around)

    for i = 1, n do
        local xi = polygon[i][1]
        local yi = polygon[i][2]
        local xj = polygon[j][1]
        local yj = polygon[j][2]

        if point_on_segment(px, py, xi, yi, xj, yj) then
            return false
        end

        -- Check if the ray from (px, py) heading right crosses edge (j→i)
        local crosses = (
            ((yi > py) ~= (yj > py)) and
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi)
        )
        if crosses then
            inside = not inside
        end

        j = i
    end

    return inside
end

-- ---------------------------------------------------------------------------
-- GPS POLLING
-- ---------------------------------------------------------------------------

local function update_gps()
    local ok, lat, lon, fix = pcall(function()
        -- Meshtastic Lua: gps.getPosition() returns lat, lon, fix_quality
        local lat, lon, fix = gps.getPosition()
        return lat, lon, fix
    end)

    if not ok then
        log_err("gps.getPosition() error: " .. tostring(lat))
        STATE.gps_fix = false
        return false
    end

    -- fix is typically 0 = no fix, 1 = GPS fix, 2 = DGPS fix, etc.
    STATE.gps_fix = (fix ~= nil) and (fix > 0)

    if STATE.gps_fix then
        STATE.latitude  = lat
        STATE.longitude = lon
        log_debug(string.format("GPS fix: %.6f, %.6f", lat, lon))
    else
        log_warn("No GPS fix")
    end

    return STATE.gps_fix
end

-- ---------------------------------------------------------------------------
-- BATTERY MONITORING
-- ---------------------------------------------------------------------------

local function update_battery()
    local ok, pct = pcall(function()
        -- Meshtastic Lua: power.getBatteryPercent()
        return power.getBatteryPercent()
    end)

    if ok and pct ~= nil then
        STATE.battery_percent = math.floor(pct)
        log_debug("Battery: " .. STATE.battery_percent .. "%")
    else
        log_warn("Could not read battery level")
        STATE.battery_percent = 0
    end
end

-- ---------------------------------------------------------------------------
-- GEOFENCE EVALUATION
-- ---------------------------------------------------------------------------

local function evaluate_geofence()
    -- No polygon configured → disable relay (safe state)
    if #CONFIG.polygon == 0 then
        log_warn("No geofence polygon configured – relay OFF (safe)")
        if STATE.relay_enabled then
            relay_safe_off()
        end
        return
    end

    -- No GPS fix → disable relay (safe state)
    if not STATE.gps_fix then
        log_warn("No GPS fix – relay OFF (safe)")
        if STATE.relay_enabled then
            relay_safe_off()
        end
        return
    end

    -- Low battery or unknown battery state → disable relay (safe state)
    if STATE.battery_percent < LOW_BATTERY_PCT then
        log_warn("Low battery (" .. STATE.battery_percent .. "%) – relay OFF (safe)")
        if STATE.relay_enabled then
            relay_safe_off()
        end
        return
    end

    -- Run point-in-polygon check
    local inside = point_in_polygon(STATE.latitude, STATE.longitude, CONFIG.polygon)

    -- Detect state change for beep event
    local changed = (inside ~= STATE.inside_geofence)
    STATE.inside_geofence = inside

    if changed then
        log_info("Geofence status CHANGED → " .. (inside and "INSIDE" or "OUTSIDE"))
        beep_event()
    end

    -- Relay logic: ON when inside geofence, OFF when outside
    if inside ~= STATE.relay_enabled then
        relay_set(inside)
    end
end

-- ---------------------------------------------------------------------------
-- STATUS REPORTING (sends JSON to mesh → HA via Meshtastic integration)
-- ---------------------------------------------------------------------------

local function send_status()
    local status_str = STATE.inside_geofence and "in_geofence" or "outside_geofence"

    -- Build JSON manually (no json library guaranteed in Meshtastic Lua)
    local msg = string.format(
        '{"device":"Tracker-%s","status":"%s","latitude":%.6f,"longitude":%.6f,' ..
        '"relay_enabled":%s,"battery_percent":%d,"gps_fix":%s,"timestamp":%d}',
        CONFIG.tracker_id,
        status_str,
        STATE.latitude,
        STATE.longitude,
        STATE.relay_enabled and "true" or "false",
        STATE.battery_percent,
        STATE.gps_fix and "true" or "false",
        os.time()
    )

    local ok, err = pcall(function()
        -- Meshtastic Lua: mesh.sendText(text, channel, wantAck)
        -- Channel 0 = primary channel (HA integration listens here by default)
        mesh.sendText(msg, 0, false)
    end)

    if ok then
        log_info("Status sent: " .. msg)
    else
        log_err("mesh.sendText failed: " .. tostring(err))
    end
end

-- ---------------------------------------------------------------------------
-- GEOFENCE CONFIG HANDLER  (receives JSON from HA via mesh)
-- ---------------------------------------------------------------------------

--[[
  Minimal JSON field extractor.
  Meshtastic Lua environments may not include a full JSON library, so we
  parse the fields we need with targeted pattern matching.

  Supported field types:
    - string:  "key":"value"
    - number:  "key":123  or  "key":123.456
    - boolean: "key":true  or  "key":false
    - array of [number, number] pairs for polygon

  Limitations:
    - Keys/values must not contain escaped quotes
    - Polygon values must be plain numbers
]]

local function json_str(text, key)
    local val = text:match('"' .. key .. '"%s*:%s*"([^"]*)"')
    return val
end

local function json_num(text, key)
    local val = text:match('"' .. key .. '"%s*:%s*(%-?%d+%.?%d*)')
    return val and tonumber(val) or nil
end

local function json_bool(text, key)
    local val = text:match('"' .. key .. '"%s*:%s*(true|false)')
    if val == nil then
        return nil
    end
    return val == "true"
end

local function json_polygon(text)
    -- Extract the polygon array string: [[...],[...],...]
    local arr_str = text:match('"polygon"%s*:%s*(%b[])')
    if not arr_str then
        return nil
    end

    local polygon = {}
    -- Each vertex is [lat,lon] possibly with spaces
    for lat_s, lon_s in arr_str:gmatch('%[%s*(%-?%d+%.?%d*)%s*,%s*(%-?%d+%.?%d*)%s*%]') do
        local lat = tonumber(lat_s)
        local lon = tonumber(lon_s)
        if lat and lon then
            table.insert(polygon, {lat, lon})
        end
    end

    return (#polygon >= 3) and polygon or nil
end

local function handle_geofence_config(payload)
    log_info("Received geofence_config message")
    log_debug("Payload: " .. payload)

    -- Verify message type
    local msg_type = json_str(payload, "type")
    if msg_type ~= "geofence_config" then
        log_warn("Ignoring message type: " .. tostring(msg_type))
        return
    end

    -- Verify this message is for us
    local tid = json_str(payload, "tracker_id")
    if tid and tid ~= CONFIG.tracker_id and tid ~= "all" then
        log_debug("Message for tracker_id " .. tid .. ", ignoring (we are " .. CONFIG.tracker_id .. ")")
        return
    end

    -- Reject stale configs
    local new_version = json_num(payload, "version")
    if new_version and new_version < CONFIG.version then
        log_warn("Ignoring stale config (version " .. new_version .. " < current " .. CONFIG.version .. ")")
        return
    end

    -- Parse polygon
    local polygon = json_polygon(payload)
    if not polygon then
        log_err("Invalid or missing polygon in config – ignoring")
        return
    end

    -- Apply new config
    CONFIG.person_name    = json_str(payload, "person_name")    or CONFIG.person_name
    CONFIG.geofence_name  = json_str(payload, "geofence_name")  or CONFIG.geofence_name
    CONFIG.polygon        = polygon
    CONFIG.relay_gpio     = json_num(payload, "relay_gpio")      or CONFIG.relay_gpio
    local enable_beeper = json_bool(payload, "enable_beeper")
    if enable_beeper ~= nil then
        CONFIG.enable_beeper = enable_beeper
    end
    CONFIG.version        = new_version or (CONFIG.version + 1)

    log_info(string.format(
        "Config applied: person=%s, geofence=%s, vertices=%d, gpio=%d, beeper=%s",
        CONFIG.person_name,
        CONFIG.geofence_name,
        #CONFIG.polygon,
        CONFIG.relay_gpio,
        tostring(CONFIG.enable_beeper)
    ))

    beep_confirm()

    -- Immediately re-evaluate with new polygon
    evaluate_geofence()
end

-- ---------------------------------------------------------------------------
-- MESH MESSAGE RECEIVE HANDLER
-- ---------------------------------------------------------------------------

local function on_receive_message(packet)
    local ok, err = pcall(function()
        if not packet then return end

        -- Only handle text messages (portnum 1 = TEXT_MESSAGE_APP)
        local portnum = packet.decoded and packet.decoded.portnum
        if portnum ~= 1 then return end

        local payload = packet.decoded and packet.decoded.text
        if not payload or payload == "" then return end

        log_debug("Received mesh message: " .. payload)

        -- Dispatch on message type field
        local msg_type = json_str(payload, "type")
        if msg_type == "geofence_config" then
            handle_geofence_config(payload)
        else
            log_debug("Unhandled message type: " .. tostring(msg_type))
        end
    end)

    if not ok then
        log_err("onReceive error: " .. tostring(err))
        pcall(relay_safe_off)
    end
end

-- ---------------------------------------------------------------------------
-- LIFECYCLE: onStart
-- ---------------------------------------------------------------------------

function onStart()
    log_info("=== Meshtastic Geofence Tracker starting ===")

    -- SAFETY FIRST: always boot with relay OFF
    relay_safe_off()

    -- Read device node ID for tracker_id (last 4 hex chars)
    local ok, node_id = pcall(function()
        return mesh.getMyNodeNum()
    end)
    if ok and node_id then
        CONFIG.tracker_id = string.format("%04x", node_id % 0x10000)
        log_info("Tracker ID: " .. CONFIG.tracker_id)
    else
        log_warn("Could not read node ID – using default tracker_id=" .. CONFIG.tracker_id)
    end

    -- Register mesh receive callback
    local cb_ok, cb_err = pcall(function()
        mesh.onReceive(on_receive_message)
    end)
    if not cb_ok then
        log_warn("mesh.onReceive registration failed: " .. tostring(cb_err))
    end

    -- Initial battery read
    update_battery()

    log_info("onStart complete – polling every " .. (POLL_INTERVAL_MS / 1000) .. "s")

    -- Return poll interval (milliseconds)
    return POLL_INTERVAL_MS
end

-- ---------------------------------------------------------------------------
-- LIFECYCLE: onPeriodic  (called every POLL_INTERVAL_MS)
-- ---------------------------------------------------------------------------

function onPeriodic()
    STATE.tick_count = STATE.tick_count + 1
    log_debug("Tick #" .. STATE.tick_count)

    -- Wrap all periodic work in pcall so a single error doesn't kill the loop
    local ok, err = pcall(function()
        -- 1. Update battery level
        update_battery()

        -- 2. Poll GPS
        update_gps()

        -- 3. Evaluate geofence & control relay
        evaluate_geofence()

        -- 4. Periodically send status to HA
        if STATE.tick_count % STATE.report_interval == 0 then
            send_status()
        end
    end)

    if not ok then
        log_err("onPeriodic error: " .. tostring(err))
        -- Ensure relay is in safe state after any error
        pcall(relay_safe_off)
    end

    -- Continue polling
    return POLL_INTERVAL_MS
end

-- ---------------------------------------------------------------------------
-- LIFECYCLE: onStop
-- ---------------------------------------------------------------------------

function onStop()
    log_info("=== Meshtastic Geofence Tracker stopping ===")

    -- SAFETY: ensure relay is OFF before halting
    pcall(relay_safe_off)

    log_info("Relay set to OFF – goodbye")
end
