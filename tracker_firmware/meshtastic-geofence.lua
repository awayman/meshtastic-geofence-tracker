--[[
  meshtastic-geofence.lua
  Meshtastic Geofence Tracker - Core Firmware Script

  Runs on: SenseCAP MeshTracker X1, T-Beam v1.1, WiseMesh Board One B1
  Purpose: Poll GPS position every 30s, check if inside polygon geofence,
           control 12V relay via GPIO, send/receive mesh messages to/from
           Home Assistant via the official Meshtastic HA integration.
           Provides progressive audio beep warnings as user approaches the
           geofence boundary.

  Message format received from HA (geofence_config):
    {
      "type": "geofence_config",
      "tracker_id": "1234",
      "person_name": "Alice",
      "geofence_name": "home_yard",
      "polygon": [[lat,lon], ...],
      "relay_gpio": 10,
      "enable_beeper": true,
      "buzzer_gpio": 14,
      "warn_dist_approaching": 50,
      "warn_dist_near": 20,
      "warn_dist_critical": 5,
      "version": 1
    }

  Message format sent to HA (status):
    {
      "device": "Tracker-1234",
      "status": "in_geofence",
      "latitude": 40.7128,
      "longitude": -74.0060,
      "dist_to_boundary": 12.3,
      "beep_zone": "NEAR",
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

    -- Buzzer/beeper support
    -- enable_beeper: master switch for audio warnings
    -- buzzer_gpio:   GPIO pin for external piezo (0 = use built-in buzzer API)
    enable_beeper   = false,
    buzzer_gpio     = 0,

    -- Distance thresholds for progressive beep warnings (metres)
    warn_dist_approaching = 50,   -- slow beep zone (inside, 20–50m from edge)
    warn_dist_near        = 20,   -- medium beep zone (inside, 5–20m from edge)
    warn_dist_critical    = 5,    -- fast beep zone (inside, 0–5m from edge)

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

    -- Distance to nearest polygon edge in metres (positive = inside, negative = outside).
    -- nil when no polygon configured or no GPS fix.
    dist_to_boundary = nil,

    -- Current beep zone: "SAFE", "APPROACHING", "NEAR", "CRITICAL", "OUTSIDE", or nil
    beep_zone       = nil,

    -- Relay status
    relay_enabled   = false,

    -- Battery
    battery_percent = 0,

    -- Uptime counter (incremented each periodic tick)
    tick_count      = 0,

    -- Status report interval in ticks (1 tick = ~30s → 10 ticks = 5 min)
    report_interval = 10,

    -- Timestamp of last beep sequence (seconds, from os.time())
    last_beep_time  = 0,
}

-- ---------------------------------------------------------------------------
-- CONSTANTS
-- ---------------------------------------------------------------------------

local POLL_INTERVAL_MS   = 30000   -- 30 seconds
local LOW_BATTERY_PCT    = 10      -- percent – relay OFF below this level
local RELAY_ON           = 1       -- GPIO HIGH = relay energised
local RELAY_OFF          = 0       -- GPIO LOW  = relay de-energised
local EPSILON            = 1e-9
local EARTH_RADIUS_M     = 6371000 -- mean Earth radius in metres
local BEEP_MIN_INTERVAL  = 2       -- minimum seconds between beep sequences

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
-- BEEPER CONTROL
-- ---------------------------------------------------------------------------
--
-- Beep pattern library.  Each pattern is a sequence of {on_ms, off_ms}
-- pairs to be played once per beeper_update invocation.
--
-- Patterns (from issue spec):
--   STARTUP     : 3 × 50 ms on / 100 ms off  – device ready
--   ERROR       : 3 × 200 ms on / 200 ms off – error occurred
--   APPROACHING : 3 × 100 ms on / 200 ms off – 20–50 m from edge
--   NEAR        : 4 × 75 ms on  / 150 ms off – 5–20 m from edge
--   CRITICAL    : 5 × 50 ms on  / 100 ms off – 0–5 m from edge
--   OUTSIDE     : 1 × 500 ms on / 500 ms off – OUTSIDE GEOFENCE
--   LOW_BATTERY : 2 × 500 ms on / 500 ms off – battery critical
--   SAFE        : (empty) – silent; safely inside and far from edge
--

local BEEP_PATTERNS = {
    STARTUP     = { {50,100}, {50,100}, {50,100} },
    ERROR       = { {200,200}, {200,200}, {200,200} },
    APPROACHING = { {100,200}, {100,200}, {100,200} },
    NEAR        = { {75,150}, {75,150}, {75,150}, {75,150} },
    CRITICAL    = { {50,100}, {50,100}, {50,100}, {50,100}, {50,100} },
    OUTSIDE     = { {500,500} },
    LOW_BATTERY = { {500,500}, {500,500} },
    SAFE        = {},
}

-- Busy-wait helper (Meshtastic Lua may not expose non-blocking sleep)
local function busy_wait_ms(ms)
    local t0 = os.clock()
    local target = ms / 1000
    while os.clock() - t0 < target do end
end

-- Drive a single beep pulse via GPIO (external piezo) or built-in buzzer API.
local function beep_pulse(on_ms)
    if CONFIG.buzzer_gpio ~= 0 then
        -- GPIO-driven piezo buzzer
        pcall(function()
            gpio.write(CONFIG.buzzer_gpio, 1)
            busy_wait_ms(on_ms)
            gpio.write(CONFIG.buzzer_gpio, 0)
        end)
    else
        -- Built-in buzzer API (SenseCAP X1 and compatible)
        pcall(function()
            buzzer.beep(on_ms)
        end)
    end
end

-- Play an entire beep pattern (blocking during the sequence).
local function play_pattern(pattern)
    if not CONFIG.enable_beeper then return end
    for _, pulse in ipairs(pattern) do
        local on_ms  = pulse[1]
        local off_ms = pulse[2]
        beep_pulse(on_ms)
        if off_ms > 0 then
            busy_wait_ms(off_ms)
        end
    end
end

-- Named helpers used from the lifecycle functions.
local function play_startup()   play_pattern(BEEP_PATTERNS.STARTUP)   end
local function play_error()     play_pattern(BEEP_PATTERNS.ERROR)     end
local function play_low_batt()  play_pattern(BEEP_PATTERNS.LOW_BATTERY) end

-- Determine the beep zone string from a signed distance value.
--   dist_m > 0  : inside  (positive = metres to nearest edge from inside)
--   dist_m <= 0 : outside (negative or zero)
--   dist_m nil  : unknown (no GPS / no polygon)
local function zone_from_dist(dist_m)
    if dist_m == nil then return nil end
    if dist_m <= 0 then return "OUTSIDE" end
    if dist_m <= CONFIG.warn_dist_critical    then return "CRITICAL"    end
    if dist_m <= CONFIG.warn_dist_near        then return "NEAR"        end
    if dist_m <= CONFIG.warn_dist_approaching then return "APPROACHING" end
    return "SAFE"
end

-- Called each periodic tick to play the appropriate beep pattern once,
-- respecting the minimum interval between sequences to prevent spam.
local function beeper_update(dist_m)
    local zone = zone_from_dist(dist_m)
    STATE.beep_zone = zone

    if not CONFIG.enable_beeper then return end
    if zone == nil or zone == "SAFE" then return end

    -- Enforce minimum interval between sequences
    local now = os.time()
    if (now - STATE.last_beep_time) < BEEP_MIN_INTERVAL then
        log_debug("Beeper suppressed (min interval not elapsed)")
        return
    end

    local pattern = BEEP_PATTERNS[zone]
    if pattern and #pattern > 0 then
        log_info("Beeper: playing pattern " .. zone ..
                 " (dist=" .. (dist_m ~= nil and string.format("%.1f", dist_m) or "?") .. "m)")
        play_pattern(pattern)
        STATE.last_beep_time = os.time()
    end
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
-- DISTANCE TO POLYGON BOUNDARY
-- ---------------------------------------------------------------------------
--
-- Converts a lat/lon offset to approximate metres using a flat-Earth
-- (equirectangular) projection.  Accurate to ±5 m for distances < 1 km.
--
-- Returns the shortest distance in metres from point (px, py) to the line
-- segment from (x1, y1) to (x2, y2), where all coordinates are in degrees.
--
local function deg_to_rad(d)
    return d * math.pi / 180.0
end

local function point_to_segment_dist_m(px, py, x1, y1, x2, y2)
    -- Convert lat/lon deltas to approximate metres.
    -- Use the midpoint latitude for the longitude-to-metres scale factor.
    local mid_lat = (x1 + x2) / 2.0
    local cos_lat = math.cos(deg_to_rad(mid_lat))
    local m_per_deg_lat = EARTH_RADIUS_M * math.pi / 180.0
    local m_per_deg_lon = m_per_deg_lat * cos_lat

    -- Project everything into a local (x_m, y_m) plane relative to (x1, y1)
    local ax = (x2 - x1) * m_per_deg_lat
    local ay = (y2 - y1) * m_per_deg_lon
    local bx = (px - x1) * m_per_deg_lat
    local by = (py - y1) * m_per_deg_lon

    local seg_len_sq = ax * ax + ay * ay

    local dist_m
    if seg_len_sq < EPSILON then
        -- Degenerate segment (zero length): distance to endpoint
        dist_m = math.sqrt(bx * bx + by * by)
    else
        -- Project b onto the segment, clamp to [0,1]
        local t = (bx * ax + by * ay) / seg_len_sq
        if t < 0.0 then t = 0.0 end
        if t > 1.0 then t = 1.0 end
        local dx = bx - t * ax
        local dy = by - t * ay
        dist_m = math.sqrt(dx * dx + dy * dy)
    end

    return dist_m
end

-- Returns a signed distance in metres from (px, py) to the nearest polygon edge:
--   positive  → inside geofence   (distance to the closest edge)
--   negative  → outside geofence  (negated distance to the closest edge)
--   nil       → polygon has fewer than 3 vertices
--
local function dist_to_boundary_m(px, py, polygon)
    local n = #polygon
    if n < 3 then return nil end

    local min_dist = math.huge
    local j = n
    for i = 1, n do
        local d = point_to_segment_dist_m(
            px, py,
            polygon[i][1], polygon[i][2],
            polygon[j][1], polygon[j][2]
        )
        if d < min_dist then
            min_dist = d
        end
        j = i
    end

    -- Sign: positive if inside, negative if outside
    local inside = point_in_polygon(px, py, polygon)
    return inside and min_dist or -min_dist
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
        STATE.dist_to_boundary = nil
        STATE.beep_zone = nil
        return
    end

    -- No GPS fix → disable relay (safe state)
    if not STATE.gps_fix then
        log_warn("No GPS fix – relay OFF (safe)")
        if STATE.relay_enabled then
            relay_safe_off()
        end
        STATE.dist_to_boundary = nil
        STATE.beep_zone = nil
        return
    end

    -- Low battery → disable relay (safe state) and warn
    if STATE.battery_percent < LOW_BATTERY_PCT then
        log_warn("Low battery (" .. STATE.battery_percent .. "%) – relay OFF (safe)")
        if STATE.relay_enabled then
            relay_safe_off()
        end
        -- Play low-battery beep pattern (subject to spam guard)
        local now = os.time()
        if (now - STATE.last_beep_time) >= BEEP_MIN_INTERVAL then
            play_low_batt()
            STATE.last_beep_time = os.time()
        end
        STATE.beep_zone = nil
        STATE.dist_to_boundary = nil
        return
    end

    -- Calculate signed distance to boundary
    local dist_m = dist_to_boundary_m(STATE.latitude, STATE.longitude, CONFIG.polygon)
    STATE.dist_to_boundary = dist_m

    -- Derive inside/outside from sign of distance
    local inside = (dist_m ~= nil) and (dist_m > 0)

    -- Detect state change for logging
    local changed = (inside ~= STATE.inside_geofence)
    STATE.inside_geofence = inside

    if changed then
        log_info("Geofence status CHANGED → " .. (inside and "INSIDE" or "OUTSIDE"))
    end

    if dist_m ~= nil then
        log_debug(string.format("dist_to_boundary=%.1fm  inside=%s", dist_m, tostring(inside)))
    else
        log_debug("dist_to_boundary=nil  inside=" .. tostring(inside))
    end

    -- Relay logic: ON when inside geofence, OFF when outside
    if inside ~= STATE.relay_enabled then
        relay_set(inside)
    end

    -- Progressive audio warning
    beeper_update(dist_m)
end

-- ---------------------------------------------------------------------------
-- STATUS REPORTING (sends JSON to mesh → HA via Meshtastic integration)
-- ---------------------------------------------------------------------------

local function send_status()
    local status_str = STATE.inside_geofence and "in_geofence" or "outside_geofence"

    -- Format optional distance field (omit when unknown)
    local dist_field = ""
    if STATE.dist_to_boundary ~= nil then
        dist_field = string.format(',"dist_to_boundary":%.1f', STATE.dist_to_boundary)
    end

    local zone_field = ""
    if STATE.beep_zone ~= nil then
        zone_field = string.format(',"beep_zone":"%s"', STATE.beep_zone)
    end

    -- Build JSON manually (no json library guaranteed in Meshtastic Lua)
    local msg = string.format(
        '{"device":"Tracker-%s","status":"%s","latitude":%.6f,"longitude":%.6f' ..
        '%s%s,"relay_enabled":%s,"battery_percent":%d,"gps_fix":%s,"timestamp":%d}',
        CONFIG.tracker_id,
        status_str,
        STATE.latitude,
        STATE.longitude,
        dist_field,
        zone_field,
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
    CONFIG.buzzer_gpio    = json_num(payload, "buzzer_gpio")     or CONFIG.buzzer_gpio
    CONFIG.warn_dist_approaching = json_num(payload, "warn_dist_approaching") or CONFIG.warn_dist_approaching
    CONFIG.warn_dist_near        = json_num(payload, "warn_dist_near")        or CONFIG.warn_dist_near
    CONFIG.warn_dist_critical    = json_num(payload, "warn_dist_critical")    or CONFIG.warn_dist_critical
    CONFIG.version        = new_version or (CONFIG.version + 1)

    log_info(string.format(
        "Config applied: person=%s, geofence=%s, vertices=%d, gpio=%d, beeper=%s, buzzer_gpio=%d",
        CONFIG.person_name,
        CONFIG.geofence_name,
        #CONFIG.polygon,
        CONFIG.relay_gpio,
        tostring(CONFIG.enable_beeper),
        CONFIG.buzzer_gpio
    ))
    log_info(string.format(
        "Beep thresholds: approaching=%dm, near=%dm, critical=%dm",
        CONFIG.warn_dist_approaching,
        CONFIG.warn_dist_near,
        CONFIG.warn_dist_critical
    ))

    -- Confirm config receipt with startup pattern
    play_startup()

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

    -- Startup beep: 3 short beeps = device ready
    play_startup()

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
