--[[
  config_example.lua
  Example static configuration for meshtastic-geofence.lua

  Copy the relevant block into meshtastic-geofence.lua's CONFIG table
  to pre-configure a device without waiting for a geofence_config
  message from Home Assistant.
]]

-- ---------------------------------------------------------------------------
-- EXAMPLE: Home Yard Polygon (rectangular, ~30m × 30m)
-- ---------------------------------------------------------------------------

--[[
CONFIG = {
    tracker_id     = "a1b2",          -- Last 4 hex digits of device node ID
    person_name    = "Alice",
    geofence_name  = "home_yard",

    -- Rectangle centred on 40.7131, -74.0057
    polygon = {
        {40.7128, -74.0060},
        {40.7135, -74.0060},
        {40.7135, -74.0055},
        {40.7128, -74.0055},
    },

    relay_gpio    = 10,   -- GPIO pin controlling the 12V relay

    -- Beeper configuration
    enable_beeper = true, -- SenseCAP X1 has a built-in buzzer
    buzzer_gpio   = 0,    -- 0 = use built-in buzzer API; set GPIO pin for external piezo

    -- Distance thresholds for progressive beep warnings (metres)
    warn_dist_approaching = 50,   -- slow beep zone (inside, 20–50 m from edge)
    warn_dist_near        = 20,   -- medium beep zone (inside, 5–20 m from edge)
    warn_dist_critical    = 5,    -- fast beep zone (inside, 0–5 m from edge)

    version       = 1,
}
]]

-- ---------------------------------------------------------------------------
-- EXAMPLE: Larger Park Polygon (irregular shape)
-- ---------------------------------------------------------------------------

--[[
CONFIG = {
    tracker_id     = "c3d4",
    person_name    = "Bob",
    geofence_name  = "central_park_south",

    polygon = {
        {40.7678, -73.9812},
        {40.7685, -73.9790},
        {40.7680, -73.9770},
        {40.7660, -73.9775},
        {40.7655, -73.9800},
        {40.7665, -73.9820},
    },

    relay_gpio    = 10,

    -- Beeper disabled for this example
    enable_beeper = false,
    buzzer_gpio   = 0,

    warn_dist_approaching = 50,
    warn_dist_near        = 20,
    warn_dist_critical    = 5,

    version       = 1,
}
]]

-- ---------------------------------------------------------------------------
-- EXAMPLE: T-Beam with external piezo buzzer on GPIO 14
-- ---------------------------------------------------------------------------

--[[
CONFIG = {
    tracker_id     = "e5f6",
    person_name    = "Charlie",
    geofence_name  = "construction_zone",

    polygon = {
        {51.5010, -0.1420},
        {51.5025, -0.1420},
        {51.5025, -0.1390},
        {51.5010, -0.1390},
    },

    relay_gpio    = 4,    -- T-Beam relay GPIO

    -- External piezo on GPIO 14 (connect via 100Ω current-limiting resistor)
    enable_beeper = true,
    buzzer_gpio   = 14,

    warn_dist_approaching = 50,
    warn_dist_near        = 20,
    warn_dist_critical    = 5,

    version       = 1,
}
]]

-- ---------------------------------------------------------------------------
-- RELAY GPIO PIN REFERENCE
-- ---------------------------------------------------------------------------
--[[
  Hardware         | Suggested relay GPIO
  -----------------|---------------------
  SenseCAP X1      | 10 (default)
  T-Beam v1.1      | 4  (check schematics)
  WiseMesh B1      | 12 (check schematics)

  All relay outputs are active-HIGH (GPIO HIGH = relay energised = relay ON).
  A flyback diode on the relay coil is strongly recommended.
]]

-- ---------------------------------------------------------------------------
-- BUZZER GPIO REFERENCE
-- ---------------------------------------------------------------------------
--[[
  Hardware         | buzzer_gpio | Notes
  -----------------|-------------|------------------------------------------
  SenseCAP X1      | 0           | Built-in buzzer, uses buzzer.beep() API
  T-Beam v1.1      | 12 or 13    | External piezo via 100Ω resistor
  WiseMesh B1      | 14          | External piezo via 100Ω resistor
  Any board        | 0–39        | Any free GPIO; 100Ω resistor recommended

  buzzer_gpio = 0 means use the built-in buzzer.beep() API (SenseCAP X1).
  Any non-zero value drives that GPIO pin directly (active-HIGH).
]]
