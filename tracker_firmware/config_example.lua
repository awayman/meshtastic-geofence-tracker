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
    enable_beeper = true, -- SenseCAP X1 has a built-in buzzer
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
    enable_beeper = false,
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
