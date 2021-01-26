# [DS18x](https://github.com/milesburton/Arduino-Temperature-Control-Library) [Mongoose OS](https://mongoose-os.com/mos.html) [RPC](https://mongoose-os.com/docs/mongoose-os/userguide/rpc.md) service

The [RPC](https://mongoose-os.com/docs/mongoose-os/userguide/rpc.md) bindings
and configuration for [Mongoose OS](https://mongoose-os.com/mos.html)
[DallasTemperature](https://github.com/mongoose-os-libs/arduino-dallas-temperature)
library, itself a wrapper for [Miles Burton](https://github.com/milesburton)'s
Arduino Library for [Maxim Temperature Integrated
Circuits](https://github.com/milesburton/Arduino-Temperature-Control-Library)
(see its _README_ for details of supported sensor types, how to power the
sensors etc).

See also the excellent
[research](https://github.com/cpetrich/counterfeit_DS18B20) by [Chris
Petrich](https://github.com/cpetrich) on subtleties of DS18B20 clones.

## Configuration

**NB:** set `ds18x.pin` to point the
[wrapper](https://github.com/mongoose-os-libs/arduino-dallas-temperature)
library to the GPIO pin to which your sensors are connected.  Not doing that
implies `ds18x.rpc.enable=false`.

### `ds18x.rpc.enable`
 - _**true**_ Register the RPC methods upon restart
 - _false_ Disable this library

### `ds18x.rpc.raw_enable`
 - _**true**_ Register all the RPC methods
 - _false_ Omit registering `ReadScratchpad`, `RecallScratchpad` and
`WriteScratchpad`

## RPC methods

### Global operations

#### `DS18x.GetAlarms` — Get temperatures triggering alarms
Equivalent to `GetTemp` but returns temperature data only for those sensors the
alarm conditions set for which are met (see `GetDevInfo`).  The order of the
result is the same as that of the sensors responding on the 1-wire bus.

With `poll: false`, the measurements from the prior `GetAlarms`, `GetDevTemp`
or `GetTemp` operation or the power-on sensor state will be used, otherwise the
temperatures will be measured anew by all sensors before reevaluating the alarm
states and returning the data.  `GetInfo().check_for_conversion` and
`GetInfo().resolution` affect how long this can take.

_Arguments_ ([default]):
```yaml
{
  "poll": true                      # [true] False disables measuring temperatures anew.
}
```

_Result_ (example):
```yaml
[
  {
    "addr": "2818b4490c00007c",     # 1-wire address of 1st sensor raising alarm.
    "temp": {                       # Temperature last measured:
      "C": 20.312500,               # - as degrees Celsius;
      "F": 68.562500,               # - as degrees Fahrenheit;
      "raw": 2600                   # - the raw number from the sensor.
    }
  }
]
```

#### `DS18x.GetInfo` — Get global state and configuration
The state and configuration are reestablished upon device restarts and `Rescan`
calls.  The configuration can be changed directly via `SetConf` and indirectly
via `SetDevConf`.

_Arguments:_ none

_Result_ (example):
```yaml
{
  "one_wire_devices": 3,            # Number of 1-wire devices at ds18x.pin.
  "ds18x_devices": 2,               # Number of supported sensors.
  "parasite_power": true,           # Whether any sensor is on a true 1-wire+GND connection.
  "conf": {
    "auto_save_scratchpad": true,   # Sensor configuration persists across power cycling.
    "check_for_conversion": true,   # When measuring temperatures, “listen” to the sensors,
                                    # don't apply the worst case delay.
    "resolution": 12                # (See GetDevInfo for the semantics.) Maximum among sensors.
  }
}
```

#### `DS18x.GetTemp` — Get all temperatures
Returns temperature data from all connected sensors (see also `GetAlarms`).  The
order of the result is the same as that of the sensors responding on the 1-wire
bus.

With `poll: false`, the measurements from the prior `GetAlarms`, `GetDevTemp`
or `GetTemp` operation or the power-on sensor state will be used, otherwise the
temperatures will be measured anew by all sensors before returning the data.
`GetInfo().check_for_conversion` and `GetInfo().resolution` affect how long this
can take.

_Arguments_ ([default]):
```yaml
{
  "poll": true                      # [true] False disables measuring temperatures anew.
}
```

_Result_ (example):
```yaml
[
  {
    "addr": "28cc19490c0000bb",     # 1-wire address of the 1st sensor.
    "temp": {                       # Temperature last measured:
      "C": 30.937500,               # - as degrees Celsius;
      "F": 87.687500,               # - as degrees Fahrenheit;
      "raw": 3960                   # - the raw number from the sensor.
    }
  },
  {                                 # Ditto for the 2nd sensor.
    "addr": "2819ef480c000021",
    "temp": {
      "C": 85.000000,               # It has possibly never been requested to actually measure the
      "F": 185.000000,              # temperature: the returned value is the power-on state.
      "raw": 10880
    }
  }
]
```

#### `DS18x.Rescan ` — Rescan the 1-wire connection
Rescans the 1-wire bus and reinitialises the whole state reported by `GetInfo`.

_Arguments:_ none

_Result:_ `null`

#### `DS18x.SetConf` — Set global configuration
Changes the behaviour when working with all sensors.

If `GetInfo().auto_save_scratchpad` (set prior or in the call), the effect of
setting `conf.resolution` persists indefinitely, otherwise until a power-off or
a `RecallScratchpad`.

_Arguments_ (example):
```yaml
{                                   # At least one conf.* setting is required,
  "conf": {                         # only those supplied will be changed.
    "auto_save_scratchpad": true,   # (See GetInfo for the semantics.)
    "check_for_conversion": true,   # (Ditto.)
    "resolution": 12                # (See GetDevInfo for the semantics.) Set for all sensors.
  }
}
```

_Result_: `null`

### Single-sensor operations

#### `DS18x.GetDevInfo` — Get the state and configuration of one sensor
The state and configuration are reset upon sensor power-ons.  The configuration
can be changed directly via `SetDevConf` and indirectly via `SetConf`
(`conf.resolution` only).

**NB:** The `addr` and `idx` arguments are mutually-exclusive selectors of the
sensor!  Either should be supplied, but not both.

_Arguments_ (examples):
```yaml
{                                   # One and only one of addr or idx is required:
  "addr": "2818b4490c00007c",       # - 1-wire address of the sensor;
  "idx": 0                          # - [0..GetInfo().ds18x_devices).
}
```

_Result_ (example):
```yaml
{
  "addr": "2818b4490c00007c",       # 1-wire address of the sensor.
  "parasite_power": true,           # Whether on a true 1-wire+GND connection.
  "conf": {                         # Alarm if the measured temperature is either:
    "alarm_high_C": 40,             # - at least this many degrees Celsius;
    "alarm_low_C": 7,               # - at most this many degrees Celsius.
    "resolution": 12,               # Bits of resolution to measure:
                                    # - 9/10/11/12: ±0.5/0.25/0.125/0.0625°C;
                                    # - 9/10/11/12: max ~94/188/375/750 ms delay.
    "user_data": 10247              # Alarm setting memory as a 16-bit value.
  }
}
```

#### `DS18x.GetDevTemp` — Get temperature from one sensor
**NB:** The `addr` and `idx` arguments are mutually-exclusive selectors of the
sensor!  Either should be supplied, but not both.

With `poll: false`, the measurement from the prior `GetAlarms`, `GetDevTemp` or
`GetTemp` operation or the power-on sensor state will be used, otherwise the
temperature will be measured anew by the sensor before returning the data.
`GetInfo().check_for_conversion` and `GetDevInfo().resolution` affect how long
this can take.

_Arguments_ (examples, [default]):
```yaml
{                                   # One and only one of addr or idx is required:
  "addr": "2819ef480c000021",       # - 1-wire address of the sensor;
  "idx": 1,                         # - [0..GetInfo().ds18x_devices).
  "poll": true                      # [true] False disables measuring temperature anew.
}
```

_Result_ (example):
```yaml
{
  "addr": "2819ef480c000021",     # 1-wire address of the sensor.
  "temp": {                       # Temperature last measured:
    "C": 32.562500,               # - as degrees Celsius;
    "F": 90.612503,               # - as degrees Fahrenheit;
    "raw": 4168                   # - the raw number from the sensor.
  }
}
```

#### `DS18x.SetDevConf` — Configure one sensor
Changes the configuration of one sensor.  If `GetInfo().auto_save_scratchpad`,
persists indefinitely, otherwise until a power-off or a `RecallScratchpad`.

**NB:** The `addr` and `idx` arguments are mutually-exclusive selectors of the
sensor!  Either should be supplied, but not both.

_Arguments_ (examples):
```yaml
{                                   # One and only one of addr or idx is required:
  "addr": "2818b4490c00007c",       # - 1-wire address of the sensor;
  "idx": 0,                         # - [0..GetInfo().ds18x_devices).
  "conf": {                         # At least one conf.* setting is required,
                                    # only those supplied will be changed.
    "alarm_high_C": 40,             # (See GetDevInfo for the semantics.) Precludes user_data.
    "alarm_low_C": 7,               # (Ditto.) Precludes user_data.
    "resolution": 12,               # (Ditto.)
    "user_data": 10247              # (Ditto.) Precludes alarm_high_C or alarm_low_C.
  }
}
```

_Result_: `null`

### Raw “scratchpad” data single-sensor operations
(Available via `ds18x.rpc.raw_enable`.)

The “scratchpad” sensor memory area (can be read via `ReadScratchpad`) contains:
- working state: the result of measuring temperature via `GetAlarms`,
  `GetDevTemp` or `GetTemp`, if none, the power-on state;
- sensor configuration:
  - read from EEPROM upon a power-on or a `RecallScratchpad`;
  - can be overwritten via `WriteScratchpad`;
  - can be modified via `SetDevConf` or `SetConf`.

#### `DS18x.ReadScratchpad` — Get raw memory of one sensor
**NB:** The `addr` and `idx` arguments are mutually-exclusive selectors of the
sensor!  Either should be supplied, but not both.

_Arguments_ (examples):
```yaml
{                                   # One and only one of addr or idx is required:
  "addr": "2818b4490c00007c",       # - 1-wire address of the sensor;
  "idx": 0                          # - [0..GetInfo().ds18x_devices).
}
```

_Result_ (example):
```yaml
{
  "addr": "2818b4490c00007c",       # 1-wire address of the sensor.
  "data": "2a014b467fff0c10f1"      # Raw sensor memory contents.
}
```

#### `DS18x.RecallScratchpad` — Restore configuration of one sensor from EEPROM
Only the configuration portion of the sensor memory is restored, the working
state portion is retained.

**NB:** The `addr` and `idx` arguments are mutually-exclusive selectors of the
sensor!  Either should be supplied, but not both.

_Arguments_ (examples):
```yaml
{                                   # One and only one of addr or idx is required:
  "addr": "2818b4490c00007c",       # - 1-wire address of the sensor;
  "idx": 0                          # - [0..GetInfo().ds18x_devices).
}
```

_Result:_ `null`

#### `DS18x.WriteScratchpad` — Set raw configuration of one sensor
Only the configuration portion of the sensor memory is overwritten, the working
state portion is retained.  If `GetInfo().auto_save_scratchpad`, the overwritten
configuration is also written into the EEPROM.

**NB:** The `addr` and `idx` arguments are mutually-exclusive selectors of the
sensor!  Either should be supplied, but not both.

_Arguments_ (examples):
```yaml
{                                   # One and only one of addr or idx is required:
  "addr": "2818b4490c00007c",       # - 1-wire address of the sensor;
  "idx": 0,                         # - [0..GetInfo().ds18x_devices).
  "data": "2a014b467fff0c10f1"      # Raw sensor memory contents.
}
```

_Result:_ `null`
