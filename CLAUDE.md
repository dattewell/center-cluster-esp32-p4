# DashRPM — Hardware Interface Specification

Firmware reference for the DashRPM custom carrier board.
Every value here was verified against the KiCad schematic/PCB netlist and the
relevant manufacturer datasheets. Where something is **unverified**, it says so
explicitly — do not silently assume a value in those places.

---

## 0. Authority of this document

**This file is the single source of truth for pin assignments and component
values. The PCB is fabricated — copper cannot change to match firmware.**

If existing firmware, an older document, or a recollection of a previous
conversation disagrees with this file, **this file is correct and the other
source is stale**. Do not reconcile a conflict by averaging, by preferring the
code, or by treating a remembered decision as evidence. Change the firmware.

Every pin in §2 was read directly from the `Net-(J3-nn)` assignments in
`DashRPM.kicad_pcb`. If you have file access, verify against that rather than
against any prose description.

---

## 1. System overview

| | |
|---|---|
| **Compute module** | Waveshare ESP32-P4-WIFI6-Touch-LCD-XC (3.4" round LCD, touch) |
| **Carrier board** | DashRPM, 110 × 80 mm, 2-layer, 2 oz copper |
| **Interconnect** | 40-way IDC ribbon, 2×20 @ 2.54 mm, Raspberry-Pi-compatible pin skeleton |
| **Vehicle** | Morris Minor, **negative earth**, alternator, 123ignition Tune+ distributor, Lucas Sports coil 3 Ω |
| **Display stack** | LVGL |

**Power topology**

```
Battery+ ──> J2 ──> F2 (7.5A) ──┬── D2 (5KP24CA TVS) ── GND
                                └── Q1 (SQD50P04 P-FET, reverse polarity)
                                      └── /12v+ ──┬── J7 ──> external buck (J2D25K2405, 8–36V in, 5V 5A out)
                                                  │                              │
                                                  └── F1 (5A) ──> U3 VS          └──> J6 ──> /5v+ ──> J3.2, J3.4
                                                        (BTS7004-1EPP PROFET)
                                                        └──> J4.1 AFR heater
```

3.3 V is **supplied by the Waveshare module** into the carrier board via J3.1 / J3.17.
The carrier board draws ~38 mA worst case from it (GPS + pull-ups).

---

## 2. GPIO map

All GPIO reach the ESP32-P4 through connector J3. Verified against the
ESP32-P4 datasheet: ADC is multiplexed onto GPIO16–23 (ADC1) and GPIO49–54 (ADC2).
**No pin used here is a strapping pin** (those are GPIO34–38).

| GPIO | J3 pin | Direction | Function | Notes |
|---|---|---|---|---|
| **20** | 13 | Analog in | Coolant temperature | **ADC1_CHANNEL4** |
| **21** | 11 | Analog in | PROFET current sense (IS) | **ADC1_CHANNEL5** |
| **28** | 15 | Output | GPS UART TX (ESP → module) | |
| **29** | 7 | Input | GPS UART RX (module → ESP) | |
| **47** | 37 | Input | RPM pulse from coil | **Active LOW**, 4k7 pull-up on board |
| **48** | 33 | Input | AFR UART RX (Spartan → ESP) | via U5 level shifter |
| **49** | 26 | Output | PROFET DEN (diagnostic enable) | 10k pull-down on board |
| **50** | 23 | Input | Headlight sense | **Active LOW**, 4k7 pull-up on board |
| **51** | 29 | Output | PROFET IN (AFR heater on/off) | 10k pull-down on board |
| **52** | 35 | Output | AFR UART TX (ESP → Spartan) | via U4 level shifter |

Both ADC channels are on **ADC1**, deliberately — ADC2 conflicts with WiFi under ESP-IDF.

### Safe-state guarantee

R14 and R17 (10 kΩ) pull GPIO51 and GPIO49 to ground. While the ESP32 is in
reset or its GPIOs are high-impedance, **the AFR heater is off**. Firmware must
not defeat this by driving them high before the AFR controller is ready.

### Pins present on J3 but NOT connected to anything

These reach the connector and terminate there. **Assigning a function to any of
them produces silence, not an error.**

```
GPIO2, GPIO3, GPIO4, GPIO5, GPIO22, GPIO24, GPIO25, GPIO30,
GPIO31, GPIO32, GPIO34, GPIO35, GPIO36, GPIO46,
plus SDA (pin 3), SCL (pin 5), TXD (pin 8), RXD (pin 10)
```

**GPIO4 in particular is not the heater enable.** It is J3 pin 18 and connects
to nothing. The heater enable is GPIO51.

### ⚠ Pins that are hazardous to misconfigure

| Pin | Why |
|---|---|
| **GPIO21** | Wired through R9 (4.7 kΩ) to U3 pin 4 — the PROFET's IS **output**. Configuring this as any kind of output puts your driver in contention with a current-source output on a power switch. Input/ADC only. |
| **GPIO51** | Drives the PROFET IN pin. Anything that raises it turns on the AFR heater. Never drive it high as a side effect of pin testing. |
| **GPIO20** | Sits on the coolant divider behind a BAV99 clamp. Input/ADC only. |

---

## 3. Coolant temperature (GPIO20 / ADC1_CH4)

### Circuit

```
3V3 ──[ R1 470R ]──┬── J8.1 ──> Lucas/Smiths NTC sender ──> chassis GND
                   │
                   ├── C16 1nF ── GND
                   │
                   └──[ R15 4k7 ]──┬── GPIO20
                                   ├── C3 1uF ── GND
                                   └── D7 BAV99 clamp to 3V3 / GND
```

### Sender characteristics

| Parameter | Value |
|---|---|
| R at 25 °C | **730 Ω** |
| Beta (25/85) | **3763** |
| Pull-up R1 | **470 Ω** |

### Conversion

```c
#define TEMP_PULLUP_OHMS   470.0f
#define TEMP_R25_OHMS      730.0f
#define TEMP_BETA          3763.0f
#define TEMP_T0_KELVIN     298.15f
#define SENSOR_SUPPLY_MV   3300.0f   // measure the real 3V3 rail and hardcode it

// millivolts -> degrees C
float coolant_c_from_mv(float mv) {
    float r = TEMP_PULLUP_OHMS * mv / (SENSOR_SUPPLY_MV - mv);
    float invT = 1.0f/TEMP_T0_KELVIN + logf(r / TEMP_R25_OHMS) / TEMP_BETA;
    return 1.0f/invT - 273.15f;
}
```

### Expected readings

| °C | R sender | ADC mV | Sensitivity |
|---|---|---|---|
| −10 | 3913 Ω | 2946 | — |
| 0 | 2444 Ω | 2765 | 18 mV/°C |
| 25 | 730 Ω | 2007 | 33 mV/°C |
| 60 | 195 Ω | 967 | 22 mV/°C |
| 90 | 74 Ω | 461 | 12.9 mV/°C |
| 100 | 58 Ω | 361 | 9.9 mV/°C |
| 110 | 45 Ω | 285 | 7.6 mV/°C |
| 120 | 35 Ω | 226 | 5.9 mV/°C |

Accuracy with the P4's ±15 mV ADC error: **±0.45 °C at 25 °C, ±1.5 °C at 100 °C**.

### Fault detection

```c
#define TEMP_OPEN_MV    3150   // open circuit reads ~3300 mV
#define TEMP_SHORT_MV     40   // short to ground reads ~0 mV
```

> **Do not use 3000 mV for the open threshold.** A *working* sender at −10 °C
> reads 2946 mV — only 54 mV of margin. 3150 mV keeps clean separation while
> still catching a genuine open (3300 mV).

### Filtering

R15 × C3 = 4.7 kΩ × 1 µF = **4.7 ms** hardware time constant. The signal is
thermally slow; sample at 1–5 Hz with 16–64× oversampling.

---

## 4. AFR heater control + current sense (GPIO51, GPIO49, GPIO21)

U3 is an **Infineon BTS7004-1EPP** PROFET (LCSC C534825, PG-TSDSO-14-22).
All nine support components match Infineon's Table 22 reference design.

### Control

| GPIO | Pin | Behaviour |
|---|---|---|
| 51 | IN | HIGH = heater on. 4k7 series (R6), 10k pull-down (R14) |
| 49 | DEN | HIGH = enable IS output. 4k7 series (R16), 10k pull-down (R17) |

**DEN must be HIGH to read a valid current from IS.** With DEN low the sense
output is disabled and GPIO21 reads near zero regardless of load.

### Current sense conversion

```
U3 IS ──[ R9 4k7 ]──┬── GPIO21
                    ├── C11 100nF ── GND
                    └── D8 BAV99 clamp to 3V3 / GND
   │
   └──[ R8 8k2 ]── GND
```

```c
#define IS_SENSE_OHMS   8200.0f
#define IS_KILIS        20000.0f   // BTS7004-1EPP

float heater_amps_from_mv(float mv) {
    return (mv / 1000.0f) / IS_SENSE_OHMS * IS_KILIS;   // = mV * 0.0024390
}
```

| Heater current | ADC reading |
|---|---|
| 0.5 A | 205 mV |
| 1.0 A | 410 mV |
| 2.0 A | 820 mV |
| 8.05 A | 3300 mV (**full scale — saturates above this**) |

### kILIS accuracy is strongly current-dependent

| Load current | Tolerance |
|---|---|
| 5.5 A | ±5.4% |
| 2.0 A | ±10.5% |
| 450 mA | ±16.5% |
| 200 mA | ±22% |
| 50 mA | ±31% |

**Do not display or log heater current below ~200 mA as a meaningful number.**

### Fault handling

A PROFET fault (short, overtemperature, open load) drives IS into saturation —
IIS(FAULT) reaches up to 10 mA, which would be 82 V across R8. The pin clamps at
VS, and D8 shunts the excess into the 3.3 V rail through R9. **Any reading pinned
at full scale should be treated as a fault, not as 8 A.**

Sense settling time is 5–40 µs, but C11 gives a 470 µs RC. Allow **~2.5 ms**
after enabling DEN or changing the load before trusting a reading.

---

## 5. RPM from ignition coil (GPIO47)

### Circuit

```
COIL1.1 (coil negative, 0–400V) ──[ R10 15k ]──[ R11 15k ]──[ R12 15k ]──┬── D5 15V zener ── GND
                                                                         │
                                                                    [ R13 2k7 ]
                                                                         │
                          ┌──────────────────────────────────────────────┤
                     [ R23 1k ]                                     U2 H11L1 LED
                          │                                              │
                         GND                                            GND

U2 output (open collector) ──┬── GPIO47
                             ├── R5 4k7 pull-up to 3V3
                             └── C19 4.7nF ── GND
```

### Signal behaviour

- **Idle (between sparks):** coil negative sits at battery voltage. R23 holds the
  opto LED at ~0.01 mA — **hard off**. GPIO47 reads **HIGH**.
- **Spark:** flyback drives coil negative to 300–400 V. D5 clamps the divider node
  at 15 V, R13 delivers ~3.85 mA to the LED. GPIO47 pulled **LOW**.

**Trigger on the FALLING edge.**

### RPM calculation

Morris Minor A-series is a 4-cylinder 4-stroke → **2 sparks per crankshaft
revolution**. Confirmed against the 123ignition Tune+ configuration.

```c
#define SPARKS_PER_REV  2.0f
// rpm = sparks_per_second * 60 / SPARKS_PER_REV = sparks_per_second * 30
```

| RPM | Sparks/sec | Period |
|---|---|---|
| 700 (idle) | 23.3 | 42.9 ms |
| 3000 | 100 | 10.0 ms |
| 6000 | 200 | 5.0 ms |

### Blanking — required

```c
#define RPM_BLANK_US  2000   // 2 ms
```

Ignition ringing produces multiple edges per spark. Reject any edge occurring
within 2 ms of the previous accepted one. This caps measurable RPM at 15000 —
far above anything the engine will see.

R5 × C19 = 4.7 kΩ × 4.7 nF gives a **22 µs** rise time. The ESP32-P4 GPIO input
is Schmitt-triggered so this is clean, but do not expect sub-microsecond edges.

### Implementation notes

- Use a GPIO ISR on the falling edge, or better, the **PCNT** peripheral with its
  hardware glitch filter.
- Timestamp edges and compute RPM from the **interval**, not by counting per
  fixed window — much better resolution at idle.
- **Timeout to 0 RPM after ~500 ms** with no edges (engine stopped).
- Median-filter over 3–5 intervals to reject the occasional missed or spurious spark.

---

## 6. Headlight sense (GPIO50)

```
J1.1 (headlight 12V) ──[ R20 1k65 ]──[ R19 1k65 ]──> U6 H11L1 LED ── GND
                       └── D6 SMBJ33A TVS ── GND

U6 output (open collector) ──┬── GPIO50
                             ├── R18 4k7 pull-up to 3V3
                             └── C17 ── GND
```

| Headlights | LED current | GPIO50 |
|---|---|---|
| Off (0 V) | 0 mA | **HIGH** |
| On (12 V) | 3.26 mA | **LOW** |
| On (14.4 V, charging) | 3.98 mA | **LOW** |

**Active LOW.** Debounce in software — 200–500 ms is appropriate; this drives
display dimming and must not flicker on a bad earth or a flash-to-pass.

### Actuator — backlight PWM on the Waveshare module

GPIO50 is the *trigger*. The *actuator* is the display backlight, which lives
entirely on the Waveshare module — **nothing on the DashRPM carrier board is
involved, and no J3 pin carries a backlight signal**.

The Waveshare schematic contains `LCD_BL_PWM`, `BL_EN`, and `LEDA`/`LEDK`, so
there is a real hardware backlight driver with PWM control. Dim by driving that
PWM channel, not by darkening colours in LVGL — an LCD backlight at full output
still glares at night regardless of what is rendered.

> **⚠ UNVERIFIED:** the GPIO number driving `LCD_BL_PWM` is not determinable
> from the schematic files. Obtain it from Waveshare's BSP header or ESP-IDF
> demo (typically `BSP_LCD_BACKLIGHT` or `EXAMPLE_PIN_NUM_BK_LIGHT`).

Suggested behaviour: ramp between day and night duty cycles over ~1 s rather
than stepping, and ignore transitions shorter than 1 s. Keep a minimum night
duty above zero so the display never goes fully dark.

---

## 7. AFR — 14Point7 Spartan 3 Lite (GPIO52 TX / GPIO48 RX)

### Level shifting

Two **SN74LVC1T45DBVR**, one per direction, fixed direction (these are push-pull,
not auto-sensing):

| | Device | DIR | Path |
|---|---|---|---|
| ESP → Spartan | U4 | tied 3V3 | A→B, 3.3 V → 5 V, R2 1 kΩ series |
| Spartan → ESP | U5 | tied GND | B→A, 5 V → 3.3 V, R4 330 Ω series, R21 10k pull-up to 5 V |

There is **no output-enable line** — the shifters are always active. Do not look
for an OE GPIO.

### UART parameters (confirmed — Spartan 3 Lite v2)

| Parameter | Value |
|---|---|
| Logic level | **5 V TTL** (handled by U4/U5, ESP32 sees 3.3 V) |
| Baud rate | **9600** |
| Data bits | 8 |
| Stop bits | **1** |
| Parity | **None** |
| Flow control | **None** |

```c
#define AFR_UART_NUM   UART_NUM_1
#define AFR_TX_GPIO    52
#define AFR_RX_GPIO    48
#define AFR_BAUD       9600

const uart_config_t afr_uart_cfg = {
    .baud_rate  = AFR_BAUD,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};
```

**Signal integrity is comfortable at this rate.** One bit is 104 µs. R2 (1 kΩ)
driving ~100 pF of cable gives a 100 ns rise time — 0.1% of a bit period. Cable
length is not a concern; TX/RX length matching is irrelevant (UART is
asynchronous, and trace skew here is ~0.6 ns against a 104 µs bit).

### Display units

The gauge is scaled in **air/fuel ratio**, not lambda.

```c
#define AFR_STOICH_PETROL   14.7f   // ethanol-free 95/98 RON
// If the Spartan streams lambda:  afr = lambda * AFR_STOICH_PETROL
```

Stoichiometric AFR is a property of the *fuel chemistry*, not its octane rating —
95 and 98 RON are both 14.7:1. What changes it is ethanol content: E10 is
14.1:1 and E85 is 9.8:1. NZ 95/98 from the major brands is normally
ethanol-free, so 14.7 applies. If the car is ever run on E10, a gauge calibrated
at 14.7 will read roughly 4% lean.

Suggested gauge range 10:1 to 20:1 with the stoichiometric mark at 14.7.

### ⚠ STILL UNVERIFIED — frame format

The **payload protocol** is not determined by the UART parameters above. Before
writing a parser, confirm from the Spartan 3 Lite v2 manual:

- ASCII text lines vs binary packets
- Units: **lambda or AFR** — and if AFR, which fuel's stoichiometric ratio
  (petrol 14.7:1 is the usual default, but the Spartan is often configured for
  lambda and converted host-side)
- Line terminator, packet framing, and whether there is a checksum
- Whether sensor status / heater state / error codes are included in the stream

At 9600 baud you have ~960 bytes/sec, so expect a modest fixed-rate stream
(typically 10–20 Hz of short frames). Buffer by line/packet rather than polling
for single bytes.

### Heater interaction

The Spartan 3 controls the LSU sensor heater itself. The BTS7004 on this board
**supplies power** to the Spartan's heater circuit; it is a supply switch, not
the PWM element. Turn GPIO51 on and leave it on — do not attempt to PWM it.

---

## 8. GPS — u-blox M10 (GPIO29 RX / GPIO28 TX)

```c
#define GPS_UART_NUM   UART_NUM_2
#define GPS_TX_GPIO    28   // ESP -> module (config commands)
#define GPS_RX_GPIO    29   // module -> ESP (NMEA)
#define GPS_BAUD       9600 // u-blox M10 default; confirm against your module
```

Direct 3.3 V connection, no level shifting, no series protection.
Module is powered from the carrier board's 3.3 V (J5.1), drawing ~25–30 mA.

Speed comes from NMEA `RMC` (or `VTG`). Prefer configuring the module to output
only what you use, at 5–10 Hz, to reduce parsing load.

### Speed units — display both

NZ roads are signed in km/h, but the original Smiths speedo carries both scales.
Display **km/h as primary** (larger) with **mph secondary** (smaller).

```c
#define KNOTS_TO_KMH   1.852f
#define KMH_TO_MPH     0.621371f
```

| NMEA sentence | Field | Native unit |
|---|---|---|
| `VTG` | 7 | **km/h** — use this, no conversion needed |
| `RMC` | 7 | knots — multiply by 1.852 |

**Keep one internal unit and convert only at render time.** Hold speed in km/h
and distance in **metres**; derive mph for display. Do not maintain two speed
variables — they will drift apart.

This matters most for the odometer: the NVS write trigger in §9 is every 100 m
of accumulated distance. Keep that accumulator in metres regardless of display
units, or rounding error compounds into the stored total over thousands of km.

---

## 9. Display and persistence

- 3.4" round LCD, LVGL. Displays AFR, coolant temperature, speed, RPM.
- Backlight dimming driven by GPIO50 (see §6).
- **Odometer persistence: NVS flash, write every 100 m of accumulated distance.**
  Do not write per-GPS-fix — NVS has finite erase cycles. Also flush on a clean
  shutdown signal if one is available.

---

## 10. ADC configuration

```c
// Both channels on ADC1
adc_oneshot_chan_cfg_t cfg = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten    = ADC_ATTEN_DB_12,      // 0–3300 mV effective range
};
// GPIO20 -> ADC_CHANNEL_4   (coolant)
// GPIO21 -> ADC_CHANNEL_5   (PROFET IS)
```

- **Use the calibration API.** Uncalibrated readings are considerably worse than
  the ±15 mV specified figure.
- ATTEN_DB_12 is **required**, not optional — coolant at −10 °C reads 2946 mV,
  which would clip on any lower attenuation setting.
- DNL −1/+3 LSB, INL −5/+3 LSB. Oversample 16–64× and average.
- Max sampling rate 100 kSPS; nothing here needs more than a few Hz.

---

## 11. Recommended init order

1. Configure GPIO51 (IN) and GPIO49 (DEN) as outputs, **drive both LOW**.
2. Init ADC1 with calibration; read coolant to sanity-check the 3.3 V rail.
3. Init GPS UART, start NMEA parsing.
4. Init AFR UART (9600 8N1, no flow control).
5. Configure GPIO47 falling-edge interrupt / PCNT with 2 ms blanking.
6. Configure GPIO50 as input with debounce.
7. Bring up LVGL and the display.
8. **Only then** raise GPIO51 (heater on) and GPIO49 (DEN on).
9. Wait ≥2.5 ms, then begin reading GPIO21 for heater current.

---

## 12. Traps

| Trap | Why it matters |
|---|---|
| GPIO47 and GPIO50 are **active LOW** | Both are open-collector opto outputs with pull-ups |
| **DEN must be high** to read IS | Otherwise GPIO21 reads ~0 regardless of actual current |
| **Do not PWM GPIO51** | The Spartan 3 does its own heater PWM; this is a supply switch |
| Coolant open threshold must be **3150 mV**, not 3000 | 3000 mV false-triggers at −10 °C |
| IS full-scale = **fault**, not 8 A | The measurable range only extends to 8.05 A |
| Ignore heater current below **200 mA** | kILIS tolerance is ±22% or worse there |
| 5 V through J3 is limited to **~2 A** | Two contacts at 1 A each, not the buck's 5 A |
| RPM needs **2 ms blanking** | Ignition ringing produces multiple edges per spark |
| GPS lines have **no series protection** | Treat a GPS cable fault as able to reach the ESP32 directly |

---

## 13. Bench validation

Before fitting to the car:

1. **Unpowered:** confirm 45 kΩ across the coil divider (R10+R11+R12) with a multimeter.
2. **Ribbon orientation:** with nothing powered, probe continuity from carrier
   board ground to the Waveshare end. Grounds must appear on pins
   **6, 9, 14, 20, 25, 30, 34, 39**. Any other pattern means the cable is reversed.
3. **Opto chain:** inject 12 V at the D5 cathode node. GPIO47 should swing from
   3.3 V to below 0.4 V.
4. **Coolant:** substitute a 730 Ω resistor for the sender — expect ~2007 mV / 25 °C.
5. **In-car RPM:** cross-reference against the 123ignition Tune+ Bluetooth app.

---

## 14. Open decisions — ask, do not assume

These are product decisions, not hardware facts. **Ask before implementing.**

| # | Question | Why it matters |
|---|---|---|
| 1 | **Which GPIO drives `LCD_BL_PWM`** on the Waveshare module? See §6 — get it from their BSP or demo code | Backlight dimming cannot be implemented without it |
| 2 | **AFR frame format** — see §7. ASCII vs binary, terminator, checksum, whether status/heater state ride along | Blocks the parser entirely |
| 3 | **Odometer: trip, total, or both?** Displayed in km, miles, or both? Is this replacing the mechanical odometer or supplementing it? Is there a reset UI? | Affects NVS schema and touch UI. Store metres internally either way (§8) |
| 4 | **Coolant warning thresholds** — at what temperature does the display change state? | Sensitivity is 9.9 mV/°C at 100 °C; ±1.5 °C accuracy |
| 5 | **Behaviour when the engine is off but ignition is on** — RPM reads 0. Does the display show a resting state, or is that indistinguishable from a sensor fault? | §5 timeout is 500 ms |

### Resolved

- **Speed units:** km/h primary, mph secondary; internal storage metric (§8)
- **AFR units:** air/fuel ratio, stoichiometric 14.7, petrol ethanol-free (§7)
- **Sparks per revolution:** 2 (§5)
- **Backlight:** hardware PWM on the Waveshare module, not this board (§6)
- **AFR UART:** 9600 8N1, no flow control (§7)


### Hardware items still open (not firmware)

- Enclosure clearance for the two **20 mm tall** electrolytics at C2 (154.13, 83) and C6 (129, 55.37)
- Fourth mounting hole at ~(68, 50) — the top-left corner is currently 71.5 mm from the nearest fixing
- **No carrier-board-presence signal.** Every J3 sense line the ESP32 reads idles at a
  defined level via an external pull-up/pull-down *on the carrier board*, so firmware
  cannot tell "carrier board unplugged" from "carrier board attached, inputs idle" —
  adding a matching internal pull (as done for GPIO50) makes the ambiguity worse, not
  better. A future revision would need a dedicated presence pin: ground one of the
  currently-NC J3 pins (§2) on the carrier board and sense it with an internal pull-up
  on the ESP side.
- PCBWay assembly scope for the 14 through-hole parts