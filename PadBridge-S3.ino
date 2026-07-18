/*
 * ESP32-S3-Zero — BLE controller → GPIO (open-drain, active-low)
 * -------------------------------------------------------------
 * Connects to a BLE controller and injects each button onto a GPIO
 * wired to a console test pad. Outputs are OPEN-DRAIN:
 *   pressed  = pin pulled to GND (sinks the pad low)
 *   released = pin high-impedance / floating (pad returns to idle)
 * This suits active-low inputs like the Nintendo DS Lite button pads,
 * where the console has an internal pull-up and a press shorts to GND.
 * We only ever sink to ground, never source 3.3 V, so we don't fight
 * the console's pull-up or its rail voltage.
 *
 * >>> WIRING — READ THIS <<<
 *  - Tie the ESP32 GND to the DS Lite GND. Without a shared ground,
 *    "pull to ground" is meaningless and nothing will register.
 *  - One GPIO per button pad (see the map below). No series resistor
 *    needed for a simple switch pad, but a ~100-330 ohm in series is
 *    cheap insurance against a miswired pad.
 *  - Confirm each pad idles at <= 3.3 V with a meter before soldering.
 *    DS Lite runs 3.3 V logic, so this is normally fine.
 *
 * TURBO MODE — hold Start + Select together for 3 seconds to toggle.
 * While active, the A/B/X/Y/L1/R1 pins auto-fire (rapidly pulse) for
 * as long as the button is held, and the RGB LED blinks green. Hold
 * Start + Select for 3 s again to turn it back off (LED goes solid).
 *
 * Library:  Bluepad32  (host-side Bluetooth HID for ESP32)
 * Chip:     ESP32-S3 (BLE only — no Bluetooth Classic)
 * Tested controller: Xbox Wireless Controller model 1914 (BLE).
 *
 * IMPORTANT — the ESP32-S3 has no Bluetooth Classic radio, so this
 * works with BLE controllers only. The Xbox 1914 (firmware v5+) is
 * BLE; PS4/PS5/Switch Pro and most 8BitDo pads are Classic and will
 * NOT connect.
 *
 * ----------------------- SETUP (Arduino IDE) -----------------------
 * 1. File > Preferences > "Additional boards manager URLs", add BOTH:
 *      https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *      https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
 * 2. Tools > Board > Boards Manager: install "esp32_bluepad32 by Ricardo Quesada".
 * 3. Tools > Board: pick an ESP32-S3 board from the *(bluepad32)* group,
 *    e.g. "ESP32S3 Dev Module".
 * 4. Tools > "USB CDC On Boot" = Enabled   (S3-Zero uses native USB for Serial).
 * 5. Flashing: hold BOOT, plug in USB-C, release, then Upload.
 * -------------------------------------------------------------------
 */

#include <Bluepad32.h>

// ============================ CONFIG ==============================

// Outputs are open-drain / active-low (see header): pressed pulls the
// pad to GND, released floats it. This is fixed by the DS Lite hardware,
// so there's no polarity option here — pressing always sinks to ground.

// Set to 0 if the RGB status LED still won't compile on your core.
// (The code auto-selects rgbLedWrite() on core 3.x and neopixelWrite()
// on core 2.x; disabling it just skips the status LED entirely.)
#define USE_STATUS_LED 1
static const int RGB_LED_PIN = 21;   // WS2812 on the ESP32-S3-Zero

// Print raw button/dpad/misc bitmasks over USB serial whenever they
// change. Handy for discovering which bit your controller uses, then
// you can remap below. Requires "USB CDC On Boot = Enabled".
#define DEBUG_SERIAL 1

// ---- Turbo ----
// Hold Start + Select together this long (ms) to toggle turbo on/off.
static const unsigned long TURBO_HOLD_MS    = 3000;
// Auto-fire timing: pad is "pressed" for one half-period, "released"
// the next. 50 ms half-period => ~10 presses/sec. Lower = faster.
static const unsigned long TURBO_HALF_MS    = 50;
// Turbo indicator: LED blink half-period (ms).
static const unsigned long LED_BLINK_MS     = 200;

// ---- Bit definitions (stable across Bluepad32 versions) ----
// Groups tell processGamepad() which reader the mask applies to.
enum Group { BTN, DPAD, MISC };

// buttons() bitmask
#define B_A         0x0001
#define B_B         0x0002
#define B_X         0x0004
#define B_Y         0x0008
#define B_L1        0x0010   // left bumper / shoulder
#define B_R1        0x0020   // right bumper / shoulder
#define B_L2        0x0040   // left trigger (digital)
#define B_R2        0x0080   // right trigger (digital)
#define B_THUMB_L   0x0100   // left stick press  (L3)
#define B_THUMB_R   0x0200   // right stick press (R3)
// dpad() bitmask
#define D_UP        0x01
#define D_DOWN      0x02
#define D_RIGHT     0x04
#define D_LEFT      0x08
// miscButtons() bitmask. On the Xbox 1914: SELECT = the "View" button
// (left, two-squares), START = the "Menu" button (right, ≡), SYSTEM =
// the Xbox logo. Confirm with DEBUG_SERIAL if unsure.
#define M_SYSTEM    0x01     // logo / Home / Xbox button
#define M_SELECT    0x02     // View / Select / Share / Back
#define M_START     0x04     // Menu / Start / Options
#define M_CAPTURE   0x08     // Capture / share (if present) / spare

// ===================== BUTTON → DS LITE PAD MAP ===================
// Each GPIO solders to one DS Lite button test pad. The 12 entries
// below match the DS Lite's digital inputs 1:1 (A B X Y L R, the
// D-pad, Select and Start). Re-order the pins to match however your
// wiring physically reaches the pads.
//
// Safe, broken-out GPIOs on the ESP32-S3-Zero: 1-18.
// AVOID: 0 (BOOT), 19/20 (USB), 21 (RGB LED), 26-32 (internal flash),
//        33-37 (not broken out), 43/44 (UART TX/RX), 45/46 (strapping).
// (GPIO3 is a mild strapping pin but is fine as a plain output.)

// The "turbo" column marks which buttons auto-fire when turbo mode is on.

struct ButtonMap {
  const char* name;
  Group       group;
  uint16_t    mask;
  uint8_t     pin;
  bool        turbo;   // true = pulses in turbo mode
};

static ButtonMap MAP[] = {
  //  name        group  mask       pin  turbo
  { "A",        BTN,  B_A,        1,  true  },
  { "B",        BTN,  B_B,        2,  true  },
  { "X",        BTN,  B_X,        3,  true  },
  { "Y",        BTN,  B_Y,        4,  true  },
  { "L1",       BTN,  B_L1,       5,  true  },
  { "R1",       BTN,  B_R1,       6,  true  },
  { "DpadUp",   DPAD, D_UP,       7,  false },
  { "DpadDown", DPAD, D_DOWN,     8,  false },
  { "DpadLeft", DPAD, D_LEFT,     9,  false },
  { "DpadRight",DPAD, D_RIGHT,    10, false },
  { "Select",   MISC, M_SELECT,   11, false },
  { "Start",    MISC, M_START,    12, false },
};
static const size_t MAP_LEN = sizeof(MAP) / sizeof(MAP[0]);

// ==================================================================

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Latest controller state, refreshed whenever a report arrives. Outputs
// are driven from these in loop() so turbo can keep pulsing a held
// button even between controller reports. (Designed for one controller.)
static uint16_t g_buttons = 0;
static uint8_t  g_dpad    = 0;
static uint8_t  g_misc    = 0;

static bool turboMode = false;

static inline void writePin(uint8_t pin, bool pressed) {
  // Open-drain pin: LOW actively sinks the pad to GND (= button press);
  // HIGH releases the driver so the pad floats back to idle (= release).
  digitalWrite(pin, pressed ? LOW : HIGH);
}

// Drive every mapped pin to its "released" (floating) state.
static void releaseAllPins() {
  for (size_t i = 0; i < MAP_LEN; i++) writePin(MAP[i].pin, false);
}

static bool anyConnected() {
  for (auto c : myControllers)
    if (c && c->isConnected()) return true;
  return false;
}

#if USE_STATUS_LED
static void setLed(uint8_t r, uint8_t g, uint8_t b) {
  // Only write when the color actually changes (avoids spamming the LED).
  static int lr = -1, lg = -1, lb = -1;
  if (r == lr && g == lg && b == lb) return;
  lr = r; lg = g; lb = b;
  // rgbLedWrite() exists on ESP32 Arduino core 3.x; core 2.x (which the
  // Bluepad32 package currently uses) calls it neopixelWrite().
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(RGB_LED_PIN, r, g, b);
#else
  neopixelWrite(RGB_LED_PIN, r, g, b);
#endif
}
#else
static void setLed(uint8_t, uint8_t, uint8_t) {}
#endif

// --------------------- connection callbacks -----------------------
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
#if DEBUG_SERIAL
      ControllerProperties p = ctl->getProperties();
      Serial.printf("Controller connected (slot %d): %s  VID=0x%04x PID=0x%04x\n",
                    i, ctl->getModelName().c_str(), p.vendor_id, p.product_id);
#endif
      return;
    }
  }
#if DEBUG_SERIAL
  Serial.println("Controller connected but no free slot");
#endif
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
#if DEBUG_SERIAL
      Serial.printf("Controller disconnected (slot %d)\n", i);
#endif
      break;
    }
  }
  g_buttons = 0;
  g_dpad = 0;
  g_misc = 0;
  releaseAllPins();              // fail-safe: release every pad (float)
}

// ------------------------- per-frame work --------------------------
// Store the latest controller state. Actual pin driving happens in
// applyOutputs() so turbo can pulse held buttons between reports.
void processGamepad(ControllerPtr ctl) {
  g_buttons = ctl->buttons();
  g_dpad    = ctl->dpad();
  g_misc    = ctl->miscButtons();

#if DEBUG_SERIAL
  static uint16_t lastB = 0xFFFF;
  static uint8_t  lastD = 0xFF, lastM = 0xFF;
  if (g_buttons != lastB || g_dpad != lastD || g_misc != lastM) {
    Serial.printf("buttons=0x%04x  dpad=0x%02x  misc=0x%02x\n",
                  g_buttons, g_dpad, g_misc);
    lastB = g_buttons; lastD = g_dpad; lastM = g_misc;
  }
#endif
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
      processGamepad(ctl);
    }
  }
}

// Toggle turbo when Start + Select are held together for TURBO_HOLD_MS.
// Latches after firing so it toggles once per hold, not repeatedly.
void handleTurboToggle() {
  static bool held = false;
  static bool latched = false;
  static unsigned long since = 0;

  bool combo = (g_misc & M_SELECT) && (g_misc & M_START);
  if (combo) {
    if (!held) { held = true; latched = false; since = millis(); }
    if (!latched && (millis() - since >= TURBO_HOLD_MS)) {
      turboMode = !turboMode;
      latched = true;
#if DEBUG_SERIAL
      Serial.printf("Turbo %s\n", turboMode ? "ON" : "OFF");
#endif
    }
  } else {
    held = false;
  }
}

// Drive all output pins from the stored state. Runs every loop so turbo
// pulses continue even when the controller isn't sending new reports.
void applyOutputs() {
  if (!anyConnected()) { releaseAllPins(); return; }

  handleTurboToggle();

  // Square wave used for auto-fire while turbo is active.
  bool turboPhase = ((millis() / TURBO_HALF_MS) & 1UL) == 0;

  for (size_t i = 0; i < MAP_LEN; i++) {
    uint16_t src;
    switch (MAP[i].group) {
      case BTN:  src = g_buttons; break;
      case DPAD: src = g_dpad;    break;
      case MISC: src = g_misc;    break;
      default:   src = 0;         break;
    }
    bool pressed = (src & MAP[i].mask) != 0;
    if (turboMode && MAP[i].turbo && pressed) {
      pressed = turboPhase;   // auto-fire while the button is held
    }
    writePin(MAP[i].pin, pressed);
  }
}

// Red = waiting, solid green = connected, blinking green = turbo on.
void updateLed() {
  if (!anyConnected()) { setLed(40, 0, 0); return; }
  if (turboMode) {
    bool on = ((millis() / LED_BLINK_MS) & 1UL) == 0;
    setLed(0, on ? 40 : 0, 0);
  } else {
    setLed(0, 40, 0);
  }
}

// ------------------------------ setup ------------------------------
void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif

  // OUTPUT_OPEN_DRAIN: the pin can pull LOW or float, but never drives
  // HIGH — exactly what an active-low console pad needs.
  for (size_t i = 0; i < MAP_LEN; i++) {
    pinMode(MAP[i].pin, OUTPUT_OPEN_DRAIN);
  }
  releaseAllPins();              // start with everything released
  setLed(40, 0, 0);              // red until a controller connects

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Forget stored pairings so a fresh controller can bond every boot.
  // Comment out if you want it to auto-reconnect to the last pad.
  BP32.forgetBluetoothKeys();

  // Don't create a second "virtual" (mouse) device for pads that offer one.
  BP32.enableVirtualDevice(false);

  // Dynamically accept any compatible controller in pairing mode.
  BP32.enableNewBluetoothConnections(true);
}

// ------------------------------ loop -------------------------------
void loop() {
  if (BP32.update()) {
    processControllers();   // refresh g_buttons/g_dpad/g_misc
  }
  applyOutputs();           // drive pins (with turbo) every iteration
  updateLed();              // status + turbo blink
  delay(2);                 // yield to the Bluetooth task on the other core
}
