/*
  AnnaSeva · Smart Ration ATM — ESP32 firmware
  by CognoSpace

  - BLE (Nordic UART style service) talks to AnnaSeva.html (Web Bluetooth)
  - 3 servo gates (one per product slot)
  - 20x4 I2C LCD shows every step
  - Calibration + product names/prices saved in flash (Preferences / NVS)

  Libraries (Arduino IDE > Library Manager):
    - ESP32Servo         by Kevin Harrington / John K. Bennett
    - LiquidCrystal I2C  by Frank de Brabander
  BLE, Wire and Preferences come with the ESP32 board package (Espressif).
  Board: "ESP32 Dev Module". If the sketch is too big, choose
  Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)".

  Wiring:
    Servo 1 signal -> GPIO 15     Servo 2 signal -> GPIO 25     Servo 3 signal -> GPIO 32
    Servo V+ -> external 5V (2A+) Servo GND -> supply GND AND ESP32 GND (common ground)
    LCD SDA -> GPIO 21  LCD SCL -> GPIO 22  LCD VCC -> 5V  LCD GND -> GND
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

// ------------------------------------------------------------ settings
#define DEVICE_NAME   "AnnaSeva-ESP32"      // the web page looks for names starting with "AnnaSeva"
#define FW_VERSION    "1.0"
#define LCD_ADDR      0x27                  // try 0x3F if the screen stays blank
#define SVC_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID       "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   // page writes commands here
#define TX_UUID       "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // we notify the page here

const uint8_t SERVO_PINS[3] = {15, 25, 32};
const uint32_t MAX_OPEN_MS  = 120000;       // safety cap for one dispense
const uint32_t THANKS_MS    = 6000;         // "thank you" screen time
const uint32_t ORDER_IDLE_MS = 180000;      // order screen falls back to welcome after 3 min
const bool DETACH_IDLE_SERVOS = false;      // true = cut servo pulses when idle (less buzz, but the gate holds less firmly)

// ------------------------------------------------------------ data
struct Slot {
  char     name[20];
  float    pricePerKg;
  uint8_t  openAngle;
  uint8_t  closeAngle;
  uint16_t msPer100g;
};

Slot slots[3] = {
  // name                price  open close ms/100g   (your calibrated angles)
  {"Sona Masoori Rice", 60,   147,  111,  800},
  {"Whole Wheat Atta",  45,   115,   59,  900},
  {"Crystal Sugar",     48,   114,   74,  700},
};

Servo servos[3];
uint32_t detachAt[3] = {0, 0, 0};           // detach idle servos to stop jitter/heat
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);
Preferences prefs;

BLECharacteristic *txChar = nullptr;
volatile bool bleConnected = false;
volatile bool bleChanged = false;
QueueHandle_t cmdQueue;
#define CMD_LEN 96

enum Mode { M_IDLE, M_ORDER, M_DISPENSE, M_THANKS, M_CONFIG };
Mode mode = M_IDLE;
Mode modeBeforeDispense = M_IDLE;

// active dispense
int      dSlot = -1;
uint32_t dStart = 0, dDuration = 0, dLastDraw = 0, dLastNotify = 0;
uint16_t dGrams = 0;
bool     dIsTest = false;          // TEST / RUN from config mode
uint32_t modeSince = 0;

// idle screen rotation
uint8_t  idleIdx = 0;
uint32_t idleLast = 0;

// ------------------------------------------------------------ LCD custom chars
byte barChars[5][8];
byte rupee[8] = {0b11111, 0b00010, 0b11111, 0b00010, 0b11100, 0b00110, 0b00011, 0b00001};
byte heart[8] = {0b00000, 0b01010, 0b11111, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000};
byte tick[8]  = {0b00000, 0b00001, 0b00011, 0b10110, 0b11100, 0b01000, 0b00000, 0b00000};
#define CH_RUPEE 5
#define CH_HEART 6
#define CH_TICK  7

void makeChars() {
  for (int n = 0; n < 5; n++) {
    byte row = 0;
    for (int b = 0; b <= n; b++) row |= (0b10000 >> b);
    for (int r = 0; r < 8; r++) barChars[n][r] = (r == 0 || r == 7) ? 0 : row;
    lcd.createChar(n, barChars[n]);
  }
  lcd.createChar(CH_RUPEE, rupee);
  lcd.createChar(CH_HEART, heart);
  lcd.createChar(CH_TICK, tick);
}

void lcdLine(uint8_t row, const String &text) {   // centred text, padded to 20 chars
  String t = text.substring(0, 20);
  int pad = (20 - t.length()) / 2;
  lcd.setCursor(0, row);
  for (int i = 0; i < pad; i++) lcd.print(' ');
  lcd.print(t);
  for (int i = pad + t.length(); i < 20; i++) lcd.print(' ');
}

void lcdBar(uint8_t row, float pct) {           // 20 cells x 5 columns = 100 steps
  int cols = constrain((int)(pct + 0.5f), 0, 100);
  lcd.setCursor(0, row);
  for (int c = 0; c < 20; c++) {
    int fill = cols - c * 5;
    if (fill >= 5) lcd.write((uint8_t)4);
    else if (fill > 0) lcd.write((uint8_t)(fill - 1));
    else lcd.print(' ');
  }
}

String gramsText(uint16_t g) {
  if (g >= 1000) {
    char b[12];
    if (g % 1000 == 0) snprintf(b, sizeof b, "%d kg", g / 1000);
    else snprintf(b, sizeof b, "%.2f kg", g / 1000.0);
    return String(b);
  }
  return String(g) + " g";
}

// ------------------------------------------------------------ screens
void showIdle() {
  lcd.clear();
  lcdLine(0, "** AnnaSeva **");
  idleLast = 0;                                 // draw product lines right away
  lcdLine(3, bleConnected ? "Screen connected" : "Waiting for screen");
}

void tickIdle() {
  if (millis() - idleLast < 2500) return;
  idleLast = millis();
  Slot &s = slots[idleIdx];                     // rotates through the 3 products, always the latest saved name
  lcdLine(1, s.name);
  String price = String((int)s.pricePerKg) + " per kg";
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor((20 - (price.length() + 2)) / 2, 2);
  lcd.write((uint8_t)CH_RUPEE);
  lcd.print(' ');
  lcd.print(price);
  idleIdx = (idleIdx + 1) % 3;
}

void showOrder(int s, uint16_t grams, uint32_t paise) {
  lcd.clear();
  lcdLine(0, slots[s].name);
  lcdLine(1, "Qty: " + gramsText(grams));
  lcd.setCursor(0, 2);
  char amt[16];
  snprintf(amt, sizeof amt, "%lu.%02lu", (unsigned long)(paise / 100), (unsigned long)(paise % 100));
  int len = 8 + strlen(amt);
  lcd.setCursor((20 - len) / 2, 2);
  lcd.print("Pay: ");
  lcd.write((uint8_t)CH_RUPEE);
  lcd.print(' ');
  lcd.print(amt);
  lcdLine(3, "Scan & pay on screen");
}

void showDispenseStart() {
  lcd.clear();
  lcdLine(0, dIsTest ? "TEST DISPENSE" : "Dispensing...");
  lcdLine(1, String(slots[dSlot].name).substring(0, 20));
  lcdLine(3, dGrams ? gramsText(dGrams) : String(dDuration) + " ms");
  lcdBar(2, 0);
}

void showThanks() {
  lcd.clear();
  lcd.setCursor(4, 0); lcd.write((uint8_t)CH_TICK); lcd.print(" Thank you! "); lcd.write((uint8_t)CH_TICK);
  lcdLine(1, "Please collect your");
  lcdLine(2, "ration bag");
  lcd.setCursor(1, 3); lcd.print("Visit again soon "); lcd.write((uint8_t)CH_HEART);
}

void showConfig() {
  lcd.clear();
  lcdLine(0, "== CONFIG MODE ==");
  lcdLine(1, "Operator connected");
  lcdLine(2, "Calibrate on screen");
  lcdLine(3, "FW " FW_VERSION);
}

void setMode(Mode m) {
  mode = m;
  modeSince = millis();
  switch (m) {
    case M_IDLE:   showIdle();   break;
    case M_THANKS: showThanks(); break;
    case M_CONFIG: showConfig(); break;
    default: break;
  }
}

// ------------------------------------------------------------ servos
void servoTo(int i, int angle, bool hold) {
  if (!servos[i].attached()) servos[i].attach(SERVO_PINS[i], 500, 2400);
  servos[i].write(constrain(angle, 0, 180));
  detachAt[i] = (hold || !DETACH_IDLE_SERVOS) ? 0 : millis() + 800;
}

void closeAll() {
  for (int i = 0; i < 3; i++) servoTo(i, slots[i].closeAngle, false);
}

// ------------------------------------------------------------ storage
void loadSlots() {
  prefs.begin("annaseva", true);
  for (int i = 0; i < 3; i++) {
    char key[8]; snprintf(key, sizeof key, "slot%d", i);
    if (prefs.getBytesLength(key) == sizeof(Slot)) {
      Slot saved;
      prefs.getBytes(key, &saved, sizeof(Slot));
      // 90/0 were the old placeholder angles: keep the calibrated defaults instead
      if (!(saved.openAngle == 90 && saved.closeAngle == 0)) slots[i] = saved;
      else { slots[i].pricePerKg = saved.pricePerKg; memcpy(slots[i].name, saved.name, sizeof(saved.name)); }
    }
  }
  prefs.end();
}

void saveSlot(int i) {
  prefs.begin("annaseva", false);
  char key[8]; snprintf(key, sizeof key, "slot%d", i);
  prefs.putBytes(key, &slots[i], sizeof(Slot));
  prefs.end();
}

// ------------------------------------------------------------ BLE
void notify(const String &msg) {
  if (!bleConnected || !txChar) return;
  String m = msg + "\n";
  txChar->setValue((uint8_t *)m.c_str(), m.length());
  txChar->notify();
  delay(12);                                     // give the stack room between notifications
}

void sendConfig() {
  for (int i = 0; i < 3; i++) {
    char b[80];
    snprintf(b, sizeof b, "CFG:%d:%d:%d:%d:%.2f:%s", i + 1, slots[i].openAngle, slots[i].closeAngle,
             slots[i].msPer100g, slots[i].pricePerKg, slots[i].name);
    notify(b);
  }
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { bleConnected = true; bleChanged = true; }
  void onDisconnect(BLEServer *s) override { bleConnected = false; bleChanged = true; }
};

class RxCB : public BLECharacteristicCallbacks {
  String buf;
  void onWrite(BLECharacteristic *c) override {
    // getValue() is std::string on core 2.x and String on core 3.x; both have c_str()
    buf += String(c->getValue().c_str());
    int nl;
    while ((nl = buf.indexOf('\n')) >= 0) {
      String line = buf.substring(0, nl);
      buf.remove(0, nl + 1);
      line.trim();
      if (line.length()) {
        char item[CMD_LEN];
        line.toCharArray(item, CMD_LEN);
        xQueueSend(cmdQueue, item, 0);
      }
    }
    // a write without a trailing newline (e.g. from nRF Connect) is one whole command
    buf.trim();
    if (buf.length()) {
      char item[CMD_LEN];
      buf.toCharArray(item, CMD_LEN);
      xQueueSend(cmdQueue, item, 0);
    }
    buf = "";
  }
};

void setupBle() {
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(185);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService *svc = server->createService(SVC_UUID);
  txChar = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// ------------------------------------------------------------ dispensing
bool startDispense(int s, uint16_t grams, uint32_t ms, bool test) {
  if (mode == M_DISPENSE) { notify("BUSY"); return false; }
  if (s < 0 || s > 2) { notify("ERR:bad slot"); return false; }
  if (grams) ms = (uint32_t)slots[s].msPer100g * grams / 100;
  if (ms == 0) { notify("ERR:zero time"); return false; }
  if (ms > MAX_OPEN_MS) ms = MAX_OPEN_MS;
  modeBeforeDispense = (mode == M_CONFIG) ? M_CONFIG : M_IDLE;
  dSlot = s; dGrams = grams; dDuration = ms; dIsTest = test;
  dStart = millis(); dLastDraw = 0; dLastNotify = 0;
  mode = M_DISPENSE;
  showDispenseStart();
  servoTo(s, slots[s].openAngle, true);
  notify("START:" + String(s + 1) + ":" + String(ms));
  Serial.printf("Dispense slot %d, %u g, %lu ms%s\n", s + 1, grams, (unsigned long)ms, test ? " (test)" : "");
  return true;
}

void tickDispense() {
  uint32_t el = millis() - dStart;
  float pct = min(100.0f, el * 100.0f / dDuration);
  if (millis() - dLastDraw > 150) { dLastDraw = millis(); lcdBar(2, pct); }
  if (millis() - dLastNotify > 400) { dLastNotify = millis(); notify("PROG:" + String((int)pct)); }
  if (el >= dDuration) {
    servoTo(dSlot, slots[dSlot].closeAngle, false);
    lcdBar(2, 100);
    notify("PROG:100");
    notify("DONE:" + String(dSlot + 1));
    if (dIsTest) { delay(600); setMode(modeBeforeDispense); }
    else setMode(M_THANKS);
    dSlot = -1;
  }
}

void emergencyStop() {
  if (mode == M_DISPENSE) {
    notify("DONE:" + String(dSlot + 1));
    dSlot = -1;
    setMode(modeBeforeDispense);
  }
  closeAll();
}

// ------------------------------------------------------------ commands
// Splits "CMD:a:b:c" into parts; the last part keeps any extra ':'
int splitCmd(char *line, char *parts[], int maxParts) {
  int n = 0;
  parts[n++] = line;
  for (char *p = line; *p && n < maxParts; p++) {
    if (*p == ':') { *p = 0; parts[n++] = p + 1; }
  }
  return n;
}

void handleCommand(char *line) {
  Serial.print("BLE > "); Serial.println(line);
  char *a[8];
  int n = splitCmd(line, a, 8);
  String cmd = a[0];
  int s = (n > 1) ? atoi(a[1]) - 1 : -1;
  bool slotOk = s >= 0 && s < 3;

  if (cmd == "PING") notify("PONG");
  else if (cmd == "GET") { notify("HELLO:" FW_VERSION); sendConfig(); }
  else if (cmd == "HOME") { if (mode != M_DISPENSE && mode != M_CONFIG) setMode(M_IDLE); }
  else if (cmd == "MODE" && n > 1) {
    if (mode == M_DISPENSE) return;
    if (!strcmp(a[1], "CFG")) setMode(M_CONFIG);
    else { closeAll(); setMode(M_IDLE); }            // leaving config: park every gate closed
  }
  else if (cmd == "SEL" && n >= 4 && slotOk) {
    if (mode == M_IDLE || mode == M_ORDER) { mode = M_ORDER; modeSince = millis(); showOrder(s, atoi(a[2]), strtoul(a[3], nullptr, 10)); }
  }
  else if (cmd == "DISP" && n >= 3 && slotOk) startDispense(s, constrain(atoi(a[2]), 1, 20000), 0, false);
  else if (cmd == "TEST" && n >= 3 && slotOk) startDispense(s, constrain(atoi(a[2]), 1, 20000), 0, true);
  else if (cmd == "RUN"  && n >= 3 && slotOk) startDispense(s, 0, constrain(atol(a[2]), 50, 60000), true);
  else if (cmd == "ANG"  && n >= 3 && slotOk) { if (mode != M_DISPENSE) servoTo(s, atoi(a[2]), true); }
  else if (cmd == "OPEN" && slotOk) { if (mode != M_DISPENSE) servoTo(s, slots[s].openAngle, true); }
  else if (cmd == "CLOSE" && slotOk) { if (mode != M_DISPENSE) servoTo(s, slots[s].closeAngle, false); }
  else if (cmd == "STOP") emergencyStop();
  else if (cmd == "CAL" && n >= 5 && slotOk) {
    slots[s].openAngle  = constrain(atoi(a[2]), 0, 180);
    slots[s].closeAngle = constrain(atoi(a[3]), 0, 180);
    slots[s].msPer100g  = constrain(atoi(a[4]), 50, 20000);
    saveSlot(s);
    notify("OK:CAL");
  }
  else if (cmd == "PRD" && n >= 4 && slotOk) {
    slots[s].pricePerKg = max(0.0f, (float)atof(a[2]));
    strncpy(slots[s].name, a[3], sizeof(slots[s].name) - 1);
    slots[s].name[sizeof(slots[s].name) - 1] = 0;
    saveSlot(s);
    idleIdx = s; idleLast = 0;                       // show the renamed product on the LCD right away
    notify("OK:PRD");
  }
  else if (cmd == "SAVE") { notify("OK:SAVE"); sendConfig(); }
  else notify("ERR:unknown " + cmd);
}

// ------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  Serial.println("\nAnnaSeva Ration ATM v" FW_VERSION);

  // power-on: close every gate first, before anything else
  loadSlots();
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  for (int i = 0; i < 3; i++) servos[i].setPeriodHertz(50);
  closeAll();

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  makeChars();
  lcdLine(0, "AnnaSeva");
  lcdLine(1, "by CognoSpace");
  lcdLine(3, "Starting...");


  cmdQueue = xQueueCreate(12, CMD_LEN);
  setupBle();
  delay(1200);
  closeAll();                                     // and once more after boot, in case a servo was slow to respond
  setMode(M_IDLE);
}

void loop() {
  // BLE connect / disconnect
  if (bleChanged) {
    bleChanged = false;
    if (!bleConnected) {
      delay(300);
      BLEDevice::startAdvertising();            // let the screen reconnect
      if (mode == M_CONFIG || mode == M_ORDER) setMode(M_IDLE);
      if (mode != M_DISPENSE) closeAll();       // screen gone: make sure no gate is left open
    }
    if (mode == M_IDLE) lcdLine(3, bleConnected ? "Screen connected" : "Waiting for screen");
  }

  // commands from the page
  char item[CMD_LEN];
  while (xQueueReceive(cmdQueue, item, 0) == pdTRUE) handleCommand(item);

  // state machine
  switch (mode) {
    case M_IDLE:     tickIdle(); break;
    case M_ORDER:    if (millis() - modeSince > ORDER_IDLE_MS) setMode(M_IDLE); break;
    case M_DISPENSE: tickDispense(); break;
    case M_THANKS:   if (millis() - modeSince > THANKS_MS) setMode(M_IDLE); break;
    case M_CONFIG:   break;
  }

  // detach idle servos
  for (int i = 0; i < 3; i++) {
    if (detachAt[i] && millis() > detachAt[i]) { servos[i].detach(); detachAt[i] = 0; }
  }
  delay(5);
}
