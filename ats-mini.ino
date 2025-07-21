#include <Wire.h>
#include <TFT_eSPI.h>
#include <SI4735-fixed.h>
#include <Rotary.h>
#include "patch_init.h"

#define PIN_POWER_ON     15
#define RESET_PIN        16
#define PIN_AMP_EN       10
#define ESP32_I2C_SCL    17
#define ESP32_I2C_SDA    18
#define ENCODER_PIN_A    2
#define ENCODER_PIN_B    1
#define ENCODER_BTN      21
#define PIN_LCD_BL       38
#define BFO_STEP_HZ      25
#define UI_W             320
#define UI_H             170

const char* mod_names[] = {"FM", "LSB", "USB", "AM"};
enum Modulation {FM, LSB, USB, AM};

struct Band {
  const char* name;
  uint16_t min_freq, max_freq, default_freq;
  Modulation default_mod;
  bool ssb_allowed;
};

const Band bands[] = {
  {"LW", 150, 520, 279, AM, false},
  {"MW", 520, 1710, 1000, AM, false},
  {"SW 1.7-4", 1710, 4000, 3570, LSB, true},
  {"SW 4-8", 4000, 8000, 7074, USB, true},
  {"SW 8-15", 8000, 15000, 14270, USB, true},
  {"SW 15-30", 15000, 30000, 21200, USB, true},
  {"FM", 6400, 10800, 9950, FM, false}
};
const int NUM_BANDS = sizeof(bands) / sizeof(bands[0]);

struct State {
  int band = 3;
  uint16_t freq = bands[3].default_freq;
  Modulation mod = bands[3].default_mod;
  int bfo = 0;
  int step = 1, stepFM = 10;
  int vol = 35;
};
State radio, menu;

enum MenuItem { M_BAND, M_VOL, M_MOD, M_STEP, M_EXIT };
const int MENU_ITEMS = 5;

SI4735_fixed rx;
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
Rotary encoder(ENCODER_PIN_B, ENCODER_PIN_A);
volatile int encoderChange = 0;
bool inMenu = false;
int selectedMenuItem = 0;
bool needsRedraw = true;

void ICACHE_RAM_ATTR onEncoder();
void setRadio();
void showMenu(), showMain();
void handleEncoder(), handleButton();
void changeBand(int), changeVol(int), changeMod(int), changeStep(int), changeFreq(int);

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP); 
  pinMode(ENCODER_PIN_B, INPUT_PULLUP); 
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  pinMode(PIN_AMP_EN, OUTPUT);
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  digitalWrite(PIN_AMP_EN, LOW);

  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL);
  tft.begin();
  tft.setRotation(3);
  spr.createSprite(UI_W, UI_H);

  ledcAttach(PIN_LCD_BL, 16000, 8);
  ledcWrite(PIN_LCD_BL, 200);

  spr.fillSprite(TFT_BLACK);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(TFT_WHITE);
  spr.drawString("Searching for SI4735...", UI_W/2, UI_H/2, 4);
  spr.pushSprite(0, 0);

  if (!rx.getDeviceI2CAddress(RESET_PIN)) {
    spr.drawString("SI4735 not found!", UI_W/2, UI_H/2, 4);
    spr.pushSprite(0, 0);
    while(true);
  }
  rx.setup(RESET_PIN, 0);
  delay(500);
  setRadio();
  digitalWrite(PIN_AMP_EN, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), onEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), onEncoder, CHANGE);
}

void loop() {
  if (encoderChange != 0) {
    handleEncoder(); 
    needsRedraw = true;
  }
  handleButton();
  if (needsRedraw) {
    if (inMenu) showMenu(); else showMain();
    needsRedraw = false;
  }
  delay(10);
}

void ICACHE_RAM_ATTR onEncoder() {
  unsigned char res = encoder.process();
  if (res == DIR_CW)      encoderChange = 1;
  else if (res == DIR_CCW) encoderChange = -1;
}

void handleEncoder() {
  noInterrupts();
  int dir = encoderChange;
  encoderChange = 0;
  interrupts();
  
  if (dir == 0) return;

  if (inMenu) {
    int direction = (dir > 0) ? 1 : -1;
    switch (selectedMenuItem) {
      case M_BAND: changeBand(direction); break;
      case M_VOL:  changeVol(direction); break;
      case M_MOD:  changeMod(direction); break;
      case M_STEP: changeStep(direction); break;
      case M_EXIT:
        inMenu = false; radio = menu; radio.freq = bands[radio.band].default_freq; setRadio();
        break;
    }
  } else {
    changeFreq(dir);
  }
}

void handleButton() {
  static unsigned long last = 0;
  if (digitalRead(ENCODER_BTN) == LOW && millis() - last > 250) {
    if (!inMenu) {
      inMenu = true; menu = radio; selectedMenuItem = 0;
    } else {
      selectedMenuItem = (selectedMenuItem + 1) % MENU_ITEMS;
    }
    needsRedraw = true;
    last = millis();
  }
}

void changeFreq(int dir) {
  auto &r = radio;
  const Band* b = &bands[r.band];
  
  if (r.mod == LSB || r.mod == USB) {
    uint16_t old_freq = r.freq;
    r.bfo += dir * BFO_STEP_HZ;
    while (r.bfo >= 500) {
      r.freq++;
      r.bfo -= 1000;
    }
    while (r.bfo <= -500) {
      r.freq--;
      r.bfo += 1000;
    }
    if (r.freq != old_freq) {
      if (r.freq > b->max_freq) r.freq = b->min_freq;
      if (r.freq < b->min_freq) r.freq = b->max_freq;
      rx.setFrequency(r.freq);
    }
    rx.setSSBBfo(-r.bfo);
  } else {
    int step = (r.mod == FM) ? r.stepFM : r.step;
    r.freq += dir * step;
    if (r.freq > b->max_freq) r.freq = b->min_freq;
    if (r.freq < b->min_freq) r.freq = b->max_freq;
    rx.setFrequency(r.freq);
  }
}

void changeBand(int d) {
  menu.band = (menu.band + d + NUM_BANDS) % NUM_BANDS;
  menu.mod = bands[menu.band].default_mod;
}

void changeVol(int d) {
    menu.vol = constrain(menu.vol + d, 0, 63);
    rx.setVolume(menu.vol);
}

void changeMod(int d) {
  auto b = bands[menu.band];
  if (!b.ssb_allowed) return;
  int m = (int)menu.mod;
  if (d > 0) menu.mod = (m >= AM) ? LSB : (Modulation)(m + 1);
  else       menu.mod = (m <= LSB) ? AM : (Modulation)(m - 1);
}

void changeStep(int d) {
  if (bands[menu.band].default_mod == FM) {
    const int steps[] = {1, 5, 10, 20}; int idx=0; for(int i=0;i<4;i++) if(menu.stepFM==steps[i]) idx=i;
    idx = (idx + d + 4) % 4; menu.stepFM = steps[idx];
  } else {
    const int steps[] = {1, 5, 9, 10}; int idx=0; for(int i=0;i<4;i++) if(menu.step==steps[i]) idx=i;
    idx = (idx + d + 4) % 4; menu.step = steps[idx];
  }
}

void showMain() {
  spr.fillSprite(TFT_BLACK); spr.setTextFont(7); spr.setTextDatum(TL_DATUM); spr.setTextColor(TFT_WHITE);
  char freq[16], unit[5];
  if (radio.mod == FM) { sprintf(freq, "%.2f", radio.freq / 100.0); strcpy(unit, "MHz"); }
  else {
    float df = radio.freq + (radio.bfo / 1000.0);
    (radio.bfo == 0) ? sprintf(freq, "%u", radio.freq) : sprintf(freq, "%.2f", df);
    strcpy(unit, "kHz");
  }
  spr.drawString(freq, 10, 20); spr.setTextFont(4); spr.drawString(unit, 270, 55);
  spr.setTextFont(2);
  spr.drawString(String("Band: ") + bands[radio.band].name, 10, 90);
  spr.drawString(String("Mode: ") + mod_names[radio.mod], 10, 110);
  int stepNow = (radio.mod == FM) ? radio.stepFM * 10 : radio.step;
  spr.drawString(String("Step: ") + stepNow + "kHz", 10, 130);
  spr.drawString(String("Volume: ") + radio.vol, 10, 150);
  spr.pushSprite(0, 0);
}

void showMenu() {
  spr.fillSprite(TFT_BLACK); spr.setTextDatum(TL_DATUM); spr.setTextFont(4);
  String items[MENU_ITEMS];
  items[M_BAND] = "Band: " + String(bands[menu.band].name);
  items[M_VOL]  = "Volume: " + String(menu.vol);
  items[M_MOD]  = "Mode: "   + String(mod_names[menu.mod]);
  int stepNow = (bands[menu.band].default_mod == FM) ? menu.stepFM * 10 : menu.step;
  items[M_STEP] = "Step: "   + String(stepNow) + "kHz";
  items[M_EXIT] = "Set & Exit";
  for(int i=0;i<MENU_ITEMS;i++) {
    spr.setTextColor(selectedMenuItem == i ? TFT_BLACK : TFT_WHITE, selectedMenuItem == i ? TFT_WHITE : TFT_BLACK);
    spr.drawString(items[i], 10, 10 + i*35);
  }
  spr.pushSprite(0, 0);
}

void setRadio() {
  digitalWrite(PIN_AMP_EN, LOW); delay(20);
  radio.bfo = 0; const Band* b = &bands[radio.band];
  if(radio.mod == FM) rx.setFM(b->min_freq, b->max_freq, radio.freq, radio.stepFM);
  else if(radio.mod == AM) rx.setAM(b->min_freq, b->max_freq, radio.freq, radio.step);
  else {
    rx.loadPatch(ssb_patch_content, sizeof(ssb_patch_content));
    uint8_t ssbm = (radio.mod == LSB) ? 1 : 2;
    rx.setSSB(b->min_freq, b->max_freq, radio.freq, radio.step, ssbm);
    rx.setSSBBfo(0);
  }
  rx.setVolume(radio.vol); delay(50);
  digitalWrite(PIN_AMP_EN, HIGH);
}
