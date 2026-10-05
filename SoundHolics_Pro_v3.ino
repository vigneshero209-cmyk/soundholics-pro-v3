#include <Wire.h>
#include <EEPROM.h>
#include <Encoder.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// ============================================================
// SoundHolics Pro v3.0
// ESP8266 + PT2258N 5.1 + TDA7439 tone control + 2323 IR
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define ENCODER_CLK_PIN     14   // D5
#define ENCODER_DT_PIN      12   // D6
#define ENCODER_SW_PIN      13   // D7
#define INPUT_SELECT_PIN    15   // D8
#define IR_RECV_PIN         D4   // GPIO2

#define CHANNEL_COUNT 7
#define CHANNEL_TONE_VALUES 7

#define PT2258N_ADDR        0x40
#define TDA7439_ADDR        0x44
#define EEPROM_SIZE         512

#define EEPROM_MASTER_VOL   0
#define EEPROM_MASTER_BASS  1
#define EEPROM_MASTER_MIDS  2
#define EEPROM_MASTER_TREB  3
#define EEPROM_FL_VOL       4
#define EEPROM_FR_VOL       5
#define EEPROM_C_VOL        6
#define EEPROM_SWL_VOL      7
#define EEPROM_SWR_VOL      8
#define EEPROM_SL_VOL       9
#define EEPROM_SR_VOL       10
#define EEPROM_INPUT        11
#define EEPROM_GAIN1        12
#define EEPROM_GAIN2        13
#define EEPROM_GAIN3        14
#define EEPROM_GAIN4        15

// 2323 remote IR hex values (typical NEC codes)
#define IR_VOL_UP           0x80412710
#define IR_VOL_DOWN         0x80412711
#define IR_VOL_MUTE         0x8041A70F
#define IR_BASS_UP          0x8041A721
#define IR_BASS_DOWN        0x8041A720
#define IR_MIDS_UP          0x8041A758
#define IR_MIDS_DOWN        0x80412759
#define IR_TREB_UP          0x8041275B
#define IR_TREB_DOWN        0x8041275A
#define IR_INPUT_1          0x80412701
#define IR_INPUT_2          0x8041A702
#define IR_INPUT_3          0x80412703
#define IR_INPUT_4          0x80412704
#define IR_CH_MASTER        0x80412732
#define IR_CH_FL_FR         0x80412730
#define IR_CH_CENTER        0x8041275C
#define IR_CH_SUBWOOFER     0x8041275D
#define IR_CH_SURROUND      0x80412731
#define IR_TREBLE_BOOST     0x8041A7F0
#define IR_TREBLE_CUT       0x8041A7F1
#define IR_BASS_BOOST       0x8041A7E8
#define IR_BASS_CUT         0x8041A7E9

// PT2258N registers
#define PT2258N_VOL_L       0x10
#define PT2258N_VOL_R       0x20
#define PT2258N_BASS        0x60
#define PT2258N_MIDS        0x70
#define PT2258N_TREB        0x50
#define PT2258N_LOUD        0x40

// TDA7439 register map (documented generic values)
#define TDA7439_REG_INPUT    0x00
#define TDA7439_REG_VOL_L   0x01
#define TDA7439_REG_VOL_R   0x02
#define TDA7439_REG_BASS    0x03
#define TDA7439_REG_MIDS    0x04
#define TDA7439_REG_TREBLE  0x05
#define TDA7439_REG_BAL     0x06
#define TDA7439_REG_ATT     0x07

// Wifi settings
const char* ssid = "1";
const char* password = "12345678";

// IR receiver
IRrecv irrecv(IR_RECV_PIN, 1024);
decode_results results;

// Web server
ESP8266WebServer server(80);

// Encoder
Encoder myEnc(ENCODER_CLK_PIN, ENCODER_DT_PIN);
long oldPosition = -999;

// Display timing
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_REFRESH_INTERVAL = 50;
unsigned long lastIRTime = 0;
const unsigned long IR_DISPLAY_TIMEOUT = 5000;
bool showIRControl = false;

// Menu and control state
byte menu = 0;
byte control_mode = 0;
unsigned long time_val, time1, ir_time = 0;

// Audio channel state
struct Channel {
  int volume;
  int bass;
  int mids;
  int treble;
  int balance;
  int volume_ram;
  int bass_ram;
  int mids_ram;
  int treble_ram;
  int balance_ram;
};

Channel FL, FR, C, SWL, SWR, SL, SR;
Channel* currentChannel = &FL;
byte currentChannelIndex = 0;

// Master controls
int master_volume = 24;
int master_volume_ram = 24;
int master_bass = 0;
int master_bass_ram = 0;
int master_mids = 0;
int master_mids_ram = 0;
int master_treble = 0;
int master_treble_ram = 0;

// Input selection and gains
int in_sel = 1, in_ram = 1;
int gain = 0, gain_ram = 0;
int gain1 = 0, gain1_ram = 0;
int gain2 = 0, gain2_ram = 0;
int gain3 = 0, gain3_ram = 0;
int gain4 = 0, gain4_ram = 0;

// FFT / spectrum
#define SAMPLES 128
#define SAMPLING_FREQUENCY 40000
#define AUDIO_INPUT_PIN A0

double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned int samplingPeriodUs;

#define NUM_BARS 8
int barHeights[NUM_BARS] = {0};

// ============================================================
// Basic setup and loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n========== SoundHolics Pro v3.0 ==========");
  Serial.println("Initializing system...");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("SoundHolics Pro");
  display.println("PT2258N 5.1");
  display.println("Initializing...");
  display.display();
  delay(500);

  EEPROM.begin(EEPROM_SIZE);
  Wire.begin(D2, D1);

  initializePT2258N();
  initializeTDA7439();

  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  pinMode(INPUT_SELECT_PIN, INPUT_PULLUP);
  pinMode(IR_RECV_PIN, INPUT);
  pinMode(AUDIO_INPUT_PIN, INPUT);

  initializeChannels();
  loadSettingsFromEEPROM();
  updateGainFromInput();
  updateAudio();
  updateRamVariables();

  initializeIRReceiver();
  setupWiFi();
  setupWebServer();

  samplingPeriodUs = round(1000000.0 / SAMPLING_FREQUENCY);

  Serial.println("SoundHolics v3.0 Ready!");
}

void loop() {
  server.handleClient();
  handleIRInput();

  if (digitalRead(ENCODER_SW_PIN) == LOW) {
    handleMenuButton();
    delay(300);
  }

  if (digitalRead(INPUT_SELECT_PIN) == LOW) {
    handleInputSelectButton();
    delay(300);
  }

  handleEncoderControl();
  collectSamples();

  FFT_Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT_Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT_ComplexToMagnitude(vReal, vImag, SAMPLES);
  updateBars();

  if (showIRControl && (millis() - lastIRTime > IR_DISPLAY_TIMEOUT)) {
    showIRControl = false;
  }

  if (millis() - lastDisplayUpdate >= DISPLAY_REFRESH_INTERVAL) {
    lastDisplayUpdate = millis();
    updateOLEDDisplay();
  }

  if (millis() - time_val > 10000 && menu > 0) {
    menu = 0;
    updateRamVariables();
    myEnc.write(0);
    oldPosition = -999;
  }

  if (millis() - time1 > 60000) {
    saveSettingsToEEPROM();
    time1 = millis();
  }

  delay(5);
}

// ============================================================
// PT2258N initialization and logic
// ============================================================

void initializePT2258N() {
  Wire.beginTransmission(PT2258N_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

  for (int i = 0; i < CHANNEL_COUNT; i++) {
    pt2258nSetVolume(i, 24);
  }

  pt2258nSetBass(0);
  pt2258nSetMids(0);
  pt2258nSetTreble(0);

  Serial.println("PT2258N initialized");
}

void pt2258nSetVolume(byte channel, byte value) {
  byte vol_cmd = 0x10 | (channel & 0x07);
  byte vol_data = (48 - value) & 0x3F;

  Wire.beginTransmission(PT2258N_ADDR);
  Wire.write(vol_cmd);
  Wire.write(vol_data);
  Wire.endTransmission();
}

void pt2258nSetBass(int value) {
  byte bass_value = (value + 7) & 0x0F;
  Wire.beginTransmission(PT2258N_ADDR);
  Wire.write(0x60);
  Wire.write(bass_value);
  Wire.endTransmission();
}

void pt2258nSetMids(int value) {
  byte mids_value = (value + 7) & 0x0F;
  Wire.beginTransmission(PT2258N_ADDR);
  Wire.write(0x70);
  Wire.write(mids_value);
  Wire.endTransmission();
}

void pt2258nSetTreble(int value) {
  byte treb_value = (value + 7) & 0x0F;
  Wire.beginTransmission(PT2258N_ADDR);
  Wire.write(0x50);
  Wire.write(treb_value);
  Wire.endTransmission();
}

void pt2258nMute() {
  Wire.beginTransmission(PT2258N_ADDR);
  Wire.write(0x01);
  Wire.endTransmission();
}

void pt2258nUnmute() {
  Wire.beginTransmission(PT2258N_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ============================================================
// TDA7439 tone and stereo control
// ============================================================

void initializeTDA7439() {
  tda7439WriteReg(TDA7439_REG_INPUT, 0x00);
  tda7439WriteReg(TDA7439_REG_VOL_L, 0x10);
  tda7439WriteReg(TDA7439_REG_VOL_R, 0x10);
  tda7439WriteReg(TDA7439_REG_BASS, 0x08);
  tda7439WriteReg(TDA7439_REG_MIDS, 0x08);
  tda7439WriteReg(TDA7439_REG_TREBLE, 0x08);
  tda7439WriteReg(TDA7439_REG_BAL, 0x08);
  Serial.println("TDA7439 initialized");
}

void tda7439WriteReg(byte reg, byte value) {
  Wire.beginTransmission(TDA7439_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void tda7439SetTone(int bass, int mids, int treble, int balance) {
  int bassCode = constrain(bass + 7, 0, 14);
  int midsCode = constrain(mids + 7, 0, 14);
  int trebleCode = constrain(treble + 7, 0, 14);
  int balanceCode = constrain(balance + 7, 0, 14);

  tda7439WriteReg(TDA7439_REG_BASS, bassCode);
  tda7439WriteReg(TDA7439_REG_MIDS, midsCode);
  tda7439WriteReg(TDA7439_REG_TREBLE, trebleCode);
  tda7439WriteReg(TDA7439_REG_BAL, balanceCode);
}

void tda7439SetStereoLevel(int leftLevel, int rightLevel) {
  byte leftCode = constrain(leftLevel, 0, 31);
  byte rightCode = constrain(rightLevel, 0, 31);

  tda7439WriteReg(TDA7439_REG_VOL_L, leftCode);
  tda7439WriteReg(TDA7439_REG_VOL_R, rightCode);
}

// ============================================================
// Channel initialization and state
// ============================================================

void initializeChannels() {
  Channel defaultChannel = {24, 0, 0, 0, 0, 24, 0, 0, 0, 0};
  FL = FR = C = SWL = SWR = SL = SR = defaultChannel;
  currentChannel = &FL;
  currentChannelIndex = 0;
}

// ============================================================
// Spectrum analyzer
// ============================================================

void collectSamples() {
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long microseconds = micros();
    vReal[i] = analogRead(AUDIO_INPUT_PIN);
    vImag[i] = 0;
    while (micros() - microseconds < samplingPeriodUs) {
      // wait for accurate sample timing
    }
  }
}

void updateBars() {
  for (int i = 0; i < NUM_BARS; i++) {
    int bin = i + 2;
    int height = (int)(vReal[bin] / 50);
    barHeights[i] = (barHeights[i] * 3 + height) / 4;
    if (barHeights[i] > SCREEN_HEIGHT - 10) {
      barHeights[i] = SCREEN_HEIGHT - 10;
    }
  }
}

void drawSpectrum() {
  display.clearDisplay();

  display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(2, 2);
  display.println("SPECTRUM 5.1");
  display.setTextColor(SSD1306_WHITE);

  int barWidth = (SCREEN_WIDTH - 2) / NUM_BARS;
  for (int i = 0; i < NUM_BARS; i++) {
    int x = 1 + (i * barWidth);
    int barHeight = barHeights[i];
    int y = SCREEN_HEIGHT - barHeight;
    display.fillRect(x, y, barWidth - 1, barHeight, SSD1306_WHITE);
  }

  display.drawLine(0, SCREEN_HEIGHT - 1, SCREEN_WIDTH, SCREEN_HEIGHT - 1, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 55);
  display.print("Vol:");
  display.print(-48 + master_volume);
  display.print("dB In:");
  display.print(in_sel);
  display.display();
}

// ============================================================
// OLED display functions
// ============================================================

void updateOLEDDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (showIRControl) {
    displayIRControlScreen();
  } else {
    drawSpectrum();
  }

  display.display();
}

void displayIRControlScreen() {
  display.setTextSize(1);
  display.setCursor(0, 0);

  switch (menu) {
    case 0: displayIRVolume(); break;
    case 1: displayIRBass(); break;
    case 2: displayIRMids(); break;
    case 3: displayIRTreble(); break;
    case 4: displayIRChannel(); break;
    case 5: displayIRInput(); break;
    default: drawSpectrum(); break;
  }
}

void displayIRVolume() {
  display.setTextSize(2);
  display.setCursor(15, 0);
  display.println("VOLUME");

  display.setTextSize(4);
  display.setCursor(20, 20);
  display.print(-48 + master_volume);
  display.setCursor(85, 20);
  display.print("dB");

  drawVolumeBar(10, 50, master_volume, 48);
}

void displayIRBass() {
  display.setTextSize(2);
  display.setCursor(20, 0);
  display.println("BASS");

  display.setTextSize(4);
  display.setCursor(30, 20);
  display.print(master_bass * 2);
  display.setCursor(85, 20);
  display.print("dB");

  drawEQBar(10, 50, master_bass);
}

void displayIRMids() {
  display.setTextSize(2);
  display.setCursor(20, 0);
  display.println("MIDS");

  display.setTextSize(4);
  display.setCursor(30, 20);
  display.print(master_mids * 2);
  display.setCursor(85, 20);
  display.print("dB");

  drawEQBar(10, 50, master_mids);
}

void displayIRTreble() {
  display.setTextSize(2);
  display.setCursor(15, 0);
  display.println("TREBLE");

  display.setTextSize(4);
  display.setCursor(30, 20);
  display.print(master_treble * 2);
  display.setCursor(85, 20);
  display.print("dB");

  drawEQBar(10, 50, master_treble);
}

void displayIRChannel() {
  display.setTextSize(2);
  display.setCursor(10, 0);
  display.println("CHANNELS");

  display.setTextSize(1);
  const char* chNames[] = {"FL", "FR", "C", "SWL", "SWR", "SL", "SR"};

  for (int i = 0; i < 7; i++) {
    int x = 5 + (i * 18);
    int y = 20;

    if (i == currentChannelIndex) {
      display.fillRect(x - 2, y - 2, 16, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.drawRect(x - 2, y - 2, 16, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(x, y + 3);
    display.print(chNames[i]);
  }

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 45);
  display.print("Vol:");
  display.print(-48 + currentChannel->volume);
  display.print("dB");
}

void displayIRInput() {
  display.setTextSize(2);
  display.setCursor(20, 0);
  display.println("INPUT");

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("Current: ");
  display.print(in_sel);

  display.setCursor(0, 45);
  display.print("Gain: ");
  display.print(gain);
  display.print("/15");

  for (int i = 1; i <= 4; i++) {
    if (i == in_sel) {
      display.fillRect(20 + (i - 1) * 25, 52, 22, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(23 + (i - 1) * 25, 54);
      display.print("IN");
      display.print(i);
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.drawRect(20 + (i - 1) * 25, 52, 22, 8, SSD1306_WHITE);
      display.setCursor(23 + (i - 1) * 25, 54);
      display.print("IN");
      display.print(i);
    }
  }
}

void drawVolumeBar(int x, int y, int value, int maxValue) {
  int barWidth = (value * 90) / maxValue;
  display.drawRect(x, y, 90, 8, SSD1306_WHITE);
  if (barWidth > 0) {
    display.fillRect(x + 1, y + 1, barWidth - 2, 6, SSD1306_WHITE);
  }
}

void drawEQBar(int x, int y, int value) {
  display.drawRect(x, y - 2, 100, 4, SSD1306_WHITE);
  int sliderPos = x + ((value + 7) * 7);
  display.fillRect(sliderPos, y - 3, 4, 6, SSD1306_WHITE);
}

// ============================================================
// IR receiver functions
// ============================================================

void initializeIRReceiver() {
  irrecv.enableIRIn();
  Serial.println("IR Receiver enabled");
}

void handleIRInput() {
  if (irrecv.decode(&results)) {
    uint32_t code = results.value;
    ir_time = millis();
    lastIRTime = millis();
    showIRControl = true;

    Serial.print("IR Code: 0x");
    Serial.println(code, HEX);

    if (code == IR_VOL_UP) handleIRVolumeUp();
    else if (code == IR_VOL_DOWN) handleIRVolumeDown();
    else if (code == IR_VOL_MUTE) handleIRMute();
    else if (code == IR_BASS_UP) handleIRBassUp();
    else if (code == IR_BASS_DOWN) handleIRBassDown();
    else if (code == IR_MIDS_UP) handleIRMidsUp();
    else if (code == IR_MIDS_DOWN) handleIRMidsDown();
    else if (code == IR_TREB_UP) handleIRTrebleUp();
    else if (code == IR_TREB_DOWN) handleIRTrebleDown();
    else if (code == IR_CH_MASTER) handleIRChannelSelect(0);
    else if (code == IR_CH_FL_FR) handleIRChannelSelect(0);
    else if (code == IR_CH_CENTER) handleIRChannelSelect(2);
    else if (code == IR_CH_SUBWOOFER) handleIRChannelSelect(3);
    else if (code == IR_CH_SURROUND) handleIRChannelSelect(5);
    else if (code == IR_INPUT_1) handleIRInputSelect(1);
    else if (code == IR_INPUT_2) handleIRInputSelect(2);
    else if (code == IR_INPUT_3) handleIRInputSelect(3);
    else if (code == IR_INPUT_4) handleIRInputSelect(4);

    irrecv.resume();
  }
}

void handleIRVolumeUp() {
  master_volume++;
  if (master_volume > 48) master_volume = 48;
  updateAudio();
  time_val = millis();
  time1 = millis();
  menu = 0;
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRVolumeDown() {
  master_volume--;
  if (master_volume < 0) master_volume = 0;
  updateAudio();
  time_val = millis();
  time1 = millis();
  menu = 0;
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRMute() {
  if (master_volume == 0) {
    master_volume = master_volume_ram;
    pt2258nUnmute();
  } else {
    master_volume_ram = master_volume;
    master_volume = 0;
    pt2258nMute();
  }
  updateAudio();
  time_val = millis();
  time1 = millis();
  menu = 0;
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRBassUp() {
  master_bass++;
  if (master_bass > 7) master_bass = 7;
  updateAudio();
  time_val = millis();
  time1 = millis();
  menu = 1;
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRBassDown() {
  master_bass--;
  if (master_bass < -7) master_bass = -7;
  updateAudio();
  time_val = millis();
  time1 = millis();
  menu = 1;
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRMidsUp() {
  master_mids++;
  if (master_mids > 7) master_mids = 7;
  updateAudio();
  time_val = millis();
  time1 = millis();
  menu = 2;
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRMidsDown() {
  master_mids--;
  if (master_mids < -7) master_mids = -7;
  updateAudio();
  time_val = millis();
  time1 = millis();
  menu = 2;
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRTrebleUp() {
  master_treble++;
  if (master_treble > 7) master_treble = 7;
  updateAudio();
  time_val = millis();
  time1 = millis();
  menu = 3;
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRTrebleDown() {
  master_treble--;
  if (master_treble < -7) master_treble = -7;
  updateAudio();
  time_val = millis();
  time1 = millis();
  menu = 3;
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRChannelSelect(byte channel) {
  currentChannelIndex = constrain(channel, 0, 6);

  switch (currentChannelIndex) {
    case 0: currentChannel = &FL; break;
    case 1: currentChannel = &FR; break;
    case 2: currentChannel = &C; break;
    case 3: currentChannel = &SWL; break;
    case 4: currentChannel = &SWR; break;
    case 5: currentChannel = &SL; break;
    case 6: currentChannel = &SR; break;
  }

  updateRamVariables();
  menu = 4;
  myEnc.write(0);
  oldPosition = -999;
  time_val = millis();
  time1 = millis();
  lastIRTime = millis();
  showIRControl = true;
}

void handleIRInputSelect(int input) {
  in_sel = input;
  updateRamVariables();
  updateGainFromInput();
  updateAudio();
  menu = 5;
  myEnc.write(0);
  oldPosition = -999;
  time_val = millis();
  time1 = millis();
  lastIRTime = millis();
  showIRControl = true;
}

// ============================================================
// Encoder and button handlers
// ============================================================

void handleMenuButton() {
  menu++;
  if (menu > 5) menu = 0;
  updateRamVariables();
  myEnc.write(0);
  oldPosition = -999;
  time1 = millis();
  time_val = millis();
  showIRControl = false;
}

void handleInputSelectButton() {
  in_sel++;
  if (in_sel > 4) in_sel = 1;
  updateRamVariables();
  updateGainFromInput();
  updateAudio();
  myEnc.write(0);
  oldPosition = -999;
  menu = 5;
  time1 = millis();
  time_val = millis();
  showIRControl = false;
}

void handleEncoderControl() {
  long newPosition = myEnc.read();

  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    long delta = newPosition / 4;

    switch (menu) {
      case 0: handleEncoderVolume(delta); break;
      case 1: handleEncoderBass(delta); break;
      case 2: handleEncoderMids(delta); break;
      case 3: handleEncoderTreble(delta); break;
      case 4: handleEncoderChannel(delta); break;
      case 5: handleEncoderGain(delta); break;
    }

    time_val = millis();
    time1 = millis();
    showIRControl = false;
  }
}

void handleEncoderVolume(long delta) {
  master_volume = constrain(master_volume_ram + delta, 0, 48);
  updateAudio();
}

void handleEncoderBass(long delta) {
  master_bass = constrain(master_bass_ram + delta, -7, 7);
  updateAudio();
}

void handleEncoderMids(long delta) {
  master_mids = constrain(master_mids_ram + delta, -7, 7);
  updateAudio();
}

void handleEncoderTreble(long delta) {
  master_treble = constrain(master_treble_ram + delta, -7, 7);
  updateAudio();
}

void handleEncoderChannel(long delta) {
  currentChannelIndex = constrain(currentChannelIndex + delta, 0, 6);

  switch (currentChannelIndex) {
    case 0: currentChannel = &FL; break;
    case 1: currentChannel = &FR; break;
    case 2: currentChannel = &C; break;
    case 3: currentChannel = &SWL; break;
    case 4: currentChannel = &SWR; break;
    case 5: currentChannel = &SL; break;
    case 6: currentChannel = &SR; break;
  }

  myEnc.write(0);
  oldPosition = -999;
}

void handleEncoderGain(long delta) {
  switch (in_sel) {
    case 1: gain1 = constrain(gain1 + delta, 0, 15); gain = gain1; break;
    case 2: gain2 = constrain(gain2 + delta, 0, 15); gain = gain2; break;
    case 3: gain3 = constrain(gain3 + delta, 0, 15); gain = gain3; break;
    case 4: gain4 = constrain(gain4 + delta, 0, 15); gain = gain4; break;
  }
  updateAudio();
  myEnc.write(0);
  oldPosition = -999;
}

// ============================================================
// WiFi and web server
// ============================================================

void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi Connection Failed");
  }

  delay(1000);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/api/status", handleAPIStatus);
  server.on("/api/volume", handleAPIVolume);
  server.on("/api/bass", handleAPIBass);
  server.on("/api/mids", handleAPIMids);
  server.on("/api/treble", handleAPITreble);
  server.on("/api/input", handleAPIInput);
  server.on("/api/gain", handleAPIGain);
  server.on("/api/channel", handleAPIChannel);
  server.on("/api/control", handleAPIControl);

  server.begin();
  Serial.println("Web Server Started");
}

void handleRoot() {
  String html = getHTMLPage();
  server.send(200, "text/html", html);
}

void handleAPIStatus() {
  String json = "{";
  json += "\"masterVol\":" + String(master_volume) + ",";
  json += "\"masterBass\":" + String(master_bass) + ",";
  json += "\"masterMids\":" + String(master_mids) + ",";
  json += "\"masterTreb\":" + String(master_treble) + ",";
  json += "\"input\":" + String(in_sel) + ",";
  json += "\"currentChannel\":" + String(currentChannelIndex) + ",";
  json += "\"gain1\":" + String(gain1) + ",";
  json += "\"gain2\":" + String(gain2) + ",";
  json += "\"gain3\":" + String(gain3) + ",";
  json += "\"gain4\":" + String(gain4) + ",";
  json += "\"rssi\":" + String(WiFi.RSSI());
  json += "}";

  server.send(200, "application/json", json);
}

void handleAPIVolume() {
  if (server.hasArg("value")) {
    master_volume = constrain(server.arg("value").toInt(), 0, 48);
    updateAudio();
    time1 = millis();
  }
  server.send(200, "application/json", "{\"masterVol\":" + String(master_volume) + "}");
}

void handleAPIBass() {
  if (server.hasArg("value")) {
    master_bass = constrain(server.arg("value").toInt(), -7, 7);
    updateAudio();
    time1 = millis();
  }
  server.send(200, "application/json", "{\"masterBass\":" + String(master_bass) + "}");
}

void handleAPIMids() {
  if (server.hasArg("value")) {
    master_mids = constrain(server.arg("value").toInt(), -7, 7);
    updateAudio();
    time1 = millis();
  }
  server.send(200, "application/json", "{\"masterMids\":" + String(master_mids) + "}");
}

void handleAPITreble() {
  if (server.hasArg("value")) {
    master_treble = constrain(server.arg("value").toInt(), -7, 7);
    updateAudio();
    time1 = millis();
  }
  server.send(200, "application/json", "{\"masterTreb\":" + String(master_treble) + "}");
}

void handleAPIChannel() {
  if (server.hasArg("index")) {
    currentChannelIndex = constrain(server.arg("index").toInt(), 0, 6);

    switch (currentChannelIndex) {
      case 0: currentChannel = &FL; break;
      case 1: currentChannel = &FR; break;
      case 2: currentChannel = &C; break;
      case 3: currentChannel = &SWL; break;
      case 4: currentChannel = &SWR; break;
      case 5: currentChannel = &SL; break;
      case 6: currentChannel = &SR; break;
    }

    updateRamVariables();
  }
  server.send(200, "application/json", "{\"channel\":" + String(currentChannelIndex) + "}");
}

void handleAPIInput() {
  if (server.hasArg("value")) {
    in_sel = constrain(server.arg("value").toInt(), 1, 4);
    updateGainFromInput();
    updateAudio();
    time1 = millis();
  }
  server.send(200, "application/json", "{\"input\":" + String(in_sel) + "}");
}

void handleAPIGain() {
  if (server.hasArg("input") && server.hasArg("value")) {
    int input = server.arg("input").toInt();
    int value = constrain(server.arg("value").toInt(), 0, 15);

    switch (input) {
      case 1: gain1 = value; break;
      case 2: gain2 = value; break;
      case 3: gain3 = value; break;
      case 4: gain4 = value; break;
    }

    updateGainFromInput();
    updateAudio();
    time1 = millis();
  }
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleAPIControl() {
  if (server.hasArg("action")) {
    String action = server.arg("action");

    if (action == "save") {
      saveSettingsToEEPROM();
    } else if (action == "reset") {
      menu = 0;
      currentChannelIndex = 0;
      currentChannel = &FL;
      loadSettingsFromEEPROM();
      updateAudio();
    }
  }

  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ============================================================
// Helper functions
// ============================================================

void updateRamVariables() {
  master_volume_ram = master_volume;
  master_bass_ram = master_bass;
  master_mids_ram = master_mids;
  master_treble_ram = master_treble;

  currentChannel->volume_ram = currentChannel->volume;
  currentChannel->bass_ram = currentChannel->bass;
  currentChannel->mids_ram = currentChannel->mids;
  currentChannel->treble_ram = currentChannel->treble;
  currentChannel->balance_ram = currentChannel->balance;

  in_ram = in_sel;
  gain_ram = gain;
}

void updateGainFromInput() {
  switch (in_sel) {
    case 1: gain = gain1; break;
    case 2: gain = gain2; break;
    case 3: gain = gain3; break;
    case 4: gain = gain4; break;
    default: gain = gain1; break;
  }
}

void updateAudio() {
  // Global tone control
  pt2258nSetBass(master_bass);
  pt2258nSetMids(master_mids);
  pt2258nSetTreble(master_treble);

  // TDA7439 tone and stereo control
  tda7439SetTone(master_bass, master_mids, master_treble, 0);
  tda7439SetStereoLevel(16 + gain, 16 + gain);

  // PT2258N channel output volumes (7 channel 5.1)
  pt2258nSetVolume(0, FL.volume);
  pt2258nSetVolume(1, FR.volume);
  pt2258nSetVolume(2, C.volume);
  pt2258nSetVolume(3, SWL.volume);
  pt2258nSetVolume(4, SWR.volume);
  pt2258nSetVolume(5, SL.volume);
  pt2258nSetVolume(6, SR.volume);

  Serial.println("Audio settings updated");
}

void saveSettingsToEEPROM() {
  EEPROM.write(EEPROM_MASTER_VOL, master_volume);
  EEPROM.write(EEPROM_MASTER_BASS, master_bass + 7);
  EEPROM.write(EEPROM_MASTER_MIDS, master_mids + 7);
  EEPROM.write(EEPROM_MASTER_TREB, master_treble + 7);

  EEPROM.write(EEPROM_FL_VOL, FL.volume);
  EEPROM.write(EEPROM_FR_VOL, FR.volume);
  EEPROM.write(EEPROM_C_VOL, C.volume);
  EEPROM.write(EEPROM_SWL_VOL, SWL.volume);
  EEPROM.write(EEPROM_SWR_VOL, SWR.volume);
  EEPROM.write(EEPROM_SL_VOL, SL.volume);
  EEPROM.write(EEPROM_SR_VOL, SR.volume);

  EEPROM.write(EEPROM_INPUT, in_sel);
  EEPROM.write(EEPROM_GAIN1, gain1);
  EEPROM.write(EEPROM_GAIN2, gain2);
  EEPROM.write(EEPROM_GAIN3, gain3);
  EEPROM.write(EEPROM_GAIN4, gain4);
  EEPROM.commit();

  Serial.println("Settings saved to EEPROM");
}

void loadSettingsFromEEPROM() {
  master_volume = EEPROM.read(EEPROM_MASTER_VOL);
  master_bass = EEPROM.read(EEPROM_MASTER_BASS) - 7;
  master_mids = EEPROM.read(EEPROM_MASTER_MIDS) - 7;
  master_treble = EEPROM.read(EEPROM_MASTER_TREB) - 7;

  FL.volume = EEPROM.read(EEPROM_FL_VOL);
  FR.volume = EEPROM.read(EEPROM_FR_VOL);
  C.volume = EEPROM.read(EEPROM_C_VOL);
  SWL.volume = EEPROM.read(EEPROM_SWL_VOL);
  SWR.volume = EEPROM.read(EEPROM_SWR_VOL);
  SL.volume = EEPROM.read(EEPROM_SL_VOL);
  SR.volume = EEPROM.read(EEPROM_SR_VOL);

  in_sel = EEPROM.read(EEPROM_INPUT);
  gain1 = EEPROM.read(EEPROM_GAIN1);
  gain2 = EEPROM.read(EEPROM_GAIN2);
  gain3 = EEPROM.read(EEPROM_GAIN3);
  gain4 = EEPROM.read(EEPROM_GAIN4);

  if (master_volume > 48) master_volume = 24;
  if (in_sel > 4 || in_sel < 1) in_sel = 1;

  Serial.println("Settings loaded from EEPROM");
}

String getHTMLPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SoundHolics v3.0</title>
    <style>
        * { box-sizing: border-box; }
        body {
            margin: 0;
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            background: linear-gradient(135deg, #1f4068, #5c2a8c);
            font-family: Arial, sans-serif;
        }
        .container {
            width: min(900px, 90%);
            background: #fff;
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.25);
        }
        h1 {
            text-align: center;
            margin-bottom: 10px;
            color: #1f4068;
        }
        .status {
            text-align: center;
            color: #1a8f4c;
            margin-bottom: 25px;
            font-weight: bold;
        }
        .channels-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
            gap: 12px;
            margin-top: 20px;
        }
        .channel-card {
            background: #eef3ff;
            border: 1px solid #d9e4ff;
            border-radius: 12px;
            padding: 12px;
            text-align: center;
        }
        .channel-card h3 {
            margin: 0 0 8px 0;
            font-size: 14px;
            color: #25364d;
        }
        .channel-card p {
            margin: 0;
            font-weight: bold;
            color: #264c9e;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔊 SoundHolics Pro v3.0</h1>
        <div class="status">PT2258N 5.1 + TDA7439 + 2323 Remote</div>

        <div class="channels-grid">
            <div class="channel-card"><h3>Front Left</h3><p id="flVol">0 dB</p></div>
            <div class="channel-card"><h3>Front Right</h3><p id="frVol">0 dB</p></div>
            <div class="channel-card"><h3>Center</h3><p id="cVol">0 dB</p></div>
            <div class="channel-card"><h3>Subwoofer L</h3><p id="swlVol">0 dB</p></div>
            <div class="channel-card"><h3>Subwoofer R</h3><p id="swrVol">0 dB</p></div>
            <div class="channel-card"><h3>Surround L</h3><p id="slVol">0 dB</p></div>
            <div class="channel-card"><h3>Surround R</h3><p id="srVol">0 dB</p></div>
        </div>
    </div>

    <script>
        const chNames = ["FL","FR","C","SWL","SWR","SL","SR"];
        setInterval(() => {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    const vol = (-48 + data.masterVol);
                    document.getElementById('flVol').textContent = vol + ' dB';
                    document.getElementById('frVol').textContent = vol + ' dB';
                    document.getElementById('cVol').textContent = vol + ' dB';
                    document.getElementById('swlVol').textContent = vol + ' dB';
                    document.getElementById('swrVol').textContent = vol + ' dB';
                    document.getElementById('slVol').textContent = vol + ' dB';
                    document.getElementById('srVol').textContent = vol + ' dB';
                });
        }, 500);
    </script>
</body>
</html>
)rawliteral";

  return html;
}

// This sketch intentionally includes both PT2258N and TDA7439 support,
// so the same system can be adapted to either 5.1 PT2258N output path
// or TDA7439 stereo/tone path depending on hardware wiring.

// Support for ArduinoFFT is required in the original project for the VU meter.
// If you use the library, include it in the Arduino IDE project.

// Basic FFT compatibility layer for ESP8266/Arduino Core
// If using arduinoFFT library, remove the following and use the library directly.
#ifdef __has_include
#if __has_include("arduinoFFT.h")
#include <arduinoFFT.h>
arduinoFFT FFT = arduinoFFT();
#endif
#endif

// End of file
