#include <RFM69.h>
#include <RFM69registers.h>
#include <SPI.h>

#define NODEID      99
#define NETWORKID   21
#define FREQUENCY   RF69_433MHZ
#define KEY         "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define ACK_TIME    30  // # of ms to wait for an ack
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!

#define RESET_PIN 23
#define RFM_INT_PIN 16

#define MAX_INT (2147483647)

RFM69 radio(15, RFM_INT_PIN, true);

unsigned long clock;

static void checkReg(uint8_t reg, const char *name, uint8_t expected, const char *comment) {
  char buffer[128];
    uint8_t v = radio.readReg(reg);
    snprintf(buffer, 128, "%s\t0x%02x 0x%02x <=> 0x%02x %s %s", name, reg, v, expected, comment, 
      v == expected ? "" : "******");
    Serial.println(buffer);
}

void debugL(unsigned n) {
  unsigned mask = 0x1;
  int ledPin = 9;
  for(int i = 5; i; i--, --ledPin, mask <<= 1) {
    digitalWrite(ledPin, n & mask ? HIGH : LOW);
  }
}

#define RED_LED (9)
#define BLUE_LED (8)
#define WHITE_LED (7)

void setup() {
  for(int ledPin = 9; ledPin >= 5; --ledPin)
    pinMode(ledPin, OUTPUT);
  for(int i = 0; i <= 31; i++) {
    debugL(i);
    delay(40);
  }

  delay(3000);
  clock = millis();
  Serial.begin(115200);
  Serial.println("Starting...");
  delay(100);
  
  // reset RFM69
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(RESET_PIN, LOW);
  delay(5);
  
  radio.initialize(FREQUENCY, NODEID, NETWORKID);

  // Set speed to 100 kb/s (NB: Xtql freq is 32MHz vs bitrate is 32e6 / REG_BITRATE)
  radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_100000);
  radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_100000);

  // 433 MHz. For some reason, not set during initialization

  radio.writeReg(0x07, 0x6c);
  radio.writeReg(0x08, 0x40);
  radio.writeReg(0x09, 0x00);
  
  delay(100);
  radio.spyMode(true);
  radio.receiveDone(); // Go into RX mode
  Serial.print("Freq registers ");
  Serial.print(radio.readReg(REG_FRFMSB));
  Serial.print(" ");
  Serial.print(radio.readReg(REG_FRFMID));
  Serial.println("");
  
  radio.receiveDone(); // Actually put in RF69_MODE_RX
  char buffer[50];
  for(int reg = 1; reg <= 0x4f; reg++) {
    sprintf(buffer, "%02x %02x  ", reg, radio.readReg(reg));
    Serial.print(buffer);
    if (!(reg % 10)) Serial.println("");
  }
  
  Serial.println("Running");
  checkReg(1, "RegOpMode", 0x10, "RX mode");
  checkReg(2, "DataMode", 0x0, "packet mode / FSK / no shaping");
  checkReg(3, "bitrate/1", 1, "100 kb/s");
  checkReg(4, "bitrate/2", 0x40, "100 kb/s");
  checkReg(7, "freq/1", 0x6c, "433 MHz");
  checkReg(8, "freq/2", 0x40, "433 MHz");
  checkReg(9, "freq/3", 0x00, "433 MHz");
}

void closeString(char *buffer) {
  unsigned char *p = (unsigned char *)buffer;
  for(int n = RF69_MAX_DATA_LEN-1; n; --n, ++p) {
    if (*p < ' ' || *p > 127) break;
  }
  *p = '\0';
}

static unsigned long nn = 0;

static uint32_t read32(const char **pp) {
  const unsigned char *p = (const unsigned char *)*pp;
  uint32_t res = *p++;
  res = res << 8 | *p++;
  res = res << 8 | *p++;
  res = res << 8 | *p++;
  *pp = (const char *)p;
  return res;
}

static uint16_t read16(const char **pp) {
  const unsigned char *p = (const unsigned char *)*pp;
  uint32_t res = *p++;
  res = res << 8 | *p++;
  *pp = (const char *)p;
  return res;
}

static uint8_t read8(const char **pp) {
  const unsigned char *p = (const unsigned char *)*pp;
  uint8_t res = *p++;
  *pp = (const char *)p;
  return res;
}

char buffer[256];

uint8_t computeCRC(unsigned len, const char *pp) {
  uint8_t res = 0;
  const unsigned char *p = (const unsigned char *)pp;
  ++len;
  --p;
  while(--len) {
    res += *++p;
  }
  return res;
}

static unsigned long crcError = 0;

static void displayPacket() {
  int rssi = radio.RSSI;
  const char *p = (const char *)radio.DATA;
  if (*p == '>') {
    radio.DATA[min(radio.DATALEN, RF69_MAX_DATA_LEN-1)] = '\0';
//    closeString((char *)radio.DATA);
    Serial.println(p);
    return;
  }
  uint32_t _clock = read32(&p);
  uint16_t voltage = read16(&p);
  uint16_t voltageReading = read16(&p);
  uint16_t current = read16(&p);
  uint16_t currentReading = read16(&p);
  uint16_t powerBudget = read16(&p);
  uint16_t mppt_direction = read8(&p);
  uint16_t leftPower = read16(&p);
  uint16_t rightPower = read16(&p);
  uint32_t peakPower = read32(&p);
  uint16_t heading = read16(&p);
  uint8_t magneticHeading = read8(&p);
  uint8_t targetHeading = read8(&p);
  int16_t uNavPnt = read16(&p);
  int32_t lat = read32(&p);
  int32_t lon = read32(&p);
  int16_t remoteRSSI = read16(&p);
  int16_t lost = read16(&p);
  uint32_t badCommand = read32(&p);

  uint8_t crc = *p;
  uint8_t crcLocal = computeCRC(p - (const char *)radio.DATA, (const char *)radio.DATA);
  if (crcLocal != crc) {
    ++crcError;
    return;
  }
  
  snprintf(buffer, 256, "%ld V=%d %d I=%d %d P=%d MPPT=%d L=%d R=%d PP=%ld H=%d MH=%d TH=%d WP=%d %f _%f %ld v=%d ^=%d l=%d CE=%ld",
    _clock, voltage, voltageReading, current, currentReading, powerBudget, mppt_direction,
    leftPower, rightPower,
    peakPower, heading, magneticHeading, targetHeading, uNavPnt, lat * 180.0 / MAX_INT, lon * 180.0 / MAX_INT,
    badCommand, rssi, remoteRSSI, 
    lost, crcError);
  Serial.println(buffer);
}

static char commandLine[128];
static char *clp = commandLine;
static const char *clp_last = commandLine + 128 - 1;

static void processInput() {
  if (!Serial.available()) return;
  if (clp >= clp_last) {
      Serial.println("*** input buffer overflow");
      clp = commandLine;
      return;
  }
  char c = Serial.read();
  if (c == '\n') {
    Serial.print("###");
    radio.send('P', commandLine, clp - commandLine);
    clp = commandLine;
    return;
  }
  *clp++ = c;
  Serial.print(c);
  Serial.print('.');
}

void loop() {
  debugL(1);
  if (radio.receiveDone()) {
    debugL(2);
    displayPacket();
  }
  debugL(3);

  processInput();
  debugL(4);
  
  unsigned long now = millis();
  if (now > clock + 1000) {
    debugL(5);
    clock = now;
    char bb[100];
    sprintf(bb, "pi ng %ld", ++nn);
    radio.send('P', bb, strlen(bb));
  }
  delay(6);
}
