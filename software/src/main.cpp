#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

// --- Configuration ---
#define SWDIO_PIN 23
#define SWCLK_PIN 22
#define BUTTON_PIN 16
#define BUZZER_PIN 12               // active buzzer: drive HIGH to sound
#define LED_FAILURE_PIN 13
#define LED_HEARTBEAT_PIN LED_FAILURE_PIN // Using LED_FAILURE_PIN for heartbeat
#define FIRMWARE_PATH "/firmware.bin"
#define LOG_PATH "/uids.csv"

// Registers and Constants
#define STM32_UID_ADDR  0x1FFFF7E8
#define FLASH_F1_KEYR   0x40022004
#define FLASH_F1_SR     0x4002200C
#define FLASH_F1_CR     0x40022010
#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

#define DP_SELECT       0x08
#define DP_RDBUFF       0x0C
#define DP_CTRLSTAT     0x04
#define AP_CSW          0x00
#define AP_TAR          0x04
#define AP_DRW          0x0C

#define DHCSR_ADDR      0xE000EDF0
#define DHCSR_HALT      0xA05F0003  // DBGKEY | C_DEBUGEN | C_HALT

#define SWD_DELAY_US   0
#define SWD_NOPS       0

AsyncWebServer server(80);
bool isProgramming = false;
unsigned long lastHeartbeat = 0;
const uint32_t heartbeatInterval = 500; // Blink every 500ms

// --- SWD Protocol Implementation ---
#define SWCLK_H()    digitalWrite(SWCLK_PIN, HIGH)
#define SWCLK_L()    digitalWrite(SWCLK_PIN, LOW)
#define SWDIO_H()    digitalWrite(SWDIO_PIN, HIGH)
#define SWDIO_L()    digitalWrite(SWDIO_PIN, LOW)
#define SWDIO_SET(b) digitalWrite(SWDIO_PIN, (b) ? HIGH : LOW)
#define SWDIO_RD()   (digitalRead(SWDIO_PIN) != 0)
#define SWDIO_OE()   pinMode(SWDIO_PIN, OUTPUT)        // drive
#define SWDIO_IE()   pinMode(SWDIO_PIN, INPUT_PULLUP)  // release (pull-up holds high)

// Configure SWD pins once.
void swd_gpio_init() {
    pinMode(SWCLK_PIN, OUTPUT);
    SWCLK_L();
    SWDIO_OE();
    SWDIO_H();   // idle/park level
}

inline void swd_delay() {
    if (SWD_DELAY_US) delayMicroseconds(SWD_DELAY_US);
    for (volatile int i = 0; i < SWD_NOPS; i++) { /* burn ~25ns each */ }
}

inline void swd_clk() {
    swd_delay();
    SWCLK_H();
    swd_delay();
    SWCLK_L();
}

void swd_seq(uint32_t bits, int n) {
    SWDIO_OE();
    for (int i = 0; i < n; i++) {
        SWDIO_SET((bits >> i) & 1);
        swd_clk();
    }
}

void swd_line_reset() {
    swd_seq(0xFFFFFFFF, 32);
    swd_seq(0xFFFFFFFF, 32);
}

void swd_idle_clocks() {
    swd_seq(0, 12);
}

uint8_t swd_header(bool ap, bool read, uint8_t a32) {
    bool par = (ap ? 1 : 0) ^ (read ? 1 : 0) ^ (a32 & 1) ^ ((a32 >> 1) & 1);
    SWDIO_OE();
    SWDIO_H();              swd_clk(); // Start = 1
    SWDIO_SET(ap);          swd_clk(); // APnDP
    SWDIO_SET(read);        swd_clk(); // RnW
    SWDIO_SET(a32 & 1);     swd_clk(); // A[2]
    SWDIO_SET((a32 >> 1) & 1); swd_clk(); // A[3]
    SWDIO_SET(par);         swd_clk(); // Parity
    SWDIO_L();              swd_clk(); // Stop = 0
    SWDIO_IE();                        // Park: release line
    swd_clk();                         // Park cycle
    swd_clk();                         // Turnaround
    uint8_t ack = 0;
    if (SWDIO_RD()) ack |= 1; swd_clk(); // ACK[0]
    if (SWDIO_RD()) ack |= 2; swd_clk(); // ACK[1]
    if (SWDIO_RD()) ack |= 4; swd_clk(); // ACK[2] + trailing clock
    return ack;
}

uint8_t swd_xfer(bool ap, bool read, uint8_t a32, uint32_t *val) {
    uint8_t ack = swd_header(ap, read, a32);
    if (ack != 1) {
        if (ack != 2 && ack != 4) for (int i = 0; i < 33; i++) swd_clk();
        swd_seq(0, 8);
        return ack;
    }

    if (read) { // RDATA[0] already on the line
        uint32_t v = 0; uint8_t ones = 0;
        for (int i = 0; i < 32; i++) {
            if (SWDIO_RD()) { v |= (1UL << i); ones++; }
            swd_clk();
        }
        uint8_t pbit = SWDIO_RD() ? 1 : 0;
        swd_clk();
        swd_seq(0, 8);                 // turnaround back to host + idle
        if ((ones & 1) != pbit) return 0xFF;
        if (val) *val = v;
    } else { // one turnaround, then host drives WDATA
        swd_clk();
        SWDIO_OE();
        uint32_t v = val ? *val : 0; uint8_t ones = 0;
        for (int i = 0; i < 32; i++) {
            bool b = (v >> i) & 1;
            SWDIO_SET(b);
            if (b) ones++;
            swd_clk();
        }
        SWDIO_SET(ones & 1);           // even parity
        swd_clk();
        swd_seq(0, 8);                 // idle
    }
    return 1;
}

// Clear DP sticky error flags via ABORT (always accepted, even in FAULT state).
void swd_clear_errors() {
    uint32_t ab = 0x1E; // ORUNERRCLR|WDERRCLR|STKERRCLR|STKCMPCLR
    swd_xfer(false, false, 0, &ab);
}

bool swd_read_reg(bool ap, uint8_t addr, uint32_t *val) {
    uint8_t a32 = (addr >> 2) & 3;
    for (int attempt = 0; attempt < 16; attempt++) {
        uint8_t ack = swd_xfer(ap, true, a32, val);
        if (ack == 1) return true;
        if (ack == 4) swd_clear_errors();   // FAULT: clear sticky flags before retry
    }
    Serial.printf("[SWD] read giving up ap=%d a32=%d\n", ap, a32);
    return false;
}

bool swd_write_reg(bool ap, uint8_t addr, uint32_t val) {
    uint8_t a32 = (addr >> 2) & 3;
    for (int attempt = 0; attempt < 16; attempt++) {
        uint8_t ack = swd_xfer(ap, false, a32, &val);
        if (ack == 1) return true;
        if (ack == 4) swd_clear_errors();
    }
    Serial.printf("[SWD] write giving up ap=%d a32=%d\n", ap, a32);
    return false;
}

bool swd_mem_access(uint32_t addr, uint32_t *val, bool write, uint32_t size_bits) {
    uint32_t csw = 0x23000000 | (size_bits == 16 ? 1 : 2); // 1=16bit, 2=32bit
    if (!swd_write_reg(false, DP_SELECT, 0)) return false;
    if (!swd_write_reg(true, AP_CSW, csw)) return false;
    if (!swd_write_reg(true, AP_TAR, addr)) return false;
    if (write) return swd_write_reg(true, AP_DRW, *val);
    uint32_t dummy;
    if (!swd_read_reg(true, AP_DRW, &dummy)) return false;
    return swd_read_reg(false, DP_RDBUFF, val);
}

bool ap_write(uint32_t addr, uint32_t val) {
    if (!swd_write_reg(true, AP_TAR, addr)) return false;
    return swd_write_reg(true, AP_DRW, val);
}
bool ap_read(uint32_t addr, uint32_t *val) {
    if (!swd_write_reg(true, AP_TAR, addr)) return false;
    uint32_t dummy;
    if (!swd_read_reg(true, AP_DRW, &dummy)) return false;
    return swd_read_reg(false, DP_RDBUFF, val);
}

bool init_swd_connection() {
    swd_gpio_init();

    // line reset -> 16-bit select sequence -> line reset -> >=12 idle zeros.
    swd_line_reset();
    swd_seq(0xE79E, 16); // 0x79E7 MSB-first == 0xE79E LSB-first
    swd_line_reset();
    swd_idle_clocks();

    // First transaction after reset MUST be a DPIDR read; it never returns FAULT.
    uint32_t idcode = 0;
    if (!swd_read_reg(false, 0x00, &idcode)) {
        Serial.println("No IDCODE — check wiring (SWDIO=PA13, SWCLK=PA14), pull-up on SWDIO, and SWD_DELAY_US");
        return false;
    }
    Serial.printf("IDCODE: 0x%08X\n", idcode);

    // Clear any sticky error flags (ABORT: ORUNERRCLR|WDERRCLR|STKERRCLR|STKCMPCLR)
    swd_write_reg(false, 0x00, 0x1E);

    if (!swd_write_reg(false, DP_CTRLSTAT, 0x50000000)) {
        Serial.println("CTRL/STAT write failed");
        return false;
    }
    uint32_t ctrl = 0;
    uint32_t timeout = 1000;
    bool printed = false;
    while (timeout--) {
        if (!swd_read_reg(false, DP_CTRLSTAT, &ctrl)) {
            Serial.println("CTRL/STAT read failed");
            return false;
        }
        if (!printed) { Serial.printf("CTRL/STAT = 0x%08X\n", ctrl); printed = true; }
        if ((ctrl & 0xA0000000) == 0xA0000000) {
            Serial.println("Powered up.");
            return true;
        }
        delay(1);
    }
    Serial.printf("Power-up ACK timeout, CTRL/STAT = 0x%08X\n", ctrl);
    return false;
}

// timeout_ms: use a large value for mass erase (up to ~40s on STM32F1)
bool wait_flash_busy(uint32_t timeout_ms = 1000) {
    uint32_t sr = 0;
    while (timeout_ms--) {
        if (!swd_mem_access(FLASH_F1_SR, &sr, false, 32)) return false;
        if (!(sr & 0x01)) return true; // BSY cleared
        delay(1);
    }
    return false;
}

bool unlock_flash() {
    uint32_t key1 = FLASH_KEY1;
    uint32_t key2 = FLASH_KEY2;
    if (!swd_mem_access(FLASH_F1_KEYR, &key1, true, 32)) return false;
    if (!swd_mem_access(FLASH_F1_KEYR, &key2, true, 32)) return false;
    return true;
}

String get_stm32_uid() {
    uint32_t u1, u2, u3;
    if (swd_mem_access(STM32_UID_ADDR, &u1, false, 32) &&
        swd_mem_access(STM32_UID_ADDR + 4, &u2, false, 32) &&
        swd_mem_access(STM32_UID_ADDR + 8, &u3, false, 32)) {
        char buf[32];
        sprintf(buf, "%08X%08X%08X", u1, u2, u3);
        return String(buf);
    }
    return "UNKNOWN_UID";
}

bool flash_relink(uint32_t csw16) {
    Serial.println("[SWD] frame desync - re-syncing link");
    if (!init_swd_connection()) return false;      // line reset, switch, IDCODE, power-up
    uint32_t dhcsr = DHCSR_HALT;                    // re-halt (debug power may have cycled)
    for (int i = 0; i < 50; i++) {
        swd_mem_access(DHCSR_ADDR, &dhcsr, true, 32);
        uint32_t s = 0;
        if (swd_mem_access(DHCSR_ADDR, &s, false, 32) && (s & (1UL << 17))) break;
    }
    swd_clear_errors();
    swd_write_reg(false, DP_SELECT, 0);            // restore AP context
    swd_write_reg(true, AP_CSW, csw16);
    uint32_t pg = 0x01;
    ap_write(FLASH_F1_CR, pg);                     // re-assert PG
    return true;
}

bool flash_stm32(String filePath) {
    File file = LittleFS.open(filePath, "r");
    if (!file) return false;

    Serial.println("Starting Flash Process...");

    uint32_t dhcsr = DHCSR_HALT;
    bool halted = false;
    for (int i = 0; i < 50; i++) {
        swd_mem_access(DHCSR_ADDR, &dhcsr, true, 32);  // assert C_HALT | C_DEBUGEN
        uint32_t s = 0;
        if (swd_mem_access(DHCSR_ADDR, &s, false, 32) && (s & (1UL << 17))) { // S_HALT
            halted = true;
            break;
        }
    }
    if (!halted) {
        Serial.println("Failed to halt CPU");
        file.close();
        return false;
    }
    Serial.println("CPU halted.");
    swd_clear_errors();  // clear anything the pre-halt run may have set

    if (!unlock_flash()) { file.close(); return false; }

    if (!wait_flash_busy(500)) { file.close(); return false; }
    uint32_t cr_val = 0x04; // MER
    swd_mem_access(FLASH_F1_CR, &cr_val, true, 32);
    cr_val = 0x44; // MER | STRT
    swd_mem_access(FLASH_F1_CR, &cr_val, true, 32);
    if (!wait_flash_busy(60000)) { // 60s timeout for mass erase
        Serial.println("Mass erase timeout");
        file.close();
        return false;
    }

    swd_clear_errors();                          // clear anything mass-erase left set
    swd_write_reg(false, DP_SELECT, 0);          // AP bank 0 (once)
    uint32_t csw16 = 0x23000001;                 // debug master, privileged, 16-bit access
    swd_write_reg(true, AP_CSW, csw16);
    cr_val = 0x01;                               // PG: programming enable (stays set)
    ap_write(FLASH_F1_CR, cr_val);

    uint32_t addr = 0x08000000;
    size_t total = file.size();
    size_t written = 0;
    const size_t BLOCK_SIZE = 1024;       // one STM32F1 flash page = progress unit
    size_t nextBlock = BLOCK_SIZE;
    Serial.printf("Programming %u bytes -> 0x08000000\n", (unsigned)total);

    while (file.available()) {
        uint16_t hw = 0xFFFF;              // pad a trailing odd byte with 0xFF
        int n = file.read((uint8_t*)&hw, 2);
        if (n <= 0) break;

        uint32_t data = ((uint32_t)hw << 16) | hw;

        bool ok = false;
        for (int attempt = 0; attempt < 4 && !ok; attempt++) {
            if (attempt > 0) {
                if (!flash_relink(csw16)) { file.close(); return false; }
                uint32_t rb = 0;
                if (ap_read(addr, &rb)) {
                    uint16_t got = (addr & 2) ? (rb >> 16) : (rb & 0xFFFF);
                    if (got == hw) { ok = true; break; }   // already written
                }
            }
            if (!ap_write(addr, data)) continue;           // write failed -> relink & retry

            // Wait for BSY=0 AND read error bits from the SAME status read.
            uint32_t sr = 0; bool busyCleared = false;
            for (int i = 0; i < 2000; i++) {
                if (!ap_read(FLASH_F1_SR, &sr)) break;     // read failed -> relink & retry
                if (!(sr & 0x01)) { busyCleared = true; break; }
            }
            if (!busyCleared) continue;                    // timeout/read fail -> relink & retry
            if (sr & 0x14) { // PGERR (bit2) | WRPRTERR (bit4) — a real programming error
                Serial.printf("Flash error SR=0x%04X at 0x%08X\n", sr, addr);
                file.close(); return false;
            }
            ok = true;
        }
        if (!ok) { Serial.printf("Write fail at 0x%08X\n", addr); file.close(); return false; }

        addr += 2;
        written += (n < 2) ? n : 2;

        // One block done (or final bytes): print address + percent, quick LED blink
        if (written >= nextBlock || !file.available()) {
            unsigned pct = total ? (unsigned)((written * 100) / total) : 100;
            Serial.printf("  0x%08X  %6u / %u bytes  (%3u%%)\n",
                          addr, (unsigned)written, (unsigned)total, pct);
            digitalWrite(LED_FAILURE_PIN, HIGH);
            delay(8);
            digitalWrite(LED_FAILURE_PIN, LOW);
            nextBlock += BLOCK_SIZE;
        }
    }

    cr_val = 0x80;                        // LOCK
    ap_write(FLASH_F1_CR, cr_val);

    Serial.printf("Programming complete: %u bytes, ended at 0x%08X\n",
                  (unsigned)written, addr);
    file.close();
    return true;
}

bool verify_stm32(String filePath) {
    File file = LittleFS.open(filePath, "r");
    if (!file) return false;

    swd_write_reg(false, DP_SELECT, 0);          // AP bank 0
    uint32_t csw32 = 0x23000002;                 // 32-bit access for fast word reads
    swd_write_reg(true, AP_CSW, csw32);

    uint32_t addr = 0x08000000;
    size_t total = file.size(), checked = 0, nextReport = 8192;
    Serial.printf("Verifying %u bytes...\n", (unsigned)total);

    while (file.available()) {
        uint32_t expected = 0xFFFFFFFF;          // unread tail bytes are erased 0xFF
        int n = file.read((uint8_t*)&expected, 4);
        if (n <= 0) break;

        uint32_t got = 0;
        if (!ap_read(addr, &got)) {
            Serial.printf("Verify read fail at 0x%08X\n", addr);
            file.close(); return false;
        }
        if (got != expected) {
            Serial.printf("VERIFY MISMATCH at 0x%08X: flash=0x%08X expected=0x%08X\n",
                          addr, got, expected);
            file.close(); return false;
        }
        addr += 4;
        checked += (n < 4) ? n : 4;
        if (checked >= nextReport) {
            Serial.printf("  verified %u / %u bytes\n", (unsigned)checked, (unsigned)total);
            nextReport += 8192;
        }
    }
    file.close();
    Serial.printf("Verify OK: %u bytes match.\n", (unsigned)checked);
    return true;
}

void stm32_run() {
    uint32_t v;
    v = 0x00000000; swd_mem_access(0xE000EDFC, &v, true, 32); // DEMCR: clear vector catch
    v = 0x05FA0004; swd_mem_access(0xE000ED0C, &v, true, 32); // AIRCR: SYSRESETREQ (reboot)
    delay(10);                                                 // let the reset complete
    v = 0xA05F0000; swd_mem_access(DHCSR_ADDR, &v, true, 32);  // DHCSR: C_HALT=0, C_DEBUGEN=0
    Serial.println("Target reset and running.");
}

void log_uid(String uid) {
    File file = LittleFS.open(LOG_PATH, FILE_APPEND);
    if (file) {
        file.printf("%lu,%s\n", millis(), uid.c_str());
        file.close();
        Serial.println("UID logged: " + uid);
    }
}

void buzzer_beep(int count, int on_ms, int off_ms) {
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(on_ms);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < count - 1) delay(off_ms);
    }
}

void run_programming_sequence() {
    if (isProgramming) return;
    isProgramming = true;
    Serial.println("Starting sequence...");

    digitalWrite(BUZZER_PIN, LOW);

    if (init_swd_connection()) {
        String uid = get_stm32_uid();
        Serial.println("Target UID: " + uid);
        if (flash_stm32(FIRMWARE_PATH) && verify_stm32(FIRMWARE_PATH)) {
            stm32_run();
            log_uid(uid);
            Serial.println("Success!");
            buzzer_beep(2, 120, 100);
        } else {
            Serial.println("Failed to program/verify Flash.");
            buzzer_beep(1, 800, 0); 
        }
    } else {
        Serial.println("Failed to connect to STM32.");
        buzzer_beep(1, 800, 0); 
    }
    isProgramming = false;
}

// --- Web Interface HTML ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP32 SWD Programmer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin: 0 auto; padding-top: 30px; }
    .btn { background-color: #008CBA; color: white; padding: 10px 20px; text-decoration: none; border-radius: 4px; }
    .log-btn { background-color: #4CAF50; }
  </style>
</head><body>
  <h1>STM32 Programmer</h1>
  <form method="POST" action="/upload" enctype="multipart/form-data">
    <input type="file" name="update">
    <input type="submit" value="Upload Firmware">
  </form>
  <br><br>
  <a href="/download" class="btn log-btn">Download UID Log (CSV)</a>
  <p>Press the physical button on IO16 to start programming.</p>
</body></html>
)rawliteral";

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_FAILURE_PIN, OUTPUT);
    digitalWrite(LED_FAILURE_PIN, LOW);
    pinMode(LED_HEARTBEAT_PIN, OUTPUT);

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }

    WiFi.softAP("ESP32_Programmer", "12345678");
    Serial.println("AP Started. IP: " + WiFi.softAPIP().toString());

    // Web Server Routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", index_html);
    });

    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", (LittleFS.exists(FIRMWARE_PATH)) ? "Upload Success" : "Upload Failed");
    }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
        if (!index) {
            request->_tempFile = LittleFS.open(FIRMWARE_PATH, "w");
        }
        if (request->_tempFile) {
            request->_tempFile.write(data, len);
        }
        if (final) {
            request->_tempFile.close();
        }
    });

    server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, LOG_PATH, "text/csv");
    });

    server.begin();
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastHeartbeat >= heartbeatInterval) {
        lastHeartbeat = currentMillis;
        digitalWrite(LED_HEARTBEAT_PIN, !digitalRead(LED_HEARTBEAT_PIN));
    }

    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(50); // Debounce
        if (digitalRead(BUTTON_PIN) == LOW) {
            run_programming_sequence();
            while(digitalRead(BUTTON_PIN) == LOW); // Wait for release
        }
    }
}
