#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <LovyanGFX.hpp>
#include <Preferences.h>
#include <time.h>
#include "LGFX_CrowPanel_5inch.h"
#include "secrets.h"

// ── Display ───────────────────────────────────────────────────────────────────
static LGFX_CrowPanel lcd;
#define canvas lcd

// ── Screen state ──────────────────────────────────────────────────────────────
enum Screen { SCREEN_MAIN, SCREEN_MANUAL, SCREEN_STATS };
static Screen        g_screen       = SCREEN_MAIN;
static unsigned long g_manual_entry = 0;
static const unsigned long MANUAL_TIMEOUT = 30000;

// ── Drum data ─────────────────────────────────────────────────────────────────
// volatile: written by Core-0 poll task, read by Core-1 UI loop.
static volatile int   g_level_pct = 0;
static volatile bool  g_pump_on   = false;
static volatile int   g_dist_mm   = -1;
static volatile bool  g_auto_mode = true;
static volatile bool  g_sensor_ok = false;
static volatile bool  g_reachable = false;
static volatile bool  g_dirty     = true;

// ── Command queue: UI (Core 1) → poll task (Core 0) ──────────────────────────
enum PendingCmd : uint8_t {
    CMD_NONE = 0,
    CMD_PUMP_ON, CMD_PUMP_OFF, CMD_AUTO,
    CMD_TIMEOUT_5, CMD_TIMEOUT_10, CMD_TIMEOUT_20, CMD_TIMEOUT_30
};
static volatile PendingCmd g_pending_cmd = CMD_NONE;

// ── Redraw tracking ───────────────────────────────────────────────────────────
static unsigned long g_last_manual_draw   = 0;
static unsigned long g_last_countdown_sec = 0xFFFFFFFF;
static unsigned long g_last_stats_draw    = 0;

// ── RTC (PCF8563, I2C 0x51) ───────────────────────────────────────────────────
// Wire times out after lcd.init() because the RGB LCD DMA interrupt starves the
// hardware I2C interrupt. Use lgfx::i2c (polling-based) instead — same driver
// that already works for the GT911 touch chip on the same bus.
static bool g_rtc_ok = false;

static uint8_t dec2bcd(uint8_t d) { return ((d / 10) << 4) | (d % 10); }
static uint8_t bcd2dec(uint8_t b) { return ((b >> 4) * 10) + (b & 0x0F); }

// Stores LOCAL Pacific time — clean mktime() roundtrip without UTC conversion.
static bool pcf8563_write(const struct tm* lt) {
    uint8_t buf[8] = {
        0x02,
        (uint8_t)(dec2bcd(lt->tm_sec)     & 0x7F), // clear VL flag
        (uint8_t)(dec2bcd(lt->tm_min)     & 0x7F),
        (uint8_t)(dec2bcd(lt->tm_hour)    & 0x3F),
        (uint8_t)(dec2bcd(lt->tm_mday)    & 0x3F),
        (uint8_t)(lt->tm_wday             & 0x07),
        (uint8_t)(dec2bcd(lt->tm_mon + 1) & 0x1F),
        (uint8_t) dec2bcd(lt->tm_year - 100)
    };
    return !lgfx::i2c::transactionWrite(0, 0x51, buf, 8, 400000u).has_error();
}

// Returns false if VL flag set (battery empty, time not valid).
static bool pcf8563_read(struct tm* lt) {
    const uint8_t reg = 0x02;
    uint8_t buf[7] = {};
    if (lgfx::i2c::transactionWriteRead(0, 0x51, &reg, 1, buf, 7, 400000u).has_error())
        return false;
    if (buf[0] & 0x80) return false;
    lt->tm_sec   = bcd2dec(buf[0] & 0x7F);
    lt->tm_min   = bcd2dec(buf[1] & 0x7F);
    lt->tm_hour  = bcd2dec(buf[2] & 0x3F);
    lt->tm_mday  = bcd2dec(buf[3] & 0x3F);
    lt->tm_wday  = buf[4] & 0x07;
    lt->tm_mon   = bcd2dec(buf[5] & 0x1F) - 1; // 0-based
    lt->tm_year  = bcd2dec(buf[6]) + 100;       // years since 1900
    lt->tm_isdst = -1;
    return true;
}

// ── Statistics (NVS-backed via Preferences) ───────────────────────────────────
static Preferences prefs;

struct Stats {
    uint32_t pump_cycles;    // total pump-on events
    uint32_t pump_total_sec; // cumulative seconds pumped
    uint32_t pump_min_sec;   // shortest run (0 = no data yet)
    uint32_t pump_max_sec;   // longest run
    uint32_t pump_last_sec;  // most recent run duration
    uint32_t fill_count;     // drum fill events (level rose ≥5%)
    uint32_t last_pump_ts;   // UTC epoch of last pump start (0 = never)
};
static Stats g_stats;

// Pump / fill tracking — Core 1 only, no cross-core races
static bool          g_prev_pump_on   = false;
static unsigned long g_pump_start_ms  = 0;
static int           g_prev_level_pct = 0;

static void loadStats() {
    prefs.begin("rwstats", true);
    g_stats.pump_cycles    = prefs.getUInt("cycles",  0);
    g_stats.pump_total_sec = prefs.getUInt("total_s", 0);
    g_stats.pump_min_sec   = prefs.getUInt("min_s",   0);
    g_stats.pump_max_sec   = prefs.getUInt("max_s",   0);
    g_stats.pump_last_sec  = prefs.getUInt("last_s",  0);
    g_stats.fill_count     = prefs.getUInt("fills",   0);
    g_stats.last_pump_ts   = prefs.getUInt("last_ts", 0);
    prefs.end();
}

static void saveStats() {
    prefs.begin("rwstats", false);
    prefs.putUInt("cycles",  g_stats.pump_cycles);
    prefs.putUInt("total_s", g_stats.pump_total_sec);
    prefs.putUInt("min_s",   g_stats.pump_min_sec);
    prefs.putUInt("max_s",   g_stats.pump_max_sec);
    prefs.putUInt("last_s",  g_stats.pump_last_sec);
    prefs.putUInt("fills",   g_stats.fill_count);
    prefs.putUInt("last_ts", g_stats.last_pump_ts);
    prefs.end();
}

static void onPumpStart() {
    g_pump_start_ms = millis();
    time_t now_epoch = time(nullptr);
    if (now_epoch > 1000000000UL) {
        g_stats.last_pump_ts = (uint32_t)now_epoch;
    } else if (g_rtc_ok) {
        struct tm rtc_lt;
        if (pcf8563_read(&rtc_lt)) {
            time_t t = mktime(&rtc_lt);
            if (t > 1000000000UL) g_stats.last_pump_ts = (uint32_t)t;
        }
    }
    g_stats.pump_cycles++;
    saveStats();
}

static void onPumpStop() {
    if (g_pump_start_ms == 0) return;
    uint32_t dur_sec = (millis() - g_pump_start_ms) / 1000;
    g_pump_start_ms = 0;
    if (dur_sec < 1) return; // ignore sub-second blips
    g_stats.pump_last_sec  = dur_sec;
    g_stats.pump_total_sec += dur_sec;
    if (g_stats.pump_min_sec == 0 || dur_sec < g_stats.pump_min_sec)
        g_stats.pump_min_sec = dur_sec;
    if (dur_sec > g_stats.pump_max_sec)
        g_stats.pump_max_sec = dur_sec;
    saveStats();
}

// ── Colour palette (RGB888) ───────────────────────────────────────────────────
static const uint32_t C_BG        = 0x143520;
static const uint32_t C_PANEL     = 0x0A2010;
static const uint32_t C_TEXT      = 0xD0D8E0;
static const uint32_t C_DIM       = 0x6B7280;
static const uint32_t C_ACCENT    = 0x00B4D8;
static const uint32_t C_GREEN     = 0x22C55E;
static const uint32_t C_RED       = 0xEF4444;
static const uint32_t C_AMBER     = 0xF59E0B;
static const uint32_t C_METAL     = 0x374151;
static const uint32_t C_METAL_HI  = 0x6B7280;
static const uint32_t C_METAL_SHI = 0x9CA3AF;

// ── Water colour ramp ─────────────────────────────────────────────────────────
static uint32_t waterColor(int pct) {
    if (pct >= 60) return 0x0077B6;
    if (pct >= 30) {
        float t = (60 - pct) / 30.0f;
        uint8_t r = (uint8_t)(0   + t * 245);
        uint8_t g = (uint8_t)(119 + t * 40);
        uint8_t b = (uint8_t)(182 - t * 182);
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
    if (pct >= 10) {
        float t  = (30 - pct) / 20.0f;
        uint8_t r = 245;
        uint8_t gv = (uint8_t)(159 - t * 159);
        return ((uint32_t)r << 16) | ((uint32_t)gv << 8);
    }
    return 0xDC2626;
}

static uint32_t lighten(uint32_t c, int amt) {
    uint8_t r = (uint8_t)min(255, (int)(c >> 16 & 0xFF) + amt);
    uint8_t g = (uint8_t)min(255, (int)(c >>  8 & 0xFF) + amt);
    uint8_t b = (uint8_t)min(255, (int)(c       & 0xFF) + amt);
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// ── Arc sweep gauge ───────────────────────────────────────────────────────────
// LGFX fillArc: 0° = right (3 o'clock), clockwise.
// Gap at bottom: arc runs 135° (lower-left) clockwise to 45° (lower-right).
static void drawArcGauge(int cx, int cy, int r, int pct, bool active,
                          const char* label, const char* sublabel) {
    const int RO = r;
    const int RI = r - 28;

    canvas.fillArc(cx, cy, RI - 3, RO + 3, 135, 45, 0x000000);
    canvas.fillArc(cx, cy, RI, RO, 135, 45, C_METAL);

    if (active && pct > 0) {
        int end_angle = (135 + 270 * pct / 100) % 360;
        uint32_t wc   = waterColor(pct);
        canvas.fillArc(cx, cy, RI, RO, 135, end_angle, wc);
        float ea_rad = end_angle * DEG_TO_RAD;
        canvas.drawLine(cx + (int)((RI - 4) * cosf(ea_rad)),
                        cy + (int)((RI - 4) * sinf(ea_rad)),
                        cx + (int)((RO + 4) * cosf(ea_rad)),
                        cy + (int)((RO + 4) * sinf(ea_rad)),
                        lighten(wc, 80));
    }

    for (int i = 0; i <= 4; i++) {
        float a     = (135.0f + 270.0f * i / 4.0f) * DEG_TO_RAD;
        bool  major = (i == 0 || i == 4);
        int   ri    = major ? RI - 8 : RI - 4;
        int   ro    = major ? RO + 8 : RO + 4;
        canvas.drawLine(cx + (int)(ri * cosf(a)), cy + (int)(ri * sinf(a)),
                        cx + (int)(ro * cosf(a)), cy + (int)(ro * sinf(a)),
                        major ? C_TEXT : C_DIM);
    }

    canvas.setTextDatum(lgfx::middle_center);
    if (active) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", pct);
        canvas.setTextColor(waterColor(pct));
        canvas.setFont(&fonts::Font7);
        canvas.setTextSize(1);
        canvas.drawString(buf, cx, cy - 12);
        canvas.setFont(&fonts::Font2);
        canvas.setTextColor(C_DIM);
        canvas.setTextSize(1);
        canvas.drawString("FULL", cx, cy + 28);
    } else {
        canvas.setTextColor(C_DIM);
        canvas.setFont(&fonts::Font4);
        canvas.setTextSize(1);
        canvas.drawString("NO SENSOR", cx, cy - 8);
        canvas.setFont(&fonts::Font2);
        canvas.setTextColor(C_METAL_HI);
        canvas.setTextSize(1);
        canvas.drawString("not installed", cx, cy + 18);
    }

    int label_y = cy + (int)(r * 0.72f) + 14;
    canvas.setTextDatum(lgfx::top_center);
    canvas.setTextColor(C_TEXT);
    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1.2);
    canvas.drawString(label, cx, label_y);

    if (sublabel && sublabel[0]) {
        canvas.setFont(&fonts::Font2);
        canvas.setTextSize(1);
        canvas.setTextColor(C_DIM);
        canvas.drawString(sublabel, cx, label_y + 22);
    }
}

// ── REST client (called only from Core-0 poll task) ───────────────────────────
static void pollDrum() {
    if (WiFi.status() != WL_CONNECTED) {
        if (g_reachable) g_dirty = true;
        g_reachable = false;
        return;
    }
    HTTPClient http;
    http.begin("http://" DRUM_IP "/status");
    http.setTimeout(1000);
    int code = http.GET();
    Serial0.printf("poll -> HTTP %d\n", code);
    if (code == HTTP_CODE_OK) {
        StaticJsonDocument<128> doc;
        if (!deserializeJson(doc, http.getString())) {
            int  new_level  = doc["level_pct"] | 0;
            bool new_pump   = doc["pump_on"]   | false;
            int  new_dist   = doc["dist_mm"]   | -1;
            bool new_auto   = doc["auto_mode"] | true;
            bool new_sensor = doc["sensor_ok"] | false;
            if (abs(new_level - g_level_pct) >= 2 || new_pump != g_pump_on ||
                new_auto != g_auto_mode  || new_sensor != g_sensor_ok ||
                !g_reachable) {
                g_dirty = true;
            }
            g_level_pct = new_level;
            g_pump_on   = new_pump;
            g_dist_mm   = new_dist;
            g_auto_mode = new_auto;
            g_sensor_ok = new_sensor;
            g_reachable = true;
        }
    } else {
        if (g_reachable) g_dirty = true;
        g_reachable = false;
    }
    http.end();
}

static void sendCmd(const char* path) {
    if (WiFi.status() != WL_CONNECTED) return;
    HTTPClient http;
    String url = String("http://" DRUM_IP) + path;
    http.begin(url);
    http.setTimeout(500);
    int code = http.GET();
    Serial0.printf("CMD %s -> HTTP %d\n", path, code);
    http.end();
}

// ── Core-0 poll task ──────────────────────────────────────────────────────────
static void pollTask(void*) {
    static const char* paths[] = {
        nullptr,
        "/pump/on", "/pump/off", "/auto",
        "/timeout?m=5", "/timeout?m=10", "/timeout?m=20", "/timeout?m=30"
    };
    for (;;) {
        for (int i = 0; i < 30 && g_pending_cmd == CMD_NONE; i++)
            vTaskDelay(pdMS_TO_TICKS(100));

        PendingCmd cmd = g_pending_cmd;
        g_pending_cmd  = CMD_NONE;
        if (cmd != CMD_NONE) {
            sendCmd(paths[cmd]);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        pollDrum();
    }
}

// ── Format helpers ────────────────────────────────────────────────────────────
static void fmtDuration(char* buf, size_t n, uint32_t sec) {
    if (sec == 0) { snprintf(buf, n, "--"); return; }
    uint32_t h = sec / 3600, m = (sec % 3600) / 60, s = sec % 60;
    if (h > 0)      snprintf(buf, n, "%uh %02um", h, m);
    else if (m > 0) snprintf(buf, n, "%um %02us", m, s);
    else            snprintf(buf, n, "%us", s);
}

static void fmtTimestamp(char* buf, size_t n, uint32_t epoch) {
    if (epoch == 0) { snprintf(buf, n, "--"); return; }
    time_t t = (time_t)epoch;
    struct tm lt;
    localtime_r(&t, &lt);
    strftime(buf, n, "%b %d  %H:%M", &lt);
}

// ── Statistics clock — partial repaint, no full screen clear ──────────────────
static void updateStatsClock() {
    const int rx = 420;
    canvas.fillRoundRect(rx, 62, 360, 170, 10, C_PANEL);
    canvas.setTextDatum(lgfx::middle_center);

    // Prefer NTP system clock; fall back to RTC only if system clock not yet set.
    // LovyanGFX shares I2C_NUM_0 with Wire which can cause rtc.adjust() writes
    // not to stick, so we don't rely on the RTC for current-time display.
    time_t t_now = time(nullptr);
    if (t_now < 1000000000UL && g_rtc_ok) {
        struct tm rtc_lt;
        if (pcf8563_read(&rtc_lt))
            t_now = mktime(&rtc_lt);
    }

    if (t_now > 1000000000UL) {
        struct tm lt;
        localtime_r(&t_now, &lt);

        char timebuf[12];
        strftime(timebuf, sizeof(timebuf), "%H:%M:%S", &lt);
        canvas.setFont(&fonts::Font7);
        canvas.setTextSize(1);
        canvas.setTextColor(C_TEXT);
        canvas.drawString(timebuf, rx + 180, 127);

        char datebuf[32];
        strftime(datebuf, sizeof(datebuf), "%A  %b %d  %Y", &lt);
        canvas.setFont(&fonts::Font2);
        canvas.setTextSize(1);
        canvas.setTextColor(C_DIM);
        canvas.drawString(datebuf, rx + 180, 210);
    } else {
        canvas.setFont(&fonts::Font4);
        canvas.setTextColor(C_DIM);
        canvas.drawString("No time source", rx + 180, 147);
    }
}

// ── Statistics screen ─────────────────────────────────────────────────────────
static void drawStatsScreen() {
    canvas.fillScreen(C_BG);

    // Header
    canvas.fillRect(0, 0, 800, 50, C_PANEL);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextColor(C_ACCENT);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextSize(1);
    canvas.drawString("STATISTICS", 400, 27);

    // Back button
    canvas.fillRoundRect(626, 8, 158, 34, 8, C_METAL);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextColor(C_TEXT);
    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1);
    canvas.drawString("<  BACK", 705, 25);

    // Dividers
    canvas.drawFastHLine(0, 55, 800, C_METAL);
    canvas.drawFastVLine(400, 55, 370, C_METAL);

    // ── LEFT: pump stats ──────────────────────────────────────────────────────
    const int LX     = 16;
    const int ROW_H  = 48;
    int ly = 64;

    canvas.setFont(&fonts::Font2);
    canvas.setTextSize(1);
    canvas.setTextDatum(lgfx::top_left);
    canvas.setTextColor(C_DIM);
    canvas.drawString("PUMP ACTIVITY", LX, ly);
    ly += 20;

    char buf[40];

    // Each stat row: label left, value right, inside a panel rectangle
    auto statRow = [&](const char* label, const char* value, uint32_t valColor) {
        canvas.fillRoundRect(LX, ly, 372, ROW_H - 4, 6, C_PANEL);
        canvas.setFont(&fonts::Font2);
        canvas.setTextSize(1);
        canvas.setTextDatum(lgfx::middle_left);
        canvas.setTextColor(C_DIM);
        canvas.drawString(label, LX + 10, ly + (ROW_H - 4) / 2);
        canvas.setFont(&fonts::Font4);
        canvas.setTextSize(1);
        canvas.setTextDatum(lgfx::middle_right);
        canvas.setTextColor(valColor);
        canvas.drawString(value, LX + 362, ly + (ROW_H - 4) / 2);
        ly += ROW_H;
    };

    snprintf(buf, sizeof(buf), "%u", g_stats.pump_cycles);
    statRow("Pump cycles (all time)", buf, C_TEXT);

    fmtDuration(buf, sizeof(buf), g_stats.pump_total_sec);
    statRow("Total pump time", buf, C_ACCENT);

    fmtDuration(buf, sizeof(buf), g_stats.pump_last_sec);
    statRow("Last run duration", buf, C_TEXT);

    fmtDuration(buf, sizeof(buf), g_stats.pump_min_sec);
    statRow("Shortest run", buf, C_GREEN);

    fmtDuration(buf, sizeof(buf), g_stats.pump_max_sec);
    statRow("Longest run", buf, C_AMBER);

    ly += 6;
    canvas.setFont(&fonts::Font2);
    canvas.setTextSize(1);
    canvas.setTextColor(C_DIM);
    canvas.setTextDatum(lgfx::top_left);
    canvas.drawString("Last pump started:", LX, ly);
    fmtTimestamp(buf, sizeof(buf), g_stats.last_pump_ts);
    canvas.setTextColor(C_TEXT);
    canvas.drawString(buf, LX, ly + 17);

    ly += 40;
    snprintf(buf, sizeof(buf), "Drum fill events detected: %u", g_stats.fill_count);
    canvas.setTextColor(C_ACCENT);
    canvas.drawString(buf, LX, ly);

    // ── RIGHT TOP: clock (drawn by updateStatsClock) ──────────────────────────
    updateStatsClock();

    // ── RIGHT BOTTOM: system info ─────────────────────────────────────────────
    const int RX = 420;
    canvas.fillRoundRect(RX, 242, 360, 175, 10, C_PANEL);
    int sy = 252;

    canvas.setFont(&fonts::Font2);
    canvas.setTextSize(1);
    canvas.setTextDatum(lgfx::top_left);
    canvas.setTextColor(C_DIM);
    canvas.drawString("SYSTEM", RX + 10, sy);
    sy += 20;

    auto sysRow = [&](const char* label, const char* val, uint32_t valColor) {
        canvas.setTextDatum(lgfx::top_left);
        canvas.setTextColor(C_DIM);
        canvas.setFont(&fonts::Font2);
        canvas.drawString(label, RX + 10, sy);
        canvas.setTextDatum(lgfx::top_right);
        canvas.setTextColor(valColor);
        canvas.drawString(val, RX + 350, sy);
        sy += 23;
    };

    uint32_t up_sec = millis() / 1000;
    snprintf(buf, sizeof(buf), "%uh %02um", up_sec / 3600, (up_sec % 3600) / 60);
    sysRow("Uptime", buf, C_TEXT);

    sysRow("Drum board", g_reachable ? "Online" : "Offline",
           g_reachable ? C_GREEN : C_RED);

    if (WiFi.status() == WL_CONNECTED)
        snprintf(buf, sizeof(buf), "%s  %ddBm",
                 WiFi.localIP().toString().c_str(), WiFi.RSSI());
    else
        snprintf(buf, sizeof(buf), "Disconnected");
    sysRow("WiFi", buf, C_TEXT);

    snprintf(buf, sizeof(buf), "%u KB", ESP.getFreeHeap() / 1024);
    sysRow("Free heap", buf, C_TEXT);

    snprintf(buf, sizeof(buf), "%s", g_rtc_ok ? "OK (battery-backed)" : "Not found");
    sysRow("RTC", buf, g_rtc_ok ? C_GREEN : C_RED);

    // ── Bottom bar ────────────────────────────────────────────────────────────
    canvas.fillRect(0, 432, 800, 48, C_PANEL);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextSize(1);
    canvas.setTextColor(C_DIM);
    canvas.drawString("Statistics saved across reboots  |  Pump events tracked automatically", 400, 456);
}

// ── Main screen ───────────────────────────────────────────────────────────────
static void drawMainScreen() {
    canvas.fillScreen(C_BG);

    canvas.fillRect(0, 0, 800, 50, C_PANEL);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextColor(C_ACCENT);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextSize(1);
    canvas.drawString("RAINWATER SYSTEM", 400, 27);

    char dist_sub[20] = "";
    if (g_reachable && g_dist_mm > 0)
        snprintf(dist_sub, sizeof(dist_sub), "%d mm", g_dist_mm);
    drawArcGauge(200, 241, 150, g_level_pct, g_reachable && g_sensor_ok, "DRUM", dist_sub);
    drawArcGauge(600, 241, 150, 0, false, "TOTE", nullptr);

    canvas.fillRect(0, 432, 800, 48, C_PANEL);

    // Drum online/offline
    canvas.fillCircle(18, 456, 8, g_reachable ? C_GREEN : C_RED);
    canvas.setTextDatum(lgfx::middle_left);
    canvas.setFont(&fonts::Font2);
    canvas.setTextSize(1);
    canvas.setTextColor(g_reachable ? C_GREEN : C_RED);
    canvas.drawString(g_reachable ? "DRUM ONLINE" : "DRUM OFFLINE", 32, 456);

    // Pump status
    bool pump = g_reachable && g_pump_on;
    canvas.fillCircle(158, 456, 8, pump ? C_GREEN : C_DIM);
    if (pump) canvas.fillCircle(158, 456, 4, 0xFFFFFF);
    canvas.setTextColor(pump ? C_GREEN : C_DIM);
    canvas.drawString(pump ? "PUMP ON" : "PUMP OFF", 172, 456);

    // Auto / manual mode (shifted left to make room for STATS button)
    canvas.setTextDatum(lgfx::middle_center);
    if (g_auto_mode) {
        canvas.setFont(&fonts::Font4);
        canvas.setTextColor(C_ACCENT);
        canvas.drawString("AUTO", 360, 456);
    } else {
        canvas.setFont(&fonts::Font2);
        canvas.setTextColor(C_AMBER);
        canvas.drawString("MANUAL OVERRIDE", 360, 456);
    }

    // STATS button
    canvas.fillRoundRect(462, 440, 100, 32, 8, C_PANEL);
    canvas.drawRoundRect(462, 440, 100, 32, 8, C_ACCENT);
    canvas.setFont(&fonts::Font4);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextColor(C_ACCENT);
    canvas.drawString("STATS", 512, 456);

    // MANUAL button (amber)
    canvas.fillRoundRect(572, 440, 212, 32, 8, C_AMBER);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextColor(C_BG);
    canvas.drawString("MANUAL  >", 678, 456);
}

// ── Pump buttons partial repaint ──────────────────────────────────────────────
static void drawPumpButtons() {
    bool pump = g_reachable && g_pump_on;
    canvas.fillRoundRect(50,  82, 320, 128, 14, pump ? C_GREEN : C_PANEL);
    canvas.drawRoundRect(50,  82, 320, 128, 14, C_GREEN);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextColor(pump ? C_BG : C_GREEN);
    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1.8);
    canvas.drawString("PUMP ON", 210, 146);
    canvas.fillRoundRect(430, 82, 320, 128, 14, !pump ? C_RED : C_PANEL);
    canvas.drawRoundRect(430, 82, 320, 128, 14, C_RED);
    canvas.setTextColor(!pump ? C_BG : C_RED);
    canvas.drawString("PUMP OFF", 590, 146);
}

// ── Countdown-only repaint (no full screen clear) ─────────────────────────────
static void updateCountdown() {
    unsigned long elapsed = millis() - g_manual_entry;
    int secs = (int)((MANUAL_TIMEOUT - min(elapsed, MANUAL_TIMEOUT)) / 1000);
    if ((unsigned long)secs == g_last_countdown_sec) return;
    g_last_countdown_sec = secs;
    char tbuf[40];
    snprintf(tbuf, sizeof(tbuf), "Auto-returning in %d seconds", secs);
    canvas.fillRect(0, 457, 800, 23, C_BG);
    canvas.setTextDatum(lgfx::bottom_center);
    canvas.setTextColor(C_DIM);
    canvas.setFont(&fonts::Font2);
    canvas.setTextSize(1);
    canvas.drawString(tbuf, 400, 478);
}

// ── Manual screen ─────────────────────────────────────────────────────────────
static void drawManualScreen() {
    canvas.fillScreen(C_BG);

    canvas.fillRect(0, 0, 800, 50, C_PANEL);
    canvas.setTextDatum(lgfx::middle_left);
    canvas.setTextColor(C_AMBER);
    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1.2);
    canvas.drawString("MANUAL CONTROL", 18, 25);

    canvas.fillRoundRect(626, 8, 158, 34, 8, C_METAL);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextColor(C_TEXT);
    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1);
    canvas.drawString("<  BACK", 705, 25);

    drawPumpButtons();

    canvas.setTextDatum(lgfx::middle_left);
    canvas.setTextColor(C_DIM);
    canvas.setFont(&fonts::Font2);
    canvas.setTextSize(1);
    canvas.drawString("PUMP TIMEOUT", 50, 248);

    const int timeouts[] = { 5, 10, 20, 30 };
    for (int i = 0; i < 4; i++) {
        int bx = 50 + i * 183;
        canvas.fillRoundRect(bx, 260, 165, 52, 10, C_PANEL);
        canvas.drawRoundRect(bx, 260, 165, 52, 10, C_METAL_HI);
        canvas.setTextDatum(lgfx::middle_center);
        canvas.setTextColor(C_TEXT);
        canvas.setFont(&fonts::Font4);
        canvas.setTextSize(1);
        char buf[12];
        snprintf(buf, sizeof(buf), "%d min", timeouts[i]);
        canvas.drawString(buf, bx + 82, 286);
    }

    canvas.fillRoundRect(175, 345, 450, 65, 14, C_PANEL);
    canvas.drawRoundRect(175, 345, 450, 65, 14, C_ACCENT);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextColor(C_ACCENT);
    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1.2);
    canvas.drawString("RETURN TO AUTO MODE", 400, 378);
}

// ── Touch ─────────────────────────────────────────────────────────────────────
static bool g_was_touched = false;

static void handleTouch() {
    lgfx::touch_point_t tp;
    bool touched = lcd.getTouch(&tp, 1) > 0;
    if (!touched) { g_was_touched = false; return; }
    if (g_was_touched) return;
    g_was_touched = true;

    int x = tp.x, y = tp.y;

    if (g_screen == SCREEN_MAIN) {
        // STATS button
        if (x >= 462 && x <= 562 && y >= 440 && y <= 472) {
            g_screen = SCREEN_STATS;
            g_dirty  = true;
            return;
        }
        // MANUAL button
        if (x >= 572 && x <= 784 && y >= 440 && y <= 472) {
            g_screen       = SCREEN_MANUAL;
            g_manual_entry = millis();
            g_dirty        = true;
        }

    } else if (g_screen == SCREEN_STATS) {
        // Back button
        if (x >= 626 && y >= 8 && y <= 42) {
            g_screen = SCREEN_MAIN;
            g_dirty  = true;
        }

    } else { // SCREEN_MANUAL
        // Back button
        if (x >= 626 && y >= 8 && y <= 42) {
            g_screen = SCREEN_MAIN;
            g_dirty  = true;
            return;
        }
        // Pump ON
        if (x >= 50 && x <= 370 && y >= 82 && y <= 210) {
            g_pump_on      = true;
            g_pending_cmd  = CMD_PUMP_ON;
            g_manual_entry = millis();
            drawPumpButtons();
            return;
        }
        // Pump OFF
        if (x >= 430 && x <= 750 && y >= 82 && y <= 210) {
            g_pump_on      = false;
            g_pending_cmd  = CMD_PUMP_OFF;
            g_manual_entry = millis();
            drawPumpButtons();
            return;
        }
        // Return to Auto
        if (x >= 175 && x <= 625 && y >= 345 && y <= 410) {
            g_auto_mode   = true; // optimistic — poll will confirm
            g_pending_cmd = CMD_AUTO;
            g_screen      = SCREEN_MAIN;
            g_dirty       = true;
            return;
        }
        // Timeout buttons
        const PendingCmd tcmds[] = {
            CMD_TIMEOUT_5, CMD_TIMEOUT_10, CMD_TIMEOUT_20, CMD_TIMEOUT_30
        };
        for (int i = 0; i < 4; i++) {
            int bx = 50 + i * 183;
            if (x >= bx && x <= bx + 165 && y >= 260 && y <= 312) {
                g_pending_cmd  = tcmds[i];
                g_manual_entry = millis();
                return;
            }
        }
        // Tap anywhere else: just reset the auto-return timer
        g_manual_entry = millis();
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial0.begin(115200);
    delay(2000);

    Wire.begin(15, 16);
    delay(200);

    // I2C bus scan — printed to serial to help diagnose which devices respond
    Serial0.print("I2C scan:");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
            Serial0.printf(" 0x%02X", addr);
    }
    Serial0.println();

    // Probe PCF8563 via Wire before lcd.init() — Wire still works at this point.
    Wire.beginTransmission(0x51);
    g_rtc_ok = (Wire.endTransmission() == 0);
    Serial0.printf("RTC: %s\n", g_rtc_ok ? "found" : "not found");

    // Backlight controller (0x30) init sequence
    Wire.beginTransmission(0x30); Wire.endTransmission();
    delay(20);
    Wire.beginTransmission(0x30); Wire.write(0x10); Wire.endTransmission();
    delay(50);
    Wire.beginTransmission(0x30); Wire.write(0x09); Wire.endTransmission();

    lcd.init();
    lcd.setRotation(0);

    // Load persisted stats from NVS
    loadStats();

    // Splash
    canvas.fillScreen(C_BG);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextColor(C_ACCENT);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextSize(1);
    canvas.drawString("RAINWATER SYSTEM", 400, 220);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(C_DIM);
    canvas.drawString("Connecting...", 400, 270);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);

    canvas.fillRect(0, 255, 800, 30, C_BG);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextSize(1);

    if (WiFi.status() == WL_CONNECTED) {
        // configTzTime sets TZ + starts SNTP in one call.
        // Using configTime(0,0,...) instead would internally call setenv("TZ","UTC0DST")
        // and override the Pacific setting — so we must use configTzTime here.
        configTzTime("PST8PDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");

        // Wait up to 5 s for a valid NTP time
        time_t now_utc = 0;
        unsigned long nt = millis();
        while (now_utc < 1000000000UL && millis() - nt < 5000) {
            delay(200);
            time(&now_utc);
        }

        // Sync RTC via lgfx::i2c (polling — immune to LCD DMA interrupt starvation).
        if (g_rtc_ok && now_utc > 1000000000UL) {
            struct tm lt;
            localtime_r(&now_utc, &lt); // store local Pacific time in the RTC
            if (pcf8563_write(&lt)) {
                struct tm verify;
                if (pcf8563_read(&verify)) {
                    Serial0.printf("RTC synced: %04d-%02d-%02d %02d:%02d:%02d\n",
                                   verify.tm_year + 1900, verify.tm_mon + 1,
                                   verify.tm_mday, verify.tm_hour,
                                   verify.tm_min, verify.tm_sec);
                } else {
                    Serial0.println("RTC write OK but readback returned VL flag");
                }
            } else {
                Serial0.println("RTC write failed");
            }
        }

        char ip_buf[32];
        snprintf(ip_buf, sizeof(ip_buf), "Connected  %s", WiFi.localIP().toString().c_str());
        canvas.setTextColor(C_GREEN);
        canvas.drawString(ip_buf, 400, 270);
        Serial0.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        canvas.setTextColor(C_RED);
        canvas.drawString("WiFi failed — running offline", 400, 270);
        // No configTzTime called, so set TZ manually for localtime_r
        setenv("TZ", "PST8PDT,M3.2.0,M11.1.0", 1);
        tzset();
    }

    pollDrum();
    g_prev_pump_on   = g_pump_on;
    g_prev_level_pct = g_level_pct;

    xTaskCreatePinnedToCore(pollTask, "poll", 4096, NULL, 1, NULL, 0);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    static unsigned long last_reconnect = 0;
    if (WiFi.status() != WL_CONNECTED && millis() - last_reconnect > 30000) {
        last_reconnect = millis();
        WiFi.reconnect();
    }

    // Pump state change detection (Core 1 — no cross-core race)
    bool cur_pump = g_pump_on;
    if (cur_pump && !g_prev_pump_on)       onPumpStart();
    else if (!cur_pump && g_prev_pump_on)  onPumpStop();
    g_prev_pump_on = cur_pump;

    // Fill event: level rose ≥5% since last check (drum refilled)
    int cur_level = g_level_pct;
    if (g_reachable && (cur_level - g_prev_level_pct) >= 5) {
        g_stats.fill_count++;
        saveStats();
    }
    g_prev_level_pct = cur_level;

    // Manual screen timeout
    if (g_screen == SCREEN_MANUAL && millis() - g_manual_entry >= MANUAL_TIMEOUT) {
        g_screen = SCREEN_MAIN;
        g_dirty  = true;
    }

    handleTouch();

    if (g_screen == SCREEN_MAIN) {
        if (g_dirty) {
            drawMainScreen();
            g_dirty = false;
        }

    } else if (g_screen == SCREEN_STATS) {
        if (g_dirty) {
            drawStatsScreen();
            g_last_stats_draw = millis();
            g_dirty = false;
        } else if (millis() - g_last_stats_draw >= 1000) {
            updateStatsClock(); // partial repaint — only the clock panel
            g_last_stats_draw = millis();
        }

    } else { // SCREEN_MANUAL
        if (g_dirty) {
            g_last_countdown_sec = 0xFFFFFFFF;
            drawManualScreen();
            updateCountdown();
            g_last_manual_draw = millis();
            g_dirty = false;
        } else if (millis() - g_last_manual_draw >= 500) {
            updateCountdown();
            g_last_manual_draw = millis();
        }
    }

    delay(10);
}
