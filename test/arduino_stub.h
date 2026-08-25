// Minimal Arduino/ESP32 HAL stub so the v3 control logic can be compiled and
// unit-tested on a host machine, with no board attached.
//
// Embedded code that can only be tested by flashing it is embedded code that
// mostly doesn't get tested. Everything in v3 that isn't a register write is
// pure logic, so it runs here.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>

#define HIGH 1
#define LOW  0
#define OUTPUT 1
#define INPUT  0
#define F(x) (x)
#define ESP_ARDUINO_VERSION_MAJOR 3
#define CHANGE 3
#define IRAM_ATTR
inline void attachInterrupt(int, void(*)(), int) {}
inline int  digitalPinToInterrupt(uint8_t p) { return p; }
inline void noInterrupts() {}
inline void interrupts() {}

// ── Simulated hardware state, driven by the tests ──────────────────────────
struct SimState {
    uint32_t millis = 0;
    std::vector<int> pinMode_  = std::vector<int>(64, 0);
    std::vector<int> digital_  = std::vector<int>(64, 0);
    std::vector<int> written_  = std::vector<int>(64, 0);
    std::deque<uint32_t> echoQueue;     // pulseIn() results, in order
    std::string out;                    // captured Serial output
    int servoAngle = -1;
    int pwmA = -1, pwmB = -1;
};
inline SimState& sim() { static SimState s; return s; }

inline uint32_t millis() { return sim().millis; }
inline void delayMicroseconds(uint32_t) {}
inline void pinMode(uint8_t p, int m) { sim().pinMode_[p] = m; }
inline void digitalWrite(uint8_t p, int v) { sim().written_[p] = v; }
inline int  digitalRead(uint8_t p) { return sim().digital_[p]; }
inline uint32_t pulseIn(uint8_t, int, uint32_t) {
    if (sim().echoQueue.empty()) return 0;
    uint32_t v = sim().echoQueue.front(); sim().echoQueue.pop_front(); return v;
}
inline void ledcAttach(uint8_t, uint32_t, uint8_t) {}
inline void ledcWrite(uint8_t pin, uint8_t duty) {
    if (pin == 27) sim().pwmA = duty; else if (pin == 5) sim().pwmB = duty;
}

// ── String: just enough of the Arduino API ─────────────────────────────────
class String {
public:
    std::string s;
    String() {}
    String(const char* c) : s(c ? c : "") {}
    String(const std::string& t) : s(t) {}
    void trim() {
        size_t a = s.find_first_not_of(" \t\r\n");
        size_t b = s.find_last_not_of(" \t\r\n");
        s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
    }
    size_t length() const { return s.size(); }
    void toUpperCase() { for (auto& c : s) c = toupper((unsigned char)c); }
    int indexOf(char c) const { auto p = s.find(c); return p == std::string::npos ? -1 : (int)p; }
    String substring(int a) const { return String(s.substr(a)); }
    String substring(int a, int b) const { return String(s.substr(a, b - a)); }
    int toInt() const { try { return std::stoi(s); } catch (...) { return 0; } }
    const char* c_str() const { return s.c_str(); }
    String& operator+=(char c) { s += c; return *this; }
    bool operator==(const char* o) const { return s == o; }
};

// ── Serial ─────────────────────────────────────────────────────────────────
struct SerialStub {
    std::deque<char> rx;
    void begin(long) {}
    int  available() { return (int)rx.size(); }
    int  read() { if (rx.empty()) return -1; char c = rx.front(); rx.pop_front(); return c; }
    void feed(const std::string& line) { for (char c : line) rx.push_back(c); rx.push_back('\n'); }
    void print(const char* s) { sim().out += s; }
    void print(int v)    { sim().out += std::to_string(v); }
    void print(uint32_t v){ sim().out += std::to_string(v); }
    void print(char c)   { sim().out += c; }
    void print(bool b)   { sim().out += (b ? "1" : "0"); }
    void print(float v, int dp) { char b[32]; snprintf(b, sizeof b, "%.*f", dp, v); sim().out += b; }
    void println(const char* s) { sim().out += s; sim().out += "\n"; }
    void println(int v)  { sim().out += std::to_string(v) + "\n"; }
    void println(uint32_t v){ sim().out += std::to_string(v) + "\n"; }
    void println(bool b) { sim().out += (b ? "1" : "0"); sim().out += "\n"; }
    void println(float v, int dp) { char b[32]; snprintf(b, sizeof b, "%.*f", dp, v); sim().out += b; sim().out += "\n"; }
    void println() { sim().out += "\n"; }
    void print(const String& v)   { sim().out += v.s; }
    void println(const String& v) { sim().out += v.s; sim().out += "\n"; }
};
inline SerialStub Serial;

// ── ESP32Servo ─────────────────────────────────────────────────────────────
class Servo {
public:
    void setPeriodHertz(int) {}
    void attach(uint8_t, int, int) {}
    void write(int a) { sim().servoAngle = a; }
};

using std::min;
using std::max;
