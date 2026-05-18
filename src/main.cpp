#include <Arduino.h>

// Bridge 1 — original wheel pair
constexpr uint8_t A0_PIN = A0;
constexpr uint8_t A1_PIN = A1;

// Bridge 2 — new wheel pair
constexpr uint8_t A2_PIN = A2;
constexpr uint8_t A3_PIN = A3;

// Helper: set both pins of a bridge
inline void bridge1(uint8_t a, uint8_t b)
{
    digitalWrite(A0_PIN, a);
    digitalWrite(A1_PIN, b);
}

inline void bridge2(uint8_t a, uint8_t b)
{
    digitalWrite(A2_PIN, a);
    digitalWrite(A3_PIN, b);
}

void setup()
{
    pinMode(A0_PIN, OUTPUT);
    pinMode(A1_PIN, OUTPUT);
    pinMode(A2_PIN, OUTPUT);
    pinMode(A3_PIN, OUTPUT);

    bridge1(LOW, LOW);
    bridge2(LOW, LOW);

    Serial1.begin(115200);
}

void loop()
{
    if (Serial1.available())
    {
        char c = Serial1.read();

        switch (c)
        {
        case 'W': // Forward — both bridges forward
            bridge1(HIGH, LOW);
            bridge2(HIGH, LOW);
            break;

        case 'S': // Backward — both bridges reverse
            bridge1(LOW, HIGH);
            bridge2(LOW, HIGH);
            break;

        case 'A': // Left — bridge1 forward, bridge2 reverse
            bridge1(HIGH, LOW);
            bridge2(LOW, HIGH);
            break;

        case 'D': // Right — bridge1 reverse, bridge2 forward
            bridge1(LOW, HIGH);
            bridge2(HIGH, LOW);
            break;

        case 'X': // Stop — all off
        default:
            bridge1(LOW, LOW);
            bridge2(LOW, LOW);
            break;
        }
    }
}
