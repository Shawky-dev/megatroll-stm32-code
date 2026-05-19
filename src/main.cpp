#include <Arduino.h>

// Bridge 1 — left wheel
constexpr uint8_t A0_PIN = A0;
constexpr uint8_t A1_PIN = A1;
constexpr uint8_t PWM_LEFT_PIN = PB6; // PWM capable pin (change based on your board)

// Bridge 2 — right wheel
constexpr uint8_t A2_PIN = A2;
constexpr uint8_t A3_PIN = A3;
constexpr uint8_t PWM_RIGHT_PIN = PB7; // PWM capable pin (change based on your board)

// Protocol constants
constexpr uint8_t HEADER1 = 0xAA;
constexpr uint8_t HEADER2 = 0x55;
constexpr uint8_t MSG_SIZE = 6; // 2 header + 2*2 bytes for PWM values

// Motor state
int16_t left_pwm = 0;
int16_t right_pwm = 0;

// Helper: set motor direction and PWM
inline void setLeftMotor(int16_t pwm)
{
    if (pwm > 0)
    {
        digitalWrite(A0_PIN, HIGH);
        digitalWrite(A1_PIN, LOW);
        analogWrite(PWM_LEFT_PIN, abs(pwm) * 2.55); // Scale -100..100 to 0..255
    }
    else if (pwm < 0)
    {
        digitalWrite(A0_PIN, LOW);
        digitalWrite(A1_PIN, HIGH);
        analogWrite(PWM_LEFT_PIN, abs(pwm) * 2.55);
    }
    else
    {
        digitalWrite(A0_PIN, LOW);
        digitalWrite(A1_PIN, LOW);
        analogWrite(PWM_LEFT_PIN, 0);
    }
}

inline void setRightMotor(int16_t pwm)
{
    if (pwm > 0)
    {
        digitalWrite(A2_PIN, HIGH);
        digitalWrite(A3_PIN, LOW);
        analogWrite(PWM_RIGHT_PIN, abs(pwm) * 2.55);
    }
    else if (pwm < 0)
    {
        digitalWrite(A2_PIN, LOW);
        digitalWrite(A3_PIN, HIGH);
        analogWrite(PWM_RIGHT_PIN, abs(pwm) * 2.55);
    }
    else
    {
        digitalWrite(A2_PIN, LOW);
        digitalWrite(A3_PIN, LOW);
        analogWrite(PWM_RIGHT_PIN, 0);
    }
}

void setup()
{
    // Direction pins
    pinMode(A0_PIN, OUTPUT);
    pinMode(A1_PIN, OUTPUT);
    pinMode(A2_PIN, OUTPUT);
    pinMode(A3_PIN, OUTPUT);

    // PWM pins
    pinMode(PWM_LEFT_PIN, OUTPUT);
    pinMode(PWM_RIGHT_PIN, OUTPUT);

    // Initialize motors to stopped
    setLeftMotor(0);
    setRightMotor(0);

    Serial1.begin(115200);

    // LED indicator for connection status
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    static uint8_t buffer[MSG_SIZE];
    static uint8_t idx = 0;

    while (Serial1.available())
    {
        uint8_t c = Serial1.read();

        // Look for header
        if (idx == 0 && c != HEADER1)
        {
            continue;
        }
        if (idx == 1 && c != HEADER2)
        {
            idx = 0;
            continue;
        }

        buffer[idx++] = c;

        // Check if complete message received
        if (idx == MSG_SIZE)
        {
            // Parse PWM values (little-endian signed 16-bit)
            left_pwm = (int16_t)((buffer[3] << 8) | buffer[2]);
            right_pwm = (int16_t)((buffer[5] << 8) | buffer[4]);

            // Apply motor commands
            setLeftMotor(left_pwm);
            setRightMotor(right_pwm);

            // Turn on LED to show active connection
            digitalWrite(LED_BUILTIN, HIGH);

            idx = 0;
        }
    }

    // Watchdog - stop motors if no command for 1 second
    static unsigned long last_cmd_time = millis();
    if (idx > 0)
    {
        last_cmd_time = millis();
    }
    else if (millis() - last_cmd_time > 1000)
    {
        setLeftMotor(0);
        setRightMotor(0);
        digitalWrite(LED_BUILTIN, LOW); // Turn off LED when stopped
    }
}
