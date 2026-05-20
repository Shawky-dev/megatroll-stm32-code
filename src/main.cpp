#include <Arduino.h>
#include <Servo.h>

// ─────────────────────────────────────────────────────────────
// MOTOR PINS
// ─────────────────────────────────────────────────────────────

// Left motor
constexpr uint8_t IN1_PIN = PA0;
constexpr uint8_t IN2_PIN = PA1;
constexpr uint8_t PWM_LEFT_PIN = PB6;

// Right motor
constexpr uint8_t IN3_PIN = PA2;
constexpr uint8_t IN4_PIN = PA3;
constexpr uint8_t PWM_RIGHT_PIN = PB7;

// ─────────────────────────────────────────────────────────────
// SERVO PIN
// ─────────────────────────────────────────────────────────────

constexpr uint8_t SERVO_PIN = PA8;

// ─────────────────────────────────────────────────────────────
// UART
// ─────────────────────────────────────────────────────────────

constexpr uint32_t BAUD_RATE = 115200;

// Motor packet
constexpr uint8_t MOTOR_HEADER1 = 0xAA;
constexpr uint8_t MOTOR_HEADER2 = 0x55;
constexpr uint8_t MOTOR_MSG_SIZE = 6;

// Gripper packet
constexpr uint8_t GRIPPER_HEADER1 = 0xAB;
constexpr uint8_t GRIPPER_HEADER2 = 0xCD;
constexpr uint8_t GRIPPER_MSG_SIZE = 3;

// ─────────────────────────────────────────────────────────────
// WATCHDOG
// ─────────────────────────────────────────────────────────────

constexpr uint32_t WATCHDOG_MS = 500;

// ─────────────────────────────────────────────────────────────
// STATE
// ─────────────────────────────────────────────────────────────

static uint8_t rx_buf[6];
static uint8_t rx_idx = 0;

static uint32_t last_cmd_time = 0;
static bool motors_active = false;

// ─────────────────────────────────────────────────────────────
// SERVO
// ─────────────────────────────────────────────────────────────

Servo gripperServo;

constexpr int OPEN_ANGLE = 0;
constexpr int CLOSE_ANGLE = 90;

// ─────────────────────────────────────────────────────────────
// MOTOR DRIVER
// ─────────────────────────────────────────────────────────────

static void setMotor(uint8_t pinA,
                     uint8_t pinB,
                     uint8_t pwmPin,
                     int16_t pwm)
{
    // Clamp
    if (pwm > 100)
        pwm = 100;

    if (pwm < -100)
        pwm = -100;

    uint8_t duty =
        (uint8_t)((uint32_t)abs(pwm) * 255 / 100);

    if (pwm > 0)
    {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
    }
    else if (pwm < 0)
    {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
    }
    else
    {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        duty = 0;
    }

    analogWrite(pwmPin, duty);
}

// ─────────────────────────────────────────────────────────────
// STOP ALL
// ─────────────────────────────────────────────────────────────

static inline void stopAll()
{
    setMotor(IN1_PIN, IN2_PIN, PWM_LEFT_PIN, 0);
    setMotor(IN3_PIN, IN4_PIN, PWM_RIGHT_PIN, 0);

    motors_active = false;

    digitalWrite(LED_BUILTIN, LOW);
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────

void setup()
{
    // Motor pins
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);

    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    pinMode(PWM_LEFT_PIN, OUTPUT);
    pinMode(PWM_RIGHT_PIN, OUTPUT);

    pinMode(LED_BUILTIN, OUTPUT);

    stopAll();

    // Servo
    gripperServo.attach(SERVO_PIN);

    gripperServo.write(OPEN_ANGLE);

    // UART
    Serial1.begin(BAUD_RATE);

    last_cmd_time = millis();
}

// ─────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────

void loop()
{
    while (Serial1.available())
    {
        uint8_t c = (uint8_t)Serial1.read();

        // ─────────────────────────────────────────
        // MOTOR PACKET
        // ─────────────────────────────────────────

        if (rx_idx == 0)
        {
            if (c == MOTOR_HEADER1)
            {
                rx_buf[0] = c;
                rx_idx = 1;
            }
            else if (c == GRIPPER_HEADER1)
            {
                rx_buf[0] = c;
                rx_idx = 100; // gripper state machine
            }
        }

        // ─────────────────────────────────────────
        // MOTOR HEADER2
        // ─────────────────────────────────────────

        else if (rx_idx == 1)
        {
            if (c == MOTOR_HEADER2)
            {
                rx_buf[1] = c;
                rx_idx = 2;
            }
            else
            {
                rx_idx = 0;
            }
        }

        // ─────────────────────────────────────────
        // MOTOR PAYLOAD
        // ─────────────────────────────────────────

        else if (rx_idx >= 2 && rx_idx < MOTOR_MSG_SIZE)
        {
            rx_buf[rx_idx++] = c;

            if (rx_idx == MOTOR_MSG_SIZE)
            {
                int16_t left_pwm =
                    (int16_t)((uint16_t)rx_buf[3] << 8 |
                              rx_buf[2]);

                int16_t right_pwm =
                    (int16_t)((uint16_t)rx_buf[5] << 8 |
                              rx_buf[4]);

                setMotor(IN1_PIN,
                         IN2_PIN,
                         PWM_LEFT_PIN,
                         left_pwm);

                setMotor(IN3_PIN,
                         IN4_PIN,
                         PWM_RIGHT_PIN,
                         right_pwm);

                last_cmd_time = millis();

                motors_active = true;

                digitalWrite(LED_BUILTIN, HIGH);

                rx_idx = 0;
            }
        }

        // ─────────────────────────────────────────
        // GRIPPER HEADER2
        // ─────────────────────────────────────────

        else if (rx_idx == 100)
        {
            if (c == GRIPPER_HEADER2)
            {
                rx_idx = 101;
            }
            else
            {
                rx_idx = 0;
            }
        }

        // ─────────────────────────────────────────
        // GRIPPER STATE BYTE
        // ─────────────────────────────────────────

        else if (rx_idx == 101)
        {
            uint8_t state = c;

            if (state == 0)
            {
                gripperServo.write(OPEN_ANGLE);
            }
            else if (state == 1)
            {
                gripperServo.write(CLOSE_ANGLE);
            }

            rx_idx = 0;
        }
    }

    // ─────────────────────────────────────────────
    // WATCHDOG
    // ─────────────────────────────────────────────

    if (motors_active &&
        (millis() - last_cmd_time > WATCHDOG_MS))
    {
        stopAll();
    }
}
