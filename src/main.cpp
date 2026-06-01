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
// SERVO
// ─────────────────────────────────────────────────────────────

constexpr uint8_t SERVO_PIN = PA8;
Servo gripperServo;

// Tune these for your gripper
constexpr int OPEN_ANGLE = 180;
constexpr int CLOSE_ANGLE = 0;

// If your servo gets hot, reduce the range:
// constexpr int OPEN_ANGLE = 160;
// constexpr int CLOSE_ANGLE = 20;

static bool servo_attached = false;

// ─────────────────────────────────────────────────────────────
// UART
// ─────────────────────────────────────────────────────────────

constexpr uint32_t BAUD_RATE = 115200;

// Motor packet: [0xAA] [0x55] [L_lo] [L_hi] [R_lo] [R_hi]
constexpr uint8_t MOTOR_HEADER1 = 0xAA;
constexpr uint8_t MOTOR_HEADER2 = 0x55;
constexpr uint8_t MOTOR_MSG_SIZE = 6;

// Gripper packet: [0xAB] [0xCD] [cmd]
constexpr uint8_t GRIPPER_HEADER1 = 0xAB;
constexpr uint8_t GRIPPER_HEADER2 = 0xCD;
constexpr uint8_t GRIPPER_OPEN = 0x00;
constexpr uint8_t GRIPPER_CLOSE = 0x01;

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

enum ParserMode : uint8_t
{
    WAIT_HEADER = 0,
    MOTOR_SECOND_HEADER = 1,
    MOTOR_PAYLOAD = 2,
    GRIPPER_SECOND_HEADER = 3,
    GRIPPER_CMD = 4
};

static ParserMode parser_mode = WAIT_HEADER;

// ─────────────────────────────────────────────────────────────
// MOTOR DRIVER
// ─────────────────────────────────────────────────────────────

// pwm: -100 (full reverse) … 0 (stop) … +100 (full forward)
static void setMotor(uint8_t pinA, uint8_t pinB, uint8_t pwmPin, int16_t pwm)
{
    if (pwm > 100)
        pwm = 100;
    if (pwm < -100)
        pwm = -100;

    uint8_t duty = (uint8_t)((uint32_t)abs(pwm) * 255 / 100);

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

static inline void stopAll()
{
    setMotor(IN1_PIN, IN2_PIN, PWM_LEFT_PIN, 0);
    setMotor(IN3_PIN, IN4_PIN, PWM_RIGHT_PIN, 0);
    motors_active = false;
    digitalWrite(LED_BUILTIN, LOW);
}

// ─────────────────────────────────────────────────────────────
// GRIPPER CONTROL
// ─────────────────────────────────────────────────────────────

static void attachServoIfNeeded()
{
    if (!servo_attached)
    {
        gripperServo.attach(SERVO_PIN);
        servo_attached = true;
    }
}

static void openGripper()
{
    attachServoIfNeeded();
    gripperServo.write(OPEN_ANGLE);
    delay(300);            // give the servo time to move
    gripperServo.detach(); // release torque when open
    servo_attached = false;
}

static void closeGripper()
{
    attachServoIfNeeded();
    gripperServo.write(CLOSE_ANGLE);
    delay(300); // let it reach the close position
    // keep attached to hold gripping force
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────

void setup()
{
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    pinMode(PWM_LEFT_PIN, OUTPUT);
    pinMode(PWM_RIGHT_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    stopAll();

    Serial1.begin(BAUD_RATE);

    // Start with gripper open and detached
    openGripper();

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

        switch (parser_mode)
        {
        case WAIT_HEADER:
            if (c == MOTOR_HEADER1)
            {
                rx_buf[0] = c;
                rx_idx = 1;
                parser_mode = MOTOR_SECOND_HEADER;
            }
            else if (c == GRIPPER_HEADER1)
            {
                parser_mode = GRIPPER_SECOND_HEADER;
            }
            break;

        case MOTOR_SECOND_HEADER:
            if (c == MOTOR_HEADER2)
            {
                rx_buf[1] = c;
                rx_idx = 2;
                parser_mode = MOTOR_PAYLOAD;
            }
            else
            {
                parser_mode = WAIT_HEADER;
                rx_idx = 0;
            }
            break;

        case MOTOR_PAYLOAD:
            rx_buf[rx_idx++] = c;

            if (rx_idx == MOTOR_MSG_SIZE)
            {
                int16_t left_pwm = (int16_t)((uint16_t)rx_buf[3] << 8 | rx_buf[2]);
                int16_t right_pwm = (int16_t)((uint16_t)rx_buf[5] << 8 | rx_buf[4]);

                setMotor(IN1_PIN, IN2_PIN, PWM_LEFT_PIN, left_pwm);
                setMotor(IN3_PIN, IN4_PIN, PWM_RIGHT_PIN, right_pwm);

                last_cmd_time = millis();
                motors_active = true;
                digitalWrite(LED_BUILTIN, HIGH);

                parser_mode = WAIT_HEADER;
                rx_idx = 0;
            }
            break;

        case GRIPPER_SECOND_HEADER:
            if (c == GRIPPER_HEADER2)
            {
                parser_mode = GRIPPER_CMD;
            }
            else
            {
                parser_mode = WAIT_HEADER;
            }
            break;

        case GRIPPER_CMD:
            if (c == GRIPPER_OPEN)
            {
                openGripper();
            }
            else if (c == GRIPPER_CLOSE)
            {
                closeGripper();
            }

            parser_mode = WAIT_HEADER;
            break;
        }
    }

    if (motors_active && (millis() - last_cmd_time > WATCHDOG_MS))
    {
        stopAll();
    }
}
