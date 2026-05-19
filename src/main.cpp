#include <Arduino.h>

// Black Pill / Blue Pill built-in LED is usually PC13
const uint8_t LED_PIN = LED_BUILTIN;

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // Start OFF
    Serial1.begin(115200);      // UART1 @ 115200 baud
}

void loop()
{
    if (Serial1.available())
    {
        char cmd = Serial1.read();
        if (cmd == 'L')
        {
            // Blink 3 times to confirm receipt
            for (int i = 0; i < 3; i++)
            {
                digitalWrite(LED_PIN, HIGH);
                delay(150);
                digitalWrite(LED_PIN, LOW);
                delay(150);
            }
        }
    }
}
