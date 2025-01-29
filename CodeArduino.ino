enum RobotState
{
    WAITING,
    MOVING,
    STOPPED
};

#include <SoftwareSerial.h>

#define RX_PIN 2
#define TX_PIN 4
#define Motor1 12
#define Motor2 13
#define PWMmotor1 3
#define PWMmotor2 11
#define pvalue0 A0
#define pvalue1 A1
#define pvalue2 A2
#define pvalue3 A3
#define pvalue4 A4
#define ldrPin A5
#define LED_R 5
#define LED_G 6
#define LED_B 7

SoftwareSerial mySerial(RX_PIN, TX_PIN);

int sensorPins[] = {A0, A1, A2, A3, A4};
int sensorMin[] = {261, 236, 240, 309, 310};
int sensorMax[] = {930, 930, 930, 930, 930};
float sensorValues[5] = {0};

// Motor speed variables
int baseSpeed = 120;
int maxSpeed = 138;

// PID control variables
float Kp = 12;
float Ki = 0;
float Kd = 1;

float integral = 0;
float derivative = 0;
float lastError = 0;
float ivalue0 = 0, ivalue1 = 0, ivalue2 = 0, ivalue3 = 0, ivalue4 = 0;
float value0 = 0, value1 = 0, value2 = 0, value3 = 0, value4 = 0;
float position = 0;
float error = 0;
float pidOutput = 0;
bool ldrbool = false;
bool pitStopCompleted = false;
bool atBifurcation = false;
bool boolbox = false;
int lastSensor = -1;
int targetPosition = 2000;
int lapCount = 0;
bool pidEnabled = true;
int LINE_THRESHOLD = 500;
unsigned long lapStartTime = 0;
unsigned long totalTime = 0;
bool only2 = false;

// Initial state
RobotState currentState = WAITING;

void sendToRaspberry(int lap, unsigned long lapTime, unsigned long totalTime)
{
    Serial.print("{\"lap\":");
    Serial.print(lap);
    Serial.print(",\"lapTime\":");
    Serial.print(lapTime / 1000.0, 2); // Envia o tempo da volta em segundos
    Serial.print(",\"totalTime\":");
    Serial.print(totalTime / 1000.0, 2); // Envia o tempo total em segundos
    Serial.println("}");
}

void stopMotors(int miliseconds)
{
    digitalWrite(Motor1, LOW);
    digitalWrite(Motor2, LOW);
    analogWrite(PWMmotor1, 0);
    analogWrite(PWMmotor2, 0);
    delay(miliseconds);
}

void stopForDuration(unsigned long duration)
{
    pidEnabled = false; // Disable PID
    stopMotors(0);      // Stop the motors immediately

    unsigned long startTime = millis(); // Record start time
    while (millis() - startTime < duration)
    {
        // Keep checking sensors (if needed)
        checkPosition();
    }

    pidEnabled = true; // Re-enable PID
}

void moveForward(int leftSpeed, int rightSpeed)
{
    digitalWrite(Motor1, LOW);
    digitalWrite(Motor2, HIGH);
    analogWrite(PWMmotor1, leftSpeed);
    analogWrite(PWMmotor2, rightSpeed);
}
void moveBackwards(int leftSpeed, int rightSpeed)
{
    digitalWrite(Motor1, HIGH);
    digitalWrite(Motor2, LOW);
    analogWrite(PWMmotor1, leftSpeed);
    analogWrite(PWMmotor2, rightSpeed);
}
void moveRight(int leftSpeed, int rightSpeed)
{
    digitalWrite(Motor1, LOW);
    digitalWrite(Motor2, LOW);
    analogWrite(PWMmotor1, leftSpeed);
    analogWrite(PWMmotor2, rightSpeed);
}

void moveLeft(int leftSpeed, int rightSpeed)
{
    digitalWrite(Motor1, HIGH);
    digitalWrite(Motor2, HIGH);
    analogWrite(PWMmotor1, leftSpeed);
    analogWrite(PWMmotor2, rightSpeed);
}

void setLEDColor(int r, int g, int b)
{
    analogWrite(LED_R, r);
    analogWrite(LED_G, g);
    analogWrite(LED_B, b);
}

void motorsPID()
{
    if (!pidEnabled)
    {
        // If PID is disabled, stop the motors
        stopMotors(0); // Stop without delay
        return;
    }
    error = targetPosition - position;
    integral += error;
    derivative = error - lastError;
    pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    int leftSpeed = constrain(baseSpeed + 8 + int(pidOutput), 20, maxSpeed + 8);
    int rightSpeed = constrain(baseSpeed - 8 - int(pidOutput), 20, maxSpeed - 10);
    moveForward(leftSpeed, rightSpeed);
}

void checkPosition()
{
    float sum = 0, weightedSum = 0;
    for (int i = 0; i < 5; i++)
    {
        int raw = analogRead(sensorPins[i]);
        sensorValues[i] = constrain((raw - sensorMin[i]) * 1000.0 / (sensorMax[i] - sensorMin[i]), 0, 1000);
        sum += sensorValues[i];
        weightedSum += sensorValues[i] * i * 1000; // Weight positions as 0, 1000, 2000, 3000, 4000
    }
    position = (sum == 0) ? 2000 : weightedSum / sum; // Default to center if no line
}

void setup()
{
    pinMode(Motor1, OUTPUT);
    pinMode(Motor2, OUTPUT);
    pinMode(PWMmotor1, OUTPUT);
    pinMode(PWMmotor2, OUTPUT);
    pinMode(pvalue0, INPUT);
    pinMode(pvalue1, INPUT);
    pinMode(pvalue2, INPUT);
    pinMode(pvalue3, INPUT);
    pinMode(pvalue4, INPUT);

    Serial.begin(9600);
    lapCount = 0;
    boolbox = false;
}

void loop()
{
    checkPosition();
    int activeSensors = 0;
    for (int i = 0; i < 5; i++)
    {
        if (sensorValues[i] < LINE_THRESHOLD)
        {
            activeSensors++;
        }
    }

    atBifurcation = (sensorValues[0] < LINE_THRESHOLD && sensorValues[4] < LINE_THRESHOLD && sensorValues[2] > LINE_THRESHOLD);
    for (int i = 0; i < 5; i++)
    {
        if (sensorValues[i] < LINE_THRESHOLD)
        {
            lastSensor = i;
        }
    }
    switch (currentState)
    {
    case WAITING:
        while (analogRead(ldrPin) < 980)
        {
            lapStartTime = millis();
            setLEDColor(0, 255, 255);
            stopMotors(0);
        }
        setLEDColor(255, 0, 0);
        currentState = MOVING;
        break;
    case MOVING:
        checkPosition();
        motorsPID();

        if (sensorValues[0] < LINE_THRESHOLD && sensorValues[1] < LINE_THRESHOLD && sensorValues[2] > LINE_THRESHOLD && sensorValues[3] > LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD)
        {
            stopForDuration(400); // Stop for 400ms
            pidEnabled = false;   // Disable PID during correction
            while (true)
            {
                moveLeft(100, 110); // Perform correction
                checkPosition();    // Update sensor readings
                only2 = sensorValues[0] > LINE_THRESHOLD && sensorValues[1] > LINE_THRESHOLD && sensorValues[2] < LINE_THRESHOLD && sensorValues[3] > LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD;
                if (only2)
                {
                    break; // Exit when condition is met
                }
            }

            pidEnabled = true; // Re-enable PID
        }
        else if (sensorValues[0] > LINE_THRESHOLD && sensorValues[1] > LINE_THRESHOLD && sensorValues[2] > LINE_THRESHOLD && sensorValues[3] < LINE_THRESHOLD && sensorValues[4] < LINE_THRESHOLD)
        {
            stopForDuration(400); // Stop for 400ms
            pidEnabled = false;   // Disable PID during correction

            while (true)
            {
                moveRight(115, 105); // Perform correction
                checkPosition();     // Update sensor readings
                only2 = sensorValues[0] > LINE_THRESHOLD && sensorValues[1] > LINE_THRESHOLD && sensorValues[2] < LINE_THRESHOLD && sensorValues[3] > LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD;
                if (only2 || sensorValues[0] > LINE_THRESHOLD && sensorValues[1] > LINE_THRESHOLD && sensorValues[2] < LINE_THRESHOLD && sensorValues[3] < LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD)
                {
                    break; // Exit when condition is met
                }
            }

            pidEnabled = true; // Re-enable PID
        }
        else if (sensorValues[0] > LINE_THRESHOLD && sensorValues[1] > LINE_THRESHOLD && sensorValues[2] > LINE_THRESHOLD && sensorValues[3] > LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD && lastSensor == 0)
        {
            unsigned long start = millis();
            while (millis() - start < 300)
            {
                stopForDuration(200);
                moveLeft(85, 95);
            }
        }
        else if (sensorValues[0] > LINE_THRESHOLD && sensorValues[1] > LINE_THRESHOLD && sensorValues[2] > LINE_THRESHOLD && sensorValues[3] > LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD && lastSensor == 4)
        {
            unsigned long start = millis();
            while (millis() - start < 300)
            {
                stopForDuration(200);
                moveRight(85, 95);
            }
        }
        else if (sensorValues[0] > LINE_THRESHOLD && sensorValues[1] > LINE_THRESHOLD && sensorValues[2] > LINE_THRESHOLD && sensorValues[3] > LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD)
        {
            unsigned long start = millis();
            while (millis() - start < 300)
            {
                moveBackwards(70, 40);
                checkPosition();
                if (error < 1000)
                {
                    break;
                }
            }
        }
        else if (activeSensors >= 4)
        {
            unsigned long start = millis();
            while (millis() - start < 800)
            {
                moveForward(70, 40);
            }
        }

        if (atBifurcation)
        {
            boolbox = true;
        }
        if (boolbox && activeSensors >= 4)
        {
            lapCount++;
            unsigned long lapTime = millis() - lapStartTime; // Calcula o tempo da volta
            totalTime += lapTime;                            // Atualiza o tempo total
            lapStartTime = millis();                         // Reinicia o tempo da próxima volta
            sendToRaspberry(lapCount, lapTime, totalTime);
            boolbox = false;
        }
        if (lapCount == 1 && atBifurcation && !pitStopCompleted)
        {
            stopForDuration(250);
            pidEnabled = false;
            while (true)
            {
                moveRight(80, 60);
                checkPosition();
                if (sensorValues[2] < LINE_THRESHOLD || sensorValues[3] < LINE_THRESHOLD)
                {
                    break;
                }
            }
            pidEnabled = true; // Re-enable PID

            // Continue moving for 1 second (non-blocking)
            unsigned long continueStart = millis();
            while (millis() - continueStart < 1000)
            {
                checkPosition();
                motorsPID();
            }

            // Pit stop pause (non-blocking)
            setLEDColor(255, 255, 255);
            stopForDuration(3000); // Stop for 3 seconds
            Serial.println("Pit stop!");

            pitStopCompleted = true;
            lapCount++;
            unsigned long lapTime = millis() - lapStartTime; // Calcula o tempo da volta
            totalTime += lapTime;                            // Atualiza o tempo total
            lapStartTime = millis();
            sendToRaspberry(lapCount, lapTime, totalTime);
            boolbox = false;
            setLEDColor(255, 0, 0);
        }

        if (atBifurcation && lapCount != 1)
        {
            unsigned long start = millis();
            while (millis() - start < 400)
            {
                moveForward(80, 120);
            }
        }
        else if (lapCount == 3)
        {
            currentState = STOPPED;
        }

        if (abs(error) < 400)
        {
            setLEDColor(255, 0, 0);
        }
        else
        {
            setLEDColor(0, 255, 0);
        }
        break;

    case STOPPED:
        stopForDuration(1000);
        setLEDColor(100, 100, 255);
        break;
    }
}