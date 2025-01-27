enum RobotState
{
    WAITING,
    MOVING,
    STOPPED
};

// Arduino pins used
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

int sensorPins[] = {A0, A1, A2, A3, A4};
int sensorMin[] = {261, 236, 240, 309, 310};
int sensorMax[] = {930, 930, 930, 930, 930};
float sensorValues[5] = {0};

// Motor speed variables
int baseSpeed = 115;
int maxSpeed = 135;

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
bool pidEnabled = true; // Enable PID by default

// Initial state
RobotState currentState = WAITING;

void stopMotors(int miliseconds)
{
    digitalWrite(Motor1, LOW);
    digitalWrite(Motor2, LOW);
    analogWrite(PWMmotor1, 0);
    analogWrite(PWMmotor2, 0);
    delay(miliseconds);
}

void stopForDuration(unsigned long duration) {
    pidEnabled = false; // Disable PID
    stopMotors(0); // Stop the motors immediately

    unsigned long startTime = millis(); // Record start time
    while (millis() - startTime < duration) {
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
    if (!pidEnabled) {
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
    int activeSensors = 0;
    int sensorValues[5] = {
        analogRead(pvalue0),
        analogRead(pvalue1),
        analogRead(pvalue2),
        analogRead(pvalue3),
        analogRead(pvalue4)};
    for (int i = 0; i < 5; i++)
    {
        if (sensorValues[i] < 500)
        {
            activeSensors++;
        }
    }

    atBifurcation = (sensorValues[0] < 500 && sensorValues[4] < 500 && sensorValues[2] > 600);
    for (int i = 0; i < 5; i++)
    {
        if (sensorValues[i] < 600)
        {
            lastSensor = i;
        }
    }
    switch (currentState)
    {
    case WAITING:
        while (analogRead(ldrPin) < 1000)
        {
            setLEDColor(255, 255, 0);
            stopMotors(0);
        }
        setLEDColor(0, 255, 0);
        currentState = MOVING;
        break;
    case MOVING:
        checkPosition();
        motorsPID();

        if (sensorValues[0] < 600 && sensorValues[1] < 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600)
        {
            stopForDuration(400);
            int a = 0;
            while (a != 1)
            {
                moveLeft(95,95);
                checkPosition();
                if (sensorValues[1] < 600 || sensorValues[2] < 600 || sensorValues[3] < 600)
                {
                    a = 1;
                }
            }
        }
        else if (sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] < 600 && sensorValues[4] < 600)
        {
            stopForDuration(400);
            int b = 0;
            while (b != 1)
            {
                moveRight(95,95);
                if (sensorValues[1] < 600 || sensorValues[2] < 600 || sensorValues[3] < 600)
                {
                    b = 1;
                }
            }
        }
        else if (sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && lastSensor == 0)
        {
            stopForDuration(300);
            moveLeft(85,95);
            delay(10);
        }
        else if (sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && lastSensor == 4)
        {
            stopForDuration(300);
            moveRight(85,95);
            delay(10);
        }
        else if (activeSensors >= 4)
        {
            moveForward(70,40);
            delay(400);
        }

        if (atBifurcation)
        {
            boolbox = true;
        }
        if (boolbox && activeSensors >= 4)
        {
            lapCount++;
            boolbox = false;
        }
        if (lapCount == 1 && atBifurcation && !pitStopCompleted)
        {
            stopForDuration(250);
            int o = 0;
            while (o != 1)
            {
                moveRight(80,60);
                if (sensorValues[3] < 600 || sensorValues[4] < 600)
                {
                    o = 1;
                }
            }
            for (int i = 0; i < 100; i++)
            {
                checkPosition();
                motorsPID();
                delay(10);
            }
            setLEDColor(255, 255, 0);
            stopMotors(3000);
            Serial.println("Pit stop!");
            pitStopCompleted = true;
            lapCount++;
            boolbox = false;
            setLEDColor(0, 255, 0);
        }

        if (atBifurcation && lapCount != 1)
        {
            moveForward(80, 120);
            delay(500);
        }
        else if (lapCount == 3)
        {
            currentState = STOPPED;
        }

        if (abs(error) < 300)
        {
            setLEDColor(0, 0, 255);
        }
        else
        {
            setLEDColor(0, 255, 0);
        }
        break;

    case STOPPED:
        stopMotors(1000);
        setLEDColor(255, 0, 0);
        break;
    }
}