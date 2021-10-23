#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <float.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include "TimerOne.h"

#define LOG(...)        print_log(__VA_ARGS__)
#define HAS_OLED        (1)

enum class RUNNING_STATE : char
{
    BOOT,
    ACTIVE,
    SHUTDOWN,
};
enum class HEAT_CTRL_MODE : char
{
    IDLE,
    UP,
    DOWN,
};
enum class BT_RESPONSE : char
{
    AOK, CMD, ERR, TIMEOUT
};
enum class STATUS_CODE : char
{
    // error
    UNKNOWN_ERROR = -64,
    INVALID_COMMAND,
    INVALID_ARGUMENT,
    COMMAND_OVERFLOW,
    TEMPERATURE_OVERLIMIT,
    TEMPERATURE_FEEDBACK_FAILED,
    BTDEVICE_ERROR,

    // ok
    STANDBY = 0, 
    COOKING,
};

template<typename T, typename... Args> void print_log_impl(T&& value, Args&& ...args)
{
    Serial.print(value);
    print_log_impl(args...);
}
void print_log_impl()
{
    Serial.println();
}
template<typename... Args> void print_log(Args&& ...args)
{
    print_log_impl(args...);
}

struct STATUS
{
    STATUS_CODE         code = STATUS_CODE::STANDBY;
    char                cmdid = 0;
    char                cmdnum = 0;
    char                power = 0;
    short               temperature = 0;
    unsigned short      remainTime = 0;

    void setCode(STATUS_CODE value) volatile
    {
        LOG(F("Error Occurred. code = "), (int)value);
        if((int)code >= 0)
            code = value;
    }
    void reset() volatile
    {  
        code = STATUS_CODE::STANDBY;
    }
    bool hasError() const volatile
    {
        return (int)code < 0;
    }
};
static_assert(sizeof(STATUS) == 8);

enum COMMAND : char
{
    CMD_NOP,
    CMD_FINISH,
    CMD_TARGET_TEMPERATURE,
    CMD_KEEP,
    CMD_SET_KP,
    CMD_SET_TI,
    CMD_SET_TD,
    CMD_SET_PHASE_DELAY,
    CMD_SET_POWER,
};

struct COMMAND_DATA
{
    unsigned char cmd;
    unsigned char index;
    unsigned char params[6];
};
static_assert(sizeof(COMMAND_DATA) == 8);
#define COMMAND_DATA_MAX    (32)

RUNNING_STATE runningState = RUNNING_STATE::BOOT;
volatile STATUS status;
volatile unsigned long zeroCrossInterval = 0;
volatile HEAT_CTRL_MODE heatControlMode = HEAT_CTRL_MODE::IDLE;
volatile char heatControlQueWPos = 0;
volatile char heatControlQueRPos = 0;
volatile float currentTemperature = 0;
volatile float targetTemperature = 0;
volatile float temperatureError = 0;
volatile float temperatureErrorIntegral = 0;
volatile float temperatureErrorDifferential = 0;
volatile float Kp = 0;
volatile float Ti = 0;
volatile float Td = 0;

volatile unsigned short currentTemperatureRaw = 0;

COMMAND_DATA commands[COMMAND_DATA_MAX] = { 0 };

// input
#define POWER_SW_PIN    (2)
#define ZERO_CROSS_PIN  (3)
#define ZERO_CROSS_DUMMY_PIN  (4)

// output
#define POWER_ON_PIN    (7)
#define BUZZER_PIN      (11)
#define HEAT_CTRL_PIN   (5)

// adc
#define THERMAL_PIN     (0)

#define OLED_ADDRESS    (0x3C)

// heater control factor
#define EEPROM_Kp_ADDR  (0)
#define EEPROM_Ti_ADDR  (4)
#define EEPROM_Td_ADDR  (8)

// service id
#define SERVICE_ID_COMMAND_RECEIVE  "001B"
#define SERVICE_ID_STATUS_NOTIFY    "001D"

// tone
#define TONE_DO4  (261)
#define TONE_RE4  (293)
#define TONE_MI4  (329)
#define TONE_FA4  (349)
#define TONE_SO4  (391)
#define TONE_RA4  (440)
#define TONE_SI4  (493)
#define TONE_DO5  (523)
#define TONE_RE5  (587)
#define TONE_MI5  (659)
#define TONE_FA5  (698)
#define TONE_SO5  (783)
#define TONE_RA5  (880)
#define TONE_SI5  (987)
#define TONE_DO6  (1046)
#define TONE_RE6  (1174)
#define TONE_MI6  (1318)
#define TONE_FA6  (1396)
#define TONE_SO6  (1567)
#define TONE_RA6  (1760)
#define TONE_SI6  (1975)

#if HAS_OLED
SSD1306AsciiAvrI2c oled;
#endif
TimerOne timer1;
SoftwareSerial serialBT(10, 9);

#define clamp(x, minv, maxv)    min(max(x, minv), maxv)

void setup() 
{
    Serial.begin(9600);
    LOG(F("Booting..."));

    LOG(F("Setup io pins..."));
    pinMode(POWER_SW_PIN, INPUT);
    pinMode(ZERO_CROSS_PIN, INPUT);
    pinMode(ZERO_CROSS_DUMMY_PIN, INPUT);
    
    pinMode(POWER_ON_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(HEAT_CTRL_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(POWER_ON_PIN, HIGH);
    digitalWrite(HEAT_CTRL_PIN, LOW);

    delay(1000);

#if HAS_OLED
    LOG(F("Setup OLED..."));
    oled.begin(&Adafruit128x32, OLED_ADDRESS);
    oled.setFont(System5x7);
    oled.displayRemap(true);
#endif

    auto inRange = [](float x, float vmin, float vmax)
    {
        return !isnanf(x) && (x >= vmin) && (x < vmax);
    };

    LOG(F("Setup EEPROM..."));
    bool forceWrite = false;

    auto loadValue = [forceWrite, &inRange](int addr, volatile float& var, float minValue, float maxValue, float defaultValue)
    {
        EEPROM.get(addr, var);
        if(forceWrite || !inRange(var, minValue, maxValue))
        {
            var = defaultValue;
            EEPROM.put(addr, var);
        }
    };

    loadValue(EEPROM_Kp_ADDR, Kp, 0.000001f, 10000.f, 0.3f);
    loadValue(EEPROM_Ti_ADDR, Ti, 0.000000f, 90000.f, 0.f);
    loadValue(EEPROM_Td_ADDR, Td, 0.000000f, 90000.f, 0.f);

    LOG(F("Kp = "), Kp);
    LOG(F("Ti = "), Ti);
    LOG(F("Td = "), Td);

    LOG(F("OK"));

    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 2000);
    delay(100);
    tone(BUZZER_PIN, 1000);
    delay(100);
    noTone(BUZZER_PIN);

    serialBT.begin(2400);
    rebootBT();

    analogReference(INTERNAL);
    attachInterrupt(1, zeroCrossInterrupt, RISING);

    timer1.initialize(100);
    timer1.attachInterrupt(timerInterrupt);
}

char advanceControlQuePos(volatile char& pos)
{
    auto old = pos;
    return old;
}

void zeroCrossInterrupt()
{
    static unsigned long lastTime = 0;

    unsigned long now = micros();
    unsigned long d = now - lastTime;
    lastTime = now;

    if(d < 5000) // ignore irregular value
        return;

    // correct by source frequency 50Hz
    zeroCrossInterval = 10000;

    const auto rate = calcPowerRateFeedbacked();
    status.power = (char)(rate*100.f);
    if(rate > 0)
    {
        noInterrupts();
        {
            if(rate >= 1.0f)
            {
                heatControlQueWPos = heatControlQueRPos = 0;
                digitalWrite(HEAT_CTRL_PIN, HIGH);
            }
            else
            {
            }
        }
        interrupts();
    }
}

void timerInterrupt()
{
    if(heatControlMode == HEAT_CTRL_MODE::IDLE){
        if(heatControlQueWPos == heatControlQueRPos)
            return;
        heatControlMode = HEAT_CTRL_MODE::UP;
    }

    unsigned long now = micros();
    {
        switch(heatControlMode)
        {
            case HEAT_CTRL_MODE::DOWN:
                digitalWrite(HEAT_CTRL_PIN, LOW);
                heatControlMode = HEAT_CTRL_MODE::IDLE;
                advanceControlQuePos(heatControlQueRPos);
                break;
            case HEAT_CTRL_MODE::UP:
                {
                    digitalWrite(HEAT_CTRL_PIN, HIGH);
                    heatControlMode = HEAT_CTRL_MODE::DOWN;
                    break;
                }
        }
    }
}

float calcPowerRateFeedbacked()
{
    const float invTi = (Ti>0) ? 1.f/Ti : 0.f;
    return currentTemperature < 30.0f ? min(rate, 0.5f) : rate;
}

unsigned long calcHeatPowerHighDelay(float powerRate)
{
    powerRate = 1.0f - clamp(powerRate, 0.f, 1.f);
    return (unsigned long)(acosf(1.f - 2.f*powerRate)*zeroCrossInterval/(float)PI);
}

template<typename T> void sort(T* a, int n)
{
    for(int i = 0; i < n - 1; ++i)
    {
        for(int j = n - 1; j > i; --j)
        {
            if(a[j] < a[j - 1])
            {
                int t = a[j - 1];
                a[j - 1] = a[j];
                a[j] = t;
            }
        }
    }
}

void measureTemperature()
{
    const int Samples = 5;
    const int Histories = 10;
    const float B = 4000.f;
    const float R0 = 58.3f;
    const float Rv = 1.5f;
    const float Vref = 4.7f;

    static short samples[Samples];
    static short histories[Histories];
    static char sampleIndex;
    static char historiesIndex;
    static unsigned long measuringInterval;

    if(measuringInterval < millis())
    {
        // get samples for median cut
        samples[sampleIndex++] = analogRead(THERMAL_PIN);
        if(sampleIndex >= Samples)
        {
            sampleIndex = 0;
            sort(samples, Samples);
            
            // moving average
            histories[historiesIndex++] = samples[Samples>>1];
            if(historiesIndex >= Histories)
                historiesIndex = 0;
            int vInt = 0;
            for(auto v : histories) 
                vInt += v;
            vInt /= Histories;

            const float r = (Rv*Vref*1024.f/1.1f - Rv*vInt)/vInt;
            currentTemperature = max((B*(T0 + 273))/(logf(r/R0)*(T0 + 273)+B) - 273, 0.f);
            
            const float e = targetTemperature - currentTemperature;
            temperatureErrorDifferential = e - temperatureError;
            temperatureError = e;
            temperatureErrorIntegral += e;
            const float integralLimit = 10e+10f;
            if(temperatureErrorIntegral > integralLimit)
                temperatureErrorIntegral = integralLimit;
            else if(temperatureErrorIntegral < -integralLimit)
                temperatureErrorIntegral = -integralLimit;

            currentTemperatureRaw = vInt;

            //LOG("Vtemp = ", vInt);
            //LOG("T = ", (int)(currentTemperature*10));

            status.temperature = (short)(currentTemperature*256);
        }

        measuringInterval = millis() + 1000/Samples;
    }
}

BT_RESPONSE waitBTResponse(unsigned long timeout)
{
    char response[10] = {0};
    char index = 0;
    auto startTime = millis();
    do
    {
        while(serialBT.available())
        {
            int c = serialBT.read();
            if(c == '\n')
            {
                response[index] = '\0';
                index = 0;

                if(strncmp(response, "AOK", 3) == 0)
                    return BT_RESPONSE::AOK;
                else if(strncmp(response, "ERR", 3) == 0)
                    return BT_RESPONSE::ERR;
                else if(strncmp(response, "CMD", 3) == 0)
                    return BT_RESPONSE::CMD;
            }
            else if (isprint(c))
            {
                response[index++] = c;
                if(index >= 8)
                    index = 0;
            }
        }
    }while((unsigned long)(millis() - startTime) < timeout);

    LOG(F("BT timeout "), (int)(millis() - startTime), " < ", timeout);

    return BT_RESPONSE::TIMEOUT;
}

void parseBT()
{
    static char line[64];
    static int index = 0;

    while(serialBT.available())
    {
        int c = serialBT.read();
        Serial.write(c);
        if(c == '\n')
        {
            line[index] = '\0';
            index = 0;

            // WV,001B,0011223344556677
            if(strncmp(line, "WV,", 3) == 0)
            {
                if(strncmp(line+3, SERVICE_ID_COMMAND_RECEIVE",", 5) == 0)
                {
                    auto p = line+8;
                    auto q = strchr(p, '.');
                    if(q == nullptr || (q - p) != 16)
                    {
                        LOG(line);
                        status.setCode(STATUS_CODE::INVALID_ARGUMENT);
                    }
                    else
                    {
                        COMMAND_DATA command;
                        auto data = reinterpret_cast<unsigned char*>(&command);
                        for(int i = 0; i < 8; ++i)
                        {
                            char hexstr[3] = { p[i*2 + 0], p[i*2 + 1], '\0' };
                            data[i] = (char)strtol(hexstr, nullptr, 16);
                        }

                        int index = 0;
                        if(command.index < 0x80)
                            index = command.index;
                        else if(command.index == 0x80)
                            index = status.cmdnum++;
                        else if(command.index == 0x81)
                            index = status.cmdid;

                        if(index >= COMMAND_DATA_MAX)
                            status.setCode(STATUS_CODE::COMMAND_OVERFLOW);
                        else
                            commands[index] = command;
                        LOG("Set command ", (int)command.cmd, ", index=", (int)index);
                    }
                }
            }
        }
        else if(isprint(c))
        {
            if(index < 64)
                line[index++] = c;
        }
    }
}

void rebootBT()
{
    LOG(F("Reboot BT"));
    for(int i = 0; i < 3; ++i)
    {
        serialBT.write("R,1\n");
        delay(1000);
        if(waitBTResponse(1000) == BT_RESPONSE::CMD)
        {
            LOG(F("OK."));
            return;
        }
        delay(1000);
    }
    status.setCode(STATUS_CODE::BTDEVICE_ERROR);
}

void playFinishBeep()
{    
    static const int T= 50;
    auto wait = [](int n){ delay(n*T*2); };

    tone(BUZZER_PIN, TONE_DO5, T); wait(2);
    tone(BUZZER_PIN, TONE_DO5, T); wait(1);
    tone(BUZZER_PIN, TONE_DO5, T); wait(1);
    tone(BUZZER_PIN, TONE_MI5); wait(2);
    tone(BUZZER_PIN, TONE_SO5); wait(2);

    tone(BUZZER_PIN, TONE_DO5, T); wait(2);
    tone(BUZZER_PIN, TONE_DO5, T); wait(1);
    tone(BUZZER_PIN, TONE_DO5, T); wait(1);
    tone(BUZZER_PIN, TONE_MI5); wait(2);
    tone(BUZZER_PIN, TONE_SO5); wait(2);

    tone(BUZZER_PIN, TONE_DO5); wait(3);
    tone(BUZZER_PIN, TONE_MI5); wait(1);
    tone(BUZZER_PIN, TONE_DO5); wait(2);
    tone(BUZZER_PIN, TONE_MI5); wait(2);
    tone(BUZZER_PIN, TONE_SO4, T); wait(2);
    tone(BUZZER_PIN, TONE_SO4, T); wait(2);
    tone(BUZZER_PIN, TONE_SO4); wait(4);

    tone(BUZZER_PIN, TONE_DO5, T); wait(2);
    tone(BUZZER_PIN, TONE_DO5, T); wait(1);
    tone(BUZZER_PIN, TONE_DO5, T); wait(1);
    tone(BUZZER_PIN, TONE_MI5); wait(2);
    tone(BUZZER_PIN, TONE_SO5); wait(2);

    tone(BUZZER_PIN, TONE_DO5, T); wait(2);
    tone(BUZZER_PIN, TONE_DO5, T); wait(1);
    tone(BUZZER_PIN, TONE_DO5, T); wait(1);
    tone(BUZZER_PIN, TONE_MI5); wait(2);
    tone(BUZZER_PIN, TONE_SO5); wait(2);

    tone(BUZZER_PIN, TONE_DO5); wait(3);
    tone(BUZZER_PIN, TONE_MI5); wait(1);
    tone(BUZZER_PIN, TONE_DO5); wait(2);
    tone(BUZZER_PIN, TONE_MI5); wait(2);
    tone(BUZZER_PIN, TONE_DO5); wait(4);
    
    noTone(BUZZER_PIN);
}

void playNotficationBeep()
{
    static const int T= 100;
    auto wait = [](int n){ delay(n*T*2); };

    tone(BUZZER_PIN, TONE_SO5); wait(1);
    tone(BUZZER_PIN, TONE_FA5); wait(1);
    tone(BUZZER_PIN, TONE_MI5); wait(2);
    tone(BUZZER_PIN, TONE_SO5); wait(1);
    tone(BUZZER_PIN, TONE_DO6); wait(1);
    tone(BUZZER_PIN, TONE_SI5); wait(2);

    tone(BUZZER_PIN, TONE_SO5); wait(1);
    tone(BUZZER_PIN, TONE_RE6); wait(1);
    tone(BUZZER_PIN, TONE_DO6); wait(2);
    tone(BUZZER_PIN, TONE_MI6); wait(4);

    tone(BUZZER_PIN, TONE_DO6); wait(1);
    tone(BUZZER_PIN, TONE_SI5); wait(1);
    tone(BUZZER_PIN, TONE_RA5); wait(2);
    tone(BUZZER_PIN, TONE_FA6); wait(1);
    tone(BUZZER_PIN, TONE_RE6); wait(1);
    tone(BUZZER_PIN, TONE_DO6); wait(2);
    tone(BUZZER_PIN, TONE_SI5); wait(2);
    tone(BUZZER_PIN, TONE_DO6); wait(1);

    tone(BUZZER_PIN, TONE_SO6, T*3/2); wait(1);
    tone(BUZZER_PIN, TONE_SO6); wait(1);
    tone(BUZZER_PIN, TONE_FA6); wait(1);
    tone(BUZZER_PIN, TONE_MI6, T*3/2); wait(4);

    noTone(BUZZER_PIN);
}

void reset()
{
    status.cmdid = 0;
    status.cmdnum = 0;
    commands[0].cmd = COMMAND::CMD_NOP;
    targetTemperature = 0;
    temperatureError = 0;
    temperatureErrorIntegral = 0;
    temperatureErrorDifferential = 0;
}

void processCommand()
{
    static const float  us_to_s = 0.000001f;

    static char previousCommand = CMD_NOP;
    static float operationTime = 0;
    static unsigned long previousTime = micros();
    auto& data = commands[status.cmdid];

    const auto changed = previousCommand != data.cmd;
    const auto now = micros();
    const auto delta = now - previousTime;

    if(changed)
        LOG(F("Active command changed "), (int)previousCommand, F("-> "), (int)data.cmd);

    switch(data.cmd)
    {
        case CMD_NOP:
            break;
        case CMD_FINISH:
            reset();
            playFinishBeep();
            break;
        case CMD_TARGET_TEMPERATURE:
            {
                const auto temp = (float)*reinterpret_cast<unsigned char*>(data.params);
                if(changed || targetTemperature != temp)
                {
                    operationTime = 0;
                    targetTemperature = temp;
                    LOG(F("Target temp "), (int)(temp),  F("."), (int)((temp - (int)temp)*100));
                }
                if(fabs(currentTemperature - targetTemperature) <= 0.5f)
                    operationTime += delta*us_to_s;
                else 
                    operationTime = 0;
                if(operationTime > 120.f)
                    ++status.cmdid;
            }
            break;
        case CMD_KEEP:
            {
                const auto waitSeconds = *reinterpret_cast<unsigned short*>(data.params)*60.f;
                if(changed)
                {
                    operationTime = 0;
                    LOG(F("Wait sec "), (int)waitSeconds);
                }
                operationTime += delta*us_to_s;
                const auto remain = max(waitSeconds - operationTime, 0.0f);
                if(remain < 3600)
                    status.remainTime = (unsigned short)ceilf(remain);
                else
                    status.remainTime = (unsigned short)ceilf(remain / 60.f) | 0x8000;
                if(operationTime >= waitSeconds)
                {
                    playNotficationBeep();
                    ++status.cmdid;
                }
            }
            break;
        case CMD_SET_KP:
            Kp = *reinterpret_cast<float*>(data.params);
            EEPROM.put(EEPROM_Kp_ADDR, Kp);
            LOG(F("Set Kp "), Kp);
            ++status.cmdid;
            break;
        case CMD_SET_TI:
            Ti = *reinterpret_cast<float*>(data.params);
            EEPROM.put(EEPROM_Ti_ADDR, Ti);
            LOG(F("Set Ti "), Ti);
            ++status.cmdid;
            break;
        case CMD_SET_TD:
            Td = *reinterpret_cast<float*>(data.params);
            EEPROM.put(EEPROM_Td_ADDR, Td);
            LOG(F("Set Td "), Td);
            ++status.cmdid;
            break;
        case CMD_SET_PHASE_DELAY:
            phaseDelayUs = *reinterpret_cast<unsigned short*>(data.params);
            LOG(F("Set Phase Delay "), phaseDelayUs);
            break;
        case CMD_SET_POWER:
            LOG(F("Set Power "), powerUserSetting);
            break;
    }

    previousTime = now;
    previousCommand = data.cmd;
}

template<typename Fn> void oledPrint(int cols, Fn&& fn)
{
#if HAS_OLED
    auto width = oled.fontWidth() + 1;
    auto curcol = oled.col();
    fn();
    auto left = (curcol + cols*width) - oled.col();
    if(left > 0)
        oled.clearField(oled.col(), oled.row(), (left + width - 1)/width);
    oled.setCol(curcol + width*cols);
#endif
}

void display()
{
    static unsigned long interval;
    static unsigned long count;

    auto now = millis();
    if((unsigned long)(now - interval) >= 5000)
    {
        interval = now;

#if HAS_OLED
        oled.home();
        oledPrint(20, [](){
            oled.print("ST:");
            oled.print((int)status.code);
            oled.print("/");
            oled.print((int)status.cmdid);
            oled.print("/");
            oled.print((int)status.cmdnum);
            oled.print("/");
            oled.print((int)status.power);
            oled.print("/");
            oled.print(status.temperature/256.f);
        });    

        oled.setCursor(0, 1);
        oledPrint(20, [](){
            oled.print("PID:");
            oled.print(Kp);
            oled.print('/');
            oled.print(Ti);
            oled.print('/');
            oled.print(Td);
        });  

        oled.setCursor(0, 2);
        oledPrint(10, [](){
            oled.print("CTL:");
            oled.print((int)currentTemperatureRaw);
        });  
#endif 

        //
        char strbuf[128];
        auto stbytes = reinterpret_cast<volatile unsigned char*>(&status);
        sprintf(strbuf, "SHW,%s,%02x%02x%02x%02x%02x%02x%02x%02x\n", SERVICE_ID_STATUS_NOTIFY, 
            (int)stbytes[0], (int)stbytes[1], (int)stbytes[2], (int)stbytes[3], (int)stbytes[4], (int)stbytes[5], (int)stbytes[6], (int)stbytes[7]);
        
#if 0
        serialBT.write(strbuf);
#else
        auto sendWithRetry = [&]()
        {
            for(int i = 0; i < 3; ++i)
            {
                serialBT.write(strbuf);
                if(waitBTResponse(1000) == BT_RESPONSE::AOK)
                    return true;
                delay(100);
            }
            return false;
        };
        if(!sendWithRetry())
            rebootBT();
#endif

        //LOG(strbuf);
    }
}

void run()
{
    if(status.hasError())
    {
        currentTemperature = 0;
        targetTemperature = 0;
        temperatureError = 0;
        temperatureErrorIntegral = 0;
        temperatureErrorDifferential = 0;
    }
    else
    {
        measureTemperature();
        parseBT();
        processCommand();
    }

    display();

    delay(1);
}

void loop()
{
    if(runningState == RUNNING_STATE::SHUTDOWN)
        return;

    const int poweron = digitalRead(POWER_SW_PIN);

    switch(runningState)
    {
        case RUNNING_STATE::BOOT:
            if(poweron == LOW)
            {
                runningState = RUNNING_STATE::ACTIVE;
                LOG(F("Stabled"));
                interrupts();
            }
            break;

        case RUNNING_STATE::ACTIVE:
            if(poweron == HIGH)
            {
                runningState = RUNNING_STATE::SHUTDOWN;
                LOG(F("Shutdown!!"));
                digitalWrite(POWER_ON_PIN, LOW);
                tone(BUZZER_PIN, 400, 500);
                noInterrupts();
            }
            else
            {
                run();
            }
            break;
    }
}
