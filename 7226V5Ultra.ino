/*
Manual TCU Controller for Mercedes 722.6 (ESP32 Core 3.x)
MAIN CORE - V5.0 - DUAL CORE & HIGH FREQUENCY PHYSICS
*/

#include <Arduino.h>
#include <driver/pcnt.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <math.h>

// ---------------- Constants ----------------------------------------------------
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// PCNT Config for Continuous Counting (Delta Method)
#define PCNT_H_LIM 32767
#define PCNT_L_LIM -32768
#define PCNT_FILTER_VAL 20   // Filter 

#define ANALOG_MAX 4095

// Vehicle / sensor constants
#define RPM_MIN 600
#define RPM_MAX 7000

// UPDATED PPR SETTINGS
#define PPR_ENGINE 58         // 60-2 Trigger Wheel
#define PPR_OUT_SHAFT 24
#define PPR_TURBINE_N2 60     // Updated to 60 based on feedback
#define PPR_TURBINE_N3 60     // Updated to 60 based on feedback

#define TIRE_CIRCUMFERENCE_M 2.0
#define FINAL_DRIVE_RATIO 3.27

const float N2_N3_GEAR_RATIO[5] = { 2.370f, 1.520f, 1.000f, 0.780f, 0.650f };
const float INPUT_RPM_EMA_ALPHA = 0.3f; 

// ---------------- UPDATED PIN MAPPING (ESP32 DevKit V1) ----------------------
// Solenoids / PWM outputs (Digital Output)
const int MPC_SOL = 26; // Line Pressure (GPIO25)
const int SPC_SOL = 25; // Shift Pressure (GPIO26)
const int SOL_12  = 14; // Solenoid 1-2 (GPIO27)
const int SOL_23  = 19; // Solenoid 2-3 (GPIO14)
const int SOL_34  = 18; // Solenoid 3-4 (GPIO18)
const int SOL_TCC = 27; // Torque Converter (GPIO19)


// Speed / Turbine / RPM pulse sensors (PCNT Inputs)
// NOTE: GPIO 34, 35, 36, 39 are INPUT ONLY. 
// If using Open Collector/Hall sensors, you MUST add external physical pull-up resistors (4.7k-10k) to 3.3V.
const int N2_SEN    = 34; // Input/Output pin (Internal pullup ok)
const int N3_SEN    = 35; // Input/Output pin (Internal pullup ok)
const int OUT_SPEED = 32; // Input Only - REQUIRES EXTERNAL PULLUP
const int ENGINE_RPM_SEN = 33; // Input Only - REQUIRES EXTERNAL PULLUP


// Analog sensors (ADC1 - Safe for WiFi)
const int TPS       = 36; // Input Only (VP)
const int TEMP_SEN  = 39; // Input Only (VN)

// Shifter / User switches (Digital Inputs with Pullups)
const int UP_SHIFT  = 21; // Upshift (GPIO21)
const int DN_SHIFT  = 22; // Downshift (GPIO22)

// ---------------- MERCEDES 4-BIT SHIFTER MAPPING ----------------
// Uses 4 wires to encode P, R, N, D, 4, 3, 2, 1
// Pins must have Pullups (Internal or External)
const int SHIFTER_PIN_A = 4;  
const int SHIFTER_PIN_B = 16;
const int SHIFTER_PIN_C = 17;
const int SHIFTER_PIN_D = 5;

// Active level config: Mercedes shifters are typically active-LOW (pull line to ground)
// Set to false if your harness is truly active-HIGH.
const bool SHIFTER_ACTIVE_LOW = false;

// ---------------- THREADING & SHARED DATA ------------------------------------
TaskHandle_t WebTask;
SemaphoreHandle_t dataMutex; // Protects shared variables

// Volatile variables for inter-core sharing
volatile int shared_engineRPM = 0;
volatile float shared_vehicleSpeed = 0;
volatile float shared_inputRPM = 0;
volatile float shared_outputRPM = 0;
volatile float shared_tpsPercentage = 0;
volatile float shared_transTemp = 0;
volatile int shared_currentGear = 99;
volatile int shared_tpsIndex = 0;
volatile int shared_rpmIndex = 0;

// Solenoid Live Tracking (Shared)
volatile int live_MPC = 0;
volatile int live_SPC = 0;
volatile int live_TCC = 0;

// ---------------- ENUMS & STRUCTS --------------------------------------------
enum ShifterPosition { 
    SH_POS_SIGNAL_NA = -1, 
    SH_POS_P = 0, 
    SH_POS_R_REV = 1, // Reverse (Mechanical linkage usually handles actual gear R)
    SH_POS_N = 2, 
    SH_POS_D = 3,
    SH_POS_4 = 4, 
    SH_POS_3 = 5, 
    SH_POS_2 = 6, 
    SH_POS_1 = 7
};

enum ShiftSMState { SM_IDLE=0, SM_PREFILL, SM_OPEN_BURST, SM_HOLDING, SM_VERIFY, SM_DONE };

struct GearRatioInfo { float ratio; float ratio_min_drift; float ratio_max_drift; };

struct ShiftParameters {
    int16_t FILL_time[8][8];
    int16_t FILL_duty[8][8];
    int16_t SHIFT_duty[8][8];
    int16_t SHIFT_MPC_duty[8][8]; 
    int16_t TARGET_SHIFT_time[8][8]; 
};

struct GeneralMaps {
    int16_t LINE_duty[8][8];
    int16_t TCC_duty[8][8];
};

struct PersistedData {
    uint8_t version;
    uint8_t adaptEnabled[8];
    ShiftParameters gearMaps[8];
    GeneralMaps genMaps;
};

struct ShiftLogEntry {
    unsigned long ts;
    int fromGear;
    int toGear;
    int mapIndex;
    unsigned long durationMs;
    bool success;
    String note;
};

// ---------------- Global Variables (Physics Core) -----------------------------
Preferences preferences;
Preferences tempPrefs;

bool garageShifting = false;
bool normalShifting = false;
bool shiftPending = false;
int requestedGear = 0;

int openingPwm = 195;
int holdingPwm = 70;
int tpsIndex = 0;
int rpmIndex = 0;

#define GEAR_PARK     0
#define GEAR_NEUTRAL  99
#define GEAR_R1      -1
#define GEAR_R2      -2

int currentGear = GEAR_NEUTRAL;
int targetGear  = 0;

// Timers for high speed loop
unsigned long lastSensorReadMicros = 0;
const unsigned long SENSOR_INTERVAL_MICROS = 10000; // 10ms (100Hz)
unsigned long lastControlLoopMicros = 0;
const unsigned long CONTROL_INTERVAL_MICROS = 1000; // 1ms (1000Hz)

int engineRPM = 0;
float vehicleSpeed = 0;
float inputRPM = 0;
float outputRPM = 0;
float tpsPercentage = 0;
float transTemp = 0;
static float inputRPM_filtered = 0.0f;
static float engineRPM_averaged = 0.0f; // Smoother for 58PPR noise

int tpsMin = 0;
int tpsMax = ANALOG_MAX;

unsigned long last_shift_time = 0;
unsigned long lastShiftDuration = 0; 
const unsigned long MIN_SHIFT_INTERVAL_MS = 500;
const unsigned long SHIFT_TIMEOUT_MS = 1500; // GLOBAL CONSTANT TIMEOUT

// Temp scaling (tunable)
int16_t temp_bp[4]   = { -20, 5, 25, 60 };   // °C breakpoints
int16_t temp_gain[4] = { 350, 200, 130, 100 }; // x100 gains (3.50, 2.00, 1.30, 1.00)

// Observed shift times (last good) per map/tps/rpm
uint16_t observedShiftMs[8][8][8] = {0};

// Closed-loop slip control constants (P-only, bounded)
const float SLIP_TARGET_MIN_CL    = 80.0f;
const float SLIP_TARGET_MID_CL    = 120.0f;
const float SLIP_TARGET_MAX_CL    = 220.0f;   // clamp threshold during control
const float SLIP_HARD_IGNORE      = 3000.0f;  // ignore absurd spikes
const float Kp_spc = 0.04f;  // PWM per rpm error for SPC
const float Kp_mpc = 0.02f;  // PWM per rpm error for MPC
const int   SPC_STEP_CLAMP  = 8;   // max PWM step per update
const int   MPC_STEP_CLAMP  = 4;
const int   SPC_HEADROOM    = 40;  // around base map value
const int   MPC_HEADROOM    = 30;  // around base map value

void loadTempScale() {
    if (!tempPrefs.begin("tcu-temp", true)) return;
    if (tempPrefs.isKey("bp0")) {
        for (int i = 0; i < 4; ++i) temp_bp[i]   = tempPrefs.getShort(("bp"+String(i)).c_str(), temp_bp[i]);
        for (int i = 0; i < 4; ++i) temp_gain[i] = tempPrefs.getShort(("gn"+String(i)).c_str(), temp_gain[i]);
    }
    tempPrefs.end();
}
void saveTempScale() {
    if (!tempPrefs.begin("tcu-temp", false)) return;
    for (int i = 0; i < 4; ++i) tempPrefs.putShort(("bp"+String(i)).c_str(), temp_bp[i]);
    for (int i = 0; i < 4; ++i) tempPrefs.putShort(("gn"+String(i)).c_str(), temp_gain[i]);
    tempPrefs.end();
}

void setPWM(int pin, int duty) {
    ledcWrite(pin, duty);
    // Update volatile trackers
    if(pin == MPC_SOL) live_MPC = duty;
    else if(pin == SPC_SOL) live_SPC = duty;
    else if(pin == SOL_TCC) live_TCC = duty;
}

// helper clamp
static inline int clampInt(int v, int lo, int hi){ return (v<lo)?lo:((v>hi)?hi:v); }

ShifterPosition shifter_pos = SH_POS_SIGNAL_NA;
unsigned long shifter_debounce_ms = 50;
unsigned long last_shifter_change_time = 0;

unsigned long lastGarageEnd = 0;
const unsigned long GARAGE_COOLDOWN_MS = 300;

const float GB_SMALL_RATIOS[5] = {3.93f, 2.41f, 1.49f, 1.00f, 0.83f};
const float GB_BIG_RATIOS[5]   = {3.59f, 2.19f, 1.41f, 1.00f, 0.83f};
const char* PREF_GB_KEY = "gb_type";
GearRatioInfo gearboxBounds[5];
float sensor_2_1_ratio = 1.0f;

ShiftParameters shiftMaps[8];
GeneralMaps generalMaps;
float adaptEma[8][8][8];

const char *MAPS_BIN_FILE = "/maps_v5.bin";
const uint8_t MAPS_VERSION = 11; // Bumped version
bool adaptEnabled[8] = {true,true,true,true,true,true,true,true};

const int SHIFT_LOG_LEN = 256;
ShiftLogEntry shiftLog[SHIFT_LOG_LEN];
int shiftLogHead = 0;
int shiftLogCount = 0;

// Forward Declares
void persistMapsBinary(); 
// Include Dashboard LAST so it can see globals
#include "WebDashboard.h" 

void pushShiftLog(const ShiftLogEntry &e){
    if(xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
        shiftLog[shiftLogHead] = e;
        shiftLogHead = (shiftLogHead+1) % SHIFT_LOG_LEN;
        if(shiftLogCount < SHIFT_LOG_LEN) shiftLogCount++;
        if(e.durationMs > 0) lastShiftDuration = e.durationMs;
        xSemaphoreGive(dataMutex);
    }
}

struct TimedPulse { int pin; int duty; unsigned long endTime; bool active; } tp = {-1,0,0,false};
void scheduleTimedPulse(int pin, int duty, unsigned long ms){
    if(pin < 0) return;
    duty = constrain(duty,0,255);
    tp.pin = pin; tp.duty = duty; tp.endTime = millis()+ms; tp.active = true;
    setPWM(pin,duty);
}
void processTimedPulse(){
    if(!tp.active) return;
    if((long)(millis()-tp.endTime) >= 0){
        setPWM(tp.pin,0);
        tp.active=false;
    }
}

#ifndef ENABLE_DEBUG
#define ENABLE_DEBUG 1
#endif
#define DBG(...) do{ if(ENABLE_DEBUG){ Serial.printf(__VA_ARGS__); } }while(0)


void set_sensor_ratio(float ratio){
    sensor_2_1_ratio = (ratio>0.0f)?ratio:1.0f;
}
void apply_ratios_and_set_sensor_ratio(const float ratios[5]){
    for(int i=0;i<5;i++){
        float r=ratios[i];
        gearboxBounds[i].ratio=r;
        gearboxBounds[i].ratio_min_drift=r*0.90f;
        gearboxBounds[i].ratio_max_drift=r*1.10f;
    }
    if(ratios[1]!=0.0f) set_sensor_ratio(ratios[0]/ratios[1]); else set_sensor_ratio(1.0f);
}

// ---------------- Persistence (FIXED for Stack Overflow) -----------------------
void persistMapsBinary(){
    // ALLOCATE ON HEAP TO SAVE STACK
    PersistedData *pd = new PersistedData();
    if(!pd) { DBG("OOM Persist\n"); return; }
    
    pd->version = MAPS_VERSION;
    for(int i=0;i<8;i++) pd->adaptEnabled[i] = adaptEnabled[i]?1:0;
    for(int m=0;m<8;m++) pd->gearMaps[m] = shiftMaps[m];
    pd->genMaps = generalMaps;

    File f = LittleFS.open(MAPS_BIN_FILE,"w");
    if(!f){ DBG("maps write failed\n"); delete pd; return; }
    f.write((const uint8_t*)pd,sizeof(PersistedData));
    f.close();
    
    delete pd; // FREE MEMORY
    DBG("V5 Maps persisted (Heap)\n");
}
bool loadMapsBinary(){
    if(!LittleFS.exists(MAPS_BIN_FILE)) return false;
    File f=LittleFS.open(MAPS_BIN_FILE,"r");
    if(!f) return false;
    
    // ALLOCATE ON HEAP
    PersistedData *pd = new PersistedData();
    if(!pd) { f.close(); return false; }
    
    size_t rd = f.readBytes((char*)pd,sizeof(PersistedData));
    f.close();
    
    if(rd != sizeof(PersistedData) || pd->version != MAPS_VERSION) {
        delete pd;
        return false;
    }
    
    for(int i=0;i<8;i++) adaptEnabled[i] = (pd->adaptEnabled[i]!=0);
    for(int m=0;m<8;m++) shiftMaps[m] = pd->gearMaps[m];
    generalMaps = pd->genMaps;
    
    delete pd; // FREE MEMORY
    return true;
}
void saveAdaptiveData(){
    Preferences p; p.begin("adapt",false);
    p.putBytes("ema",adaptEma,sizeof(adaptEma));
    p.end();
}
void loadAdaptiveData(){
    Preferences p; p.begin("adapt",true);
    if(p.isKey("ema")){ p.getBytes("ema",adaptEma,sizeof(adaptEma)); }
    else { memset(adaptEma,0,sizeof(adaptEma)); }
    p.end();
}

// V5 Defaults - 80C Base
void initV5Defaults(){
    for(int m=0;m<8;m++) {
        // Base target times per map index (legacy default values)
        int defaultTarget = 500;
        if(m==0) defaultTarget = 450;
        else if(m==1) defaultTarget = 470;
        else if(m==2) defaultTarget = 480;
        else if(m==3) defaultTarget = 500;
        else if(m==4) defaultTarget = 520;
        else if(m==5) defaultTarget = 540;
        else if(m==6) defaultTarget = 560;
        else if(m==7) defaultTarget = 580;

        for(int t=0;t<8;t++) {
            for(int r=0;r<8;r++) {
                int fillTime = 150; 
                if(m==0) fillTime = 160; 
                else if(m==1) fillTime = 120;
                else if(m==2) fillTime = 140;
                else if(m==3) fillTime = 140;
                else if(m==4) fillTime = 140;
                else if(m==5) fillTime = 120;
                else if(m==6) fillTime = 130;
                else if(m==7) fillTime = 130;

                shiftMaps[m].FILL_time[t][r] = fillTime;
                shiftMaps[m].FILL_duty[t][r] = 160; 
                shiftMaps[m].SHIFT_duty[t][r] = 140 - (t*8); 
                shiftMaps[m].SHIFT_MPC_duty[t][r] = 120; 
                shiftMaps[m].TARGET_SHIFT_time[t][r] = defaultTarget;
            }
        }
    }
    
    for(int t=0;t<8;t++) {
        for(int r=0;r<8;r++) {
            int mpc = 240 - (t * 30); 
            generalMaps.LINE_duty[t][r] = constrain(mpc, 0, 255);
            generalMaps.TCC_duty[t][r] = 0;
        }
    }
    persistMapsBinary();
}
void initDefaultMaps(){ initV5Defaults(); }
void saveMapsToFS(){ persistMapsBinary(); }
bool loadMapsFromFS(){ return loadMapsBinary(); }

void saveCalibration(){
    Preferences p; p.begin("tps-cal",false);
    p.putInt("tpsMin",tpsMin);
    p.putInt("tpsMax",tpsMax);
    p.end();
}
void loadCalibration(){
    Preferences p; p.begin("tps-cal",true);
    tpsMin = p.getInt("tpsMin",0);
    tpsMax = p.getInt("tpsMax",ANALOG_MAX);
    p.end();
}

// ---------------- Helper Functions -------------------------------------------
float getTemperatureScalingFactor() {
    float t = transTemp;
    // enforce non-decreasing breakpoints if user entered descending
    for (int i = 1; i < 4; ++i) {
        if (temp_bp[i] < temp_bp[i-1]) temp_bp[i] = temp_bp[i-1];
    }
    if (t <= temp_bp[0]) return temp_gain[0] / 100.0f;
    if (t >= temp_bp[3]) return temp_gain[3] / 100.0f;
    for (int i = 0; i < 3; ++i) {
        if (t <= temp_bp[i+1]) {
            float x0 = temp_bp[i],   x1 = temp_bp[i+1];
            float y0 = temp_gain[i], y1 = temp_gain[i+1];
            float f = y0 + (y1 - y0) * ((t - x0) / (x1 - x0 + 0.0001f));
            return f / 100.0f;
        }
    }
    return 1.0f;
}

float readTPS(){
    int raw=analogRead(TPS);
    if(tpsMax>tpsMin){
        long mv=map(raw,tpsMin,tpsMax,0,100);
        return constrain((int)mv,0,100);
    }
    return 0;
}
float readTemperature(){
    int raw=analogRead(TEMP_SEN);
    float v=(raw/(float)ANALOG_MAX)*3.3f;
    float t=(v-0.5f)*100.0f;
    return constrain(t,-40.0f,150.0f);
}

// ---------------- SHIFTER LOGIC (MERCEDES 4-BIT) ----------------
// Decodes 4 input pins into a logical shifter position using Mercedes PRND4321 codes.
// Codes: P=0110, R=0111, N=1110, D=1100, 4=1101, 3=1001, 2=1011, 1=1010 (rnd-ash ultimate NAG scheme).
static ShifterPosition decodeMercedesCode(uint8_t code){
    switch(code){
        case 0b0110: return SH_POS_P;
        case 0b0111: return SH_POS_R_REV;
        case 0b1110: return SH_POS_N;
        case 0b1100: return SH_POS_D;
        case 0b1101: return SH_POS_4;
        case 0b1001: return SH_POS_3;
        case 0b1011: return SH_POS_2;
        case 0b1010: return SH_POS_1;
        default:     return SH_POS_SIGNAL_NA;
    }
}

ShifterPosition readShifterPosition(){
    static ShifterPosition last_valid = SH_POS_SIGNAL_NA; // holds last valid decode

    int a = digitalRead(SHIFTER_PIN_A);
    int b = digitalRead(SHIFTER_PIN_B);
    int c = digitalRead(SHIFTER_PIN_C);
    int d = digitalRead(SHIFTER_PIN_D);
    
    // Combine into a 4-bit integer (bit0=A, bit1=B, bit2=C, bit3=D)
    uint8_t val = (d << 3) | (c << 2) | (b << 1) | a;

    // If the harness is active-LOW (lines pulled to GND), invert so logical "1" means asserted/high.
    if(SHIFTER_ACTIVE_LOW){
        val = (~val) & 0x0F;
    }

    ShifterPosition decoded = decodeMercedesCode(val);
    if(decoded != SH_POS_SIGNAL_NA){
        last_valid = decoded;       // remember good state
        return decoded;
    }
    // If invalid/noise, return the last known valid position (or SIGNAL_NA if none yet)
    return last_valid;
}

void controlTCC(){
    if(currentGear>=3 && transTemp>=60){
        int duty = generalMaps.TCC_duty[tpsIndex][rpmIndex];
        setPWM(SOL_TCC, duty);
    } else {
        setPWM(SOL_TCC, 0);
    }
}

void controlSteadyStatePressure(){
    if(!normalShifting && !garageShifting){
        int mpc = generalMaps.LINE_duty[tpsIndex][rpmIndex];
        if(transTemp < 40) mpc = max(0, mpc - 50);
        setPWM(MPC_SOL, mpc);
    }
}

void setupPCNT(pcnt_unit_t unit,int pin){
    pcnt_config_t c={
        .pulse_gpio_num=pin,
        .ctrl_gpio_num=PCNT_PIN_NOT_USED,
        .lctrl_mode=PCNT_MODE_KEEP,
        .hctrl_mode=PCNT_MODE_KEEP,
        .pos_mode=PCNT_COUNT_INC,
        .neg_mode=PCNT_COUNT_DIS,
        .counter_h_lim=PCNT_H_LIM,
        .counter_l_lim=PCNT_L_LIM,
        .unit=unit,
        .channel=PCNT_CHANNEL_0
    };
    pcnt_unit_config(&c);
    pcnt_set_filter_value(unit,PCNT_FILTER_VAL);
    pcnt_filter_enable(unit);
    // Don't clear here, just resume to allow continuous counting
    pcnt_counter_resume(unit);
}

float filter_input_rpm(float raw_rpm) {
    if (raw_rpm <= 0.0f) {
        inputRPM_filtered *= 0.9f; 
        if(inputRPM_filtered < 10.0f) inputRPM_filtered = 0.0f;
        return inputRPM_filtered;
    }
    if (inputRPM_filtered <= 0.0f) inputRPM_filtered = raw_rpm; 
    else inputRPM_filtered = INPUT_RPM_EMA_ALPHA * raw_rpm + (1.0f - INPUT_RPM_EMA_ALPHA) * inputRPM_filtered;
    return inputRPM_filtered;
}

float calc_turbine_rpm_from_n2n3_v2(int32_t n2, int32_t n3, float dt, 
                                     float ppr_n2, float ppr_n3, int gear) {
    if (dt <= 0.0001f || ppr_n2 <= 0 || ppr_n3 <= 0) return 0.0f;
    if (gear < 1 || gear > 5) {
        float n2hz = (n2 >= 0) ? (n2 / dt) / ppr_n2 : 0.0f;
        float n3hz = (n3 >= 0) ? (n3 / dt) / ppr_n3 : 0.0f;
        return max(n2hz, n3hz) * 60.0f;
    }
    float n2hz = (n2 >= 0) ? (n2 / dt) / ppr_n2 : 0.0f;
    float n3hz = (n3 >= 0) ? (n3 / dt) / ppr_n3 : 0.0f;
    float ratio = N2_N3_GEAR_RATIO[gear - 1];
    if (gear <= 3) {
        if (n3hz < 0.5f) return (n2hz >= 0.5f) ? (n2hz / ratio) * 60.0f : 0.0f;
        return n3hz * 60.0f;
    } else {
        if (n2hz < 0.5f) return (n3hz >= 0.5f) ? n3hz * 60.0f : 0.0f;
        return (n2hz / ratio) * 60.0f;
    }
}

// V5: High Frequency Speed Sensor Loop (Runs at 100Hz / 10ms)
// UPDATED: Now uses continuous delta tracking to prevent missed pulses
void speedSensors(){
    unsigned long now = micros();
    if(now - lastSensorReadMicros < SENSOR_INTERVAL_MICROS) return;
    
    // Static variables to remember previous counter values
    static int16_t last_n2 = 0, last_n3 = 0, last_out = 0, last_eng = 0;
    int16_t curr_n2=0, curr_n3=0, curr_out=0, curr_eng=0;

    // Get current counts (don't clear)
    pcnt_get_counter_value(PCNT_UNIT_0, &curr_n2);
    pcnt_get_counter_value(PCNT_UNIT_1, &curr_n3);
    pcnt_get_counter_value(PCNT_UNIT_2, &curr_out);
    pcnt_get_counter_value(PCNT_UNIT_3, &curr_eng);
    
    // Calculate Delta (handles wrap-around automatically with int16 math)
    int16_t d_n2 = curr_n2 - last_n2;
    int16_t d_n3 = curr_n3 - last_n3;
    int16_t d_out = curr_out - last_out;
    int16_t d_eng = curr_eng - last_eng;

    // Update last for next loop
    last_n2 = curr_n2;
    last_n3 = curr_n3;
    last_out = curr_out;
    last_eng = curr_eng;
    
    float dt = SENSOR_INTERVAL_MICROS / 1000000.0f; // Should be 0.01s
    
    // 1. Engine RPM (58 PPR)
    // Use Delta 'd_eng' instead of raw count
    float rawEng = ((d_eng/dt)/PPR_ENGINE)*60.0f;
    // Add simple rolling average for 10ms samples (Alpha 0.2)
    // This helps smooth out the 60-2 trigger wheel gap effect
    engineRPM_averaged = (rawEng * 0.2f) + (engineRPM_averaged * 0.8f);
    engineRPM = (int)engineRPM_averaged;
    
    // 2. Output RPM (24 PPR) - OK at speed, rough at low speed
    outputRPM = ((d_out/dt)/PPR_OUT_SHAFT)*60.0f;

    int gearForCalc = currentGear;
    if(gearForCalc < 1 || gearForCalc > 5) gearForCalc = 1; 

    // Use Deltas for Turbine Calc
    float rawInputRPM = calc_turbine_rpm_from_n2n3_v2(
        d_n2, d_n3, dt, PPR_TURBINE_N2, PPR_TURBINE_N3, gearForCalc
    );
    inputRPM = filter_input_rpm(rawInputRPM);

    float wheelRPS = (d_out/dt)/ (PPR_OUT_SHAFT*FINAL_DRIVE_RATIO);
    vehicleSpeed = wheelRPS*TIRE_CIRCUMFERENCE_M*3.6f;
    
    // Update shared variables for Web Thread
    if(xSemaphoreTake(dataMutex, 0) == pdTRUE) {
        shared_engineRPM = engineRPM;
        shared_vehicleSpeed = vehicleSpeed;
        shared_inputRPM = inputRPM;
        shared_outputRPM = outputRPM;
        xSemaphoreGive(dataMutex);
    }

    lastSensorReadMicros = now;
}

const int SLIP_TARGET_MAX=220;
const int SLIP_TARGET_MIN=80;
const int FLARE_THRESHOLD=600;
const int BIND_THRESHOLD=50;
const int SOFT_TOLERANCE_MS=120;
const int HARD_TOLERANCE_MS=-120;

float computeSlip(float inRpm,float outRpm,int targetGear){
    if(targetGear<1||targetGear>5) return 9999.0f;
    if(outRpm<50.0f) return 9999.0f;
    float expected=gearboxBounds[targetGear-1].ratio;
    return fabs(inRpm - outRpm*expected);
}

// DEFINING GARAGE SHIFT STRUCTURES BEFORE USE
enum GaragePhase { GS_IDLE=0, GS_OPEN_BURST, GS_HOLDING_RAMP, GS_FINISH };
struct GarageState {
    bool active=false; bool intoReverse=false;
    unsigned long phaseStart=0; GaragePhase phase=GS_IDLE;
    int spc_prefill=0; int mpc_working=0;
    unsigned long elapsed=0; bool completed_ok=false;
} garageState;

void garageShiftStart(bool intoReverse){
    if(millis()-lastGarageEnd < GARAGE_COOLDOWN_MS) return;
    if(garageState.active || normalShifting) return;
    garageShifting=true; garageState.active=true;
    garageState.intoReverse=intoReverse;
    garageState.phase=GS_OPEN_BURST;
    garageState.phaseStart=millis();
    garageState.spc_prefill = 100; 
    garageState.mpc_working = 120;
    setPWM(SPC_SOL, garageState.spc_prefill/2);
    setPWM(SOL_34, openingPwm);
    DBG("Garage shift start\n");
}
void garageShiftHandler(){
    if(!garageState.active) return;
    unsigned long now=millis();
    switch(garageState.phase){
        case GS_OPEN_BURST:
            if(now - garageState.phaseStart >= 50){
                setPWM(SOL_34, holdingPwm);
                garageState.phase=GS_HOLDING_RAMP;
                garageState.phaseStart=now;
            }
            break;
        case GS_HOLDING_RAMP:{
            garageState.elapsed = now - garageState.phaseStart;
            float rampFrac = min((float)garageState.elapsed/2500.0f, 2.0f);
            int spc=constrain((int)((garageState.spc_prefill+20)*(1.0f+rampFrac)),0,255);
            int mpc=constrain(garageState.mpc_working+(spc/2),0,255);
            setPWM(SPC_SOL,spc); setPWM(MPC_SOL,mpc);
            float expected = (!garageState.intoReverse)?
                outputRPM*gearboxBounds[1].ratio :
                outputRPM*fabs(gearboxBounds[0].ratio);
            if(garageState.elapsed>1000 && inputRPM<=expected+100){
                garageState.completed_ok=true;
                garageState.phase=GS_FINISH; garageState.phaseStart=now;
            } else if(garageState.elapsed>=2500){
                garageState.completed_ok=false;
                garageState.phase=GS_FINISH; garageState.phaseStart=now;
            }
        }break;
        case GS_FINISH:
            if(!garageState.completed_ok){
                setPWM(SPC_SOL, garageState.spc_prefill/2);
                setPWM(SOL_34,0);
                currentGear=GEAR_NEUTRAL;
                pushShiftLog({millis(),GEAR_PARK,GEAR_NEUTRAL,-1,garageState.elapsed,false,"garage_failed"});
            }else{
                setPWM(SPC_SOL,0);
                setPWM(SOL_34,0);
                currentGear = garageState.intoReverse?GEAR_R2:2;
                pushShiftLog({millis(),GEAR_PARK,currentGear,-1,garageState.elapsed,true,"garage_ok"});
            }
            garageState.active=false; garageShifting=false;
            garageState.phase=GS_IDLE; lastGarageEnd=millis();
            break;
        default:
            garageState.active=false; garageShifting=false;
            garageState.phase=GS_IDLE; lastGarageEnd=millis();
            break;
    }
}

struct ShiftGuard{ int minRPM,maxRPM,minSpeed,maxSpeed; };
ShiftGuard upShiftGuards[4]   = {{2000,6000,10,60},{2500,6500,15,80},{3000,7000,20,100},{3500,7500,25,120}};
ShiftGuard downShiftGuards[4] = {{1500,4000,0,50},{2000,4500,5,60},{2500,5000,10,70},{3000,5500,15,80}};

bool safeToShiftUp(int gear){
    if(gear<1||gear>=5) return false;
    ShiftGuard sg=upShiftGuards[gear-1];
    return engineRPM>=sg.minRPM && engineRPM<=sg.maxRPM &&
           vehicleSpeed>=sg.minSpeed && vehicleSpeed<=sg.maxSpeed;
}
bool safeToShiftDown(int gear){
    if(gear<=1||gear>5) return false;
    ShiftGuard sg=downShiftGuards[gear-2];
    return engineRPM>=sg.minRPM && engineRPM<=sg.maxRPM &&
           vehicleSpeed>=sg.minSpeed && vehicleSpeed<=sg.maxSpeed;
}
void GearShiftScheduler(){
    if(garageShifting) return;
    if(millis()-last_shift_time < MIN_SHIFT_INTERVAL_MS) return;
    bool up=(digitalRead(UP_SHIFT)==LOW);
    bool dn=(digitalRead(DN_SHIFT)==LOW);
    if(up && dn) return;
    if(!normalShifting && !shiftPending){
        if(up && currentGear<5 && safeToShiftUp(currentGear)){
            requestedGear=currentGear+1; shiftPending=true;
        }else if(dn && currentGear>1 && safeToShiftDown(currentGear)){
            requestedGear=currentGear-1; shiftPending=true;
        }
    }
}

struct ShiftSM{
    ShiftSMState state=SM_IDLE;
    int mapIndex=-1;
    int shiftSolenoidPin=-1;
    int expectedTargetGear=0;
    unsigned long stateStart=0;
    unsigned long shiftStart=0;
    int confirmationHits=0;
    int fill_duty=0;
    int shift_duty=0;
    int shift_mpc_duty=0; 
    unsigned long lastSample=0;
    float peakSlip=0.0f;
    float finalSlip=0.0f;
    // Closed-loop cmds (per-shift)
    int spc_cmd=0;
    int mpc_cmd=0;
} shiftSM;

void performShiftInit(int mapIdx,int solenoidPin,int expectedGear){
    normalShifting=true;
    shiftSM.mapIndex=mapIdx;
    shiftSM.shiftSolenoidPin=solenoidPin;
    shiftSM.expectedTargetGear=expectedGear;
    shiftSM.state=SM_PREFILL;
    shiftSM.stateStart=millis();
    shiftSM.shiftStart=shiftSM.stateStart;
    shiftSM.confirmationHits=0;
    
    shiftSM.fill_duty = shiftMaps[mapIdx].FILL_duty[tpsIndex][rpmIndex];
    shiftSM.shift_duty = shiftMaps[mapIdx].SHIFT_duty[tpsIndex][rpmIndex];
    shiftSM.shift_mpc_duty = shiftMaps[mapIdx].SHIFT_MPC_duty[tpsIndex][rpmIndex];
    
    shiftSM.lastSample=0;
    shiftSM.peakSlip=0; shiftSM.finalSlip=0;

    // Initialize per-shift commands from map
    shiftSM.spc_cmd = shiftSM.shift_duty;
    shiftSM.mpc_cmd = shiftSM.shift_mpc_duty;

    setPWM(SPC_SOL, shiftSM.fill_duty); 
    setPWM(MPC_SOL, shiftSM.shift_mpc_duty);
    
    // Sync ShiftState to Shared (Optional, for display)
    if(xSemaphoreTake(dataMutex, 0)==pdTRUE){
        // Can expose detailed shift status later
        xSemaphoreGive(dataMutex);
    }

    DBG("Shift V5: G%d->G%d\n", currentGear, expectedGear);
}

int estimateGearFromRatioGuarded(float inRpm,float outRpm){
    if(outRpm<80.0f || outRpm<=1.0f || inRpm<=1.0f) return currentGear;
    float ratio=inRpm/outRpm;
    for(int i=0;i<5;i++){
        GearRatioInfo g=gearboxBounds[i];
        if(ratio>=g.ratio_min_drift && ratio<=g.ratio_max_drift) return i+1;
    }
    return currentGear;
}

static unsigned long lastAdaptPersist = 0;
static int adaptSuccessCounter = 0;
const unsigned long ADAPT_PERSIST_INTERVAL_MS = 60000;
const int ADAPT_SUCCESS_BATCH = 5;

void maybePersistAdapt(){
    unsigned long now=millis();
    if(adaptSuccessCounter >= ADAPT_SUCCESS_BATCH || (now - lastAdaptPersist) > ADAPT_PERSIST_INTERVAL_MS){
        saveAdaptiveData();
        saveMapsToFS();
        adaptSuccessCounter=0;
        lastAdaptPersist=now;
    }
}

void performShiftStateMachine(){
    if(garageShifting) return;
    unsigned long now=millis();
    
    if(shiftSM.state==SM_IDLE){
        if(!shiftPending || normalShifting) return;
        targetGear=requestedGear;
        int mapIdx=-1, pin=-1, expected=targetGear;
        if(expected==currentGear+1){
            if(expected==2) mapIdx=0, pin=SOL_12;
            else if(expected==3) mapIdx=1, pin=SOL_23;
            else if(expected==4) mapIdx=2, pin=SOL_34;
            else if(expected==5) mapIdx=3, pin=SOL_12;
        }else if(expected==currentGear-1){
            if(expected==1) mapIdx=4, pin=SOL_12;
            else if(expected==2) mapIdx=5, pin=SOL_23;
            else if(expected==3) mapIdx=6, pin=SOL_34;
            else if(expected==4) mapIdx=7, pin=SOL_12;
        }else{
            shiftPending=false; return;
        }
        if(mapIdx>=0){
            performShiftInit(mapIdx,pin,expected);
            shiftPending=false;
        }
        return;
    }
    
    if(shiftSM.state==SM_PREFILL){
        int base_fill = shiftMaps[shiftSM.mapIndex].FILL_time[tpsIndex][rpmIndex];
        float scaler = getTemperatureScalingFactor();
        int actual_fill = (int)((float)base_fill * scaler);

        if(now - shiftSM.stateStart >= (unsigned long)actual_fill){
            setPWM(shiftSM.shiftSolenoidPin, openingPwm);
            setPWM(SPC_SOL, shiftSM.shift_duty);
            shiftSM.stateStart=now;
            shiftSM.state=SM_OPEN_BURST;
        }
        return;
    }
    
    if(shiftSM.state==SM_OPEN_BURST){
        if(now - shiftSM.stateStart >= 60){
            setPWM(shiftSM.shiftSolenoidPin, holdingPwm);
            shiftSM.stateStart=now;
            shiftSM.state=SM_HOLDING;
        }
        return;
    }
    
    if(shiftSM.state==SM_HOLDING){
        // Apply closed-loop slip control every 50 ms (reuse existing cadence)
        if(now - shiftSM.lastSample >= 50){
            shiftSM.lastSample=now;
            float slip=computeSlip(inputRPM,outputRPM,shiftSM.expectedTargetGear);
            if(slip<5000.0f && slip>shiftSM.peakSlip) shiftSM.peakSlip=slip;
            
            int est=estimateGearFromRatioGuarded(inputRPM,outputRPM);
            bool ratioOk=(est==shiftSM.expectedTargetGear);
            bool slipOk=(slip<SLIP_TARGET_MAX);
            
            if(ratioOk && slipOk) shiftSM.confirmationHits++; else shiftSM.confirmationHits=0;
            if(shiftSM.confirmationHits >= 3){
                shiftSM.state=SM_VERIFY;
                shiftSM.stateStart=now;
            }

            // Closed-loop slip adjust (bounded P)
            if(slip > 0 && slip < SLIP_HARD_IGNORE && outputRPM > 50 && inputRPM > 100){
                float err = slip - SLIP_TARGET_MID_CL;
                int base_spc = shiftSM.shift_duty;
                int base_mpc = shiftSM.shift_mpc_duty;

                int delta_spc = clampInt((int)(Kp_spc * err), -SPC_STEP_CLAMP, SPC_STEP_CLAMP);
                int delta_mpc = clampInt((int)(Kp_mpc * err), -MPC_STEP_CLAMP, MPC_STEP_CLAMP);

                shiftSM.spc_cmd = clampInt(shiftSM.spc_cmd + delta_spc,
                                           clampInt(base_spc - SPC_HEADROOM,0,255),
                                           clampInt(base_spc + SPC_HEADROOM,0,255));

                shiftSM.mpc_cmd = clampInt(shiftSM.mpc_cmd + delta_mpc,
                                           clampInt(base_mpc - MPC_HEADROOM,0,255),
                                           clampInt(base_mpc + MPC_HEADROOM,0,255));
            }
        }

        // Apply current commands (closed-loop adjusted)
        setPWM(SPC_SOL, shiftSM.spc_cmd);
        setPWM(MPC_SOL, shiftSM.mpc_cmd);

        if(now - shiftSM.shiftStart >= SHIFT_TIMEOUT_MS){
            setPWM(shiftSM.shiftSolenoidPin,0);
            setPWM(SPC_SOL,0);
            setPWM(MPC_SOL,0);
            pushShiftLog({now,currentGear,shiftSM.expectedTargetGear,shiftSM.mapIndex,0,false,"timeout"});
            shiftSM.state=SM_DONE;
        }
        return;
    }
    
    if(shiftSM.state==SM_VERIFY){
        shiftSM.finalSlip=computeSlip(inputRPM,outputRPM,shiftSM.expectedTargetGear);
        int est=estimateGearFromRatioGuarded(inputRPM,outputRPM);
        if(est==shiftSM.expectedTargetGear && shiftSM.finalSlip < SLIP_TARGET_MAX){
            unsigned long dur=now - shiftSM.shiftStart;
            pushShiftLog({now,currentGear,shiftSM.expectedTargetGear,shiftSM.mapIndex,dur,true,""});
            
            // Record observed last-good shift time (clamped to uint16_t)
            if(shiftSM.mapIndex >=0 && shiftSM.mapIndex <8 &&
               tpsIndex>=0 && tpsIndex<8 && rpmIndex>=0 && rpmIndex<8){
                observedShiftMs[shiftSM.mapIndex][tpsIndex][rpmIndex] = (uint16_t)min(dur, (unsigned long)65535);
            }

            if(adaptEnabled[shiftSM.mapIndex]){
                int mapIdx=shiftSM.mapIndex;
                int tIdx=tpsIndex, rIdx=rpmIndex;
                
                int targetMs = shiftMaps[mapIdx].TARGET_SHIFT_time[tIdx][rIdx];
                int diff = (int)dur - targetMs;
                
                if(diff > 100) {
                    shiftMaps[mapIdx].SHIFT_duty[tIdx][rIdx] -= 2; 
                } else if(diff < -100) {
                    shiftMaps[mapIdx].SHIFT_duty[tIdx][rIdx] += 2; 
                }
                shiftMaps[mapIdx].SHIFT_duty[tIdx][rIdx] = constrain(shiftMaps[mapIdx].SHIFT_duty[tIdx][rIdx], 0, 255);
                adaptSuccessCounter++;
                maybePersistAdapt();
            }
            setPWM(shiftSM.shiftSolenoidPin,0);
            setPWM(SPC_SOL,0);
            currentGear=shiftSM.expectedTargetGear;
            
            if(xSemaphoreTake(dataMutex, 10)==pdTRUE){
                shared_currentGear = currentGear;
                xSemaphoreGive(dataMutex);
            }
            
            last_shift_time=now;
            shiftSM.state=SM_DONE;
        }else{
            pushShiftLog({now,currentGear,shiftSM.expectedTargetGear,shiftSM.mapIndex,0,false,"verify_fail"});
            setPWM(shiftSM.shiftSolenoidPin,0);
            setPWM(SPC_SOL,0);
            setPWM(MPC_SOL,0);
            shiftSM.state=SM_DONE;
        }
        return;
    }
    if(shiftSM.state==SM_DONE){
        normalShifting=false;
        shiftSM.state=SM_IDLE;
        shiftSM.mapIndex=-1;
        shiftSM.shiftSolenoidPin=-1;
        shiftSM.expectedTargetGear=0;
        shiftSM.confirmationHits=0;
        shiftSM.spc_cmd=0;
        shiftSM.mpc_cmd=0;
    }
}

void handleShifterChange(ShifterPosition last, ShifterPosition now){
    bool last_controllable=(last==SH_POS_D||last==SH_POS_R_REV);
    bool now_controllable=(now==SH_POS_D||now==SH_POS_R_REV);
    
    // Entering Garage Shift?
    if(!last_controllable && now_controllable){
        garageShiftStart(now==SH_POS_R_REV);
    }
    // Leaving Garage Shift (to P or N)?
    else if(last_controllable && !now_controllable){
        if(now==SH_POS_P){ currentGear=GEAR_PARK; targetGear=GEAR_PARK; }
        else if(now==SH_POS_N){ currentGear=GEAR_NEUTRAL; targetGear=GEAR_NEUTRAL; }
        
        if(xSemaphoreTake(dataMutex, 10)==pdTRUE){ shared_currentGear=currentGear; xSemaphoreGive(dataMutex); }
        
        setPWM(SPC_SOL, 80);
        scheduleTimedPulse(SOL_34, holdingPwm, 50);
        pushShiftLog({millis(),(int)last,(int)now,-1,0,true,"move_to_PN"});
    }
    // Mode Change (D -> 4 -> 3 etc)
    else if(now_controllable) {
        // Placeholder for future mode logic using 4/3/2/1
    }
}

// V5: Telemetry reads from Shared Volatile vars (Safe for Core 0)
// FIXED: Uses snprintf to prevent String memory fragmentation/crashes
String getExtendedTelemetryJSON() {
    char buf[512]; // Fixed buffer, no heap fragmentation
    
    int rpm=0, tMin=0, tMax=0, gr=99;
    float spd=0, tps=0, tmp=0, inRpm=0, outRpm=0;
    
    // Read protected data quickly
    if(xSemaphoreTake(dataMutex, 10) == pdTRUE) {
        rpm = shared_engineRPM;
        spd = shared_vehicleSpeed;
        tps = shared_tpsPercentage;
        tmp = shared_transTemp;
        gr = shared_currentGear;
        inRpm = shared_inputRPM;
        outRpm = shared_outputRPM;
        tMin = tpsMin;
        tMax = tpsMax;
        xSemaphoreGive(dataMutex);
    }

    const char* sState = "IDLE";
    if(garageShifting) sState = "GARAGE";
    else {
        switch(shiftSM.state){
            case SM_PREFILL: sState="PREFILL"; break;
            case SM_OPEN_BURST: sState="BURST"; break;
            case SM_HOLDING: sState="HOLDING"; break;
            case SM_VERIFY: sState="VERIFY"; break;
            case SM_DONE: sState="DONE"; break;
            default: break;
        }
    }
    
    // Format safely into buffer
    snprintf(buf, sizeof(buf), 
        "{\"rpm\":%d,\"spd\":%.1f,\"tps\":%.0f,\"tmp\":%.0f,\"gr\":%d,\"inRpm\":%.0f,\"outRpm\":%.0f,"
        "\"tMin\":%d,\"tMax\":%d,\"mpc\":%d,\"spc\":%d,\"tcc\":%d,"
        "\"ti\":%d,\"ri\":%d,\"uptime\":%lu,\"sState\":\"%s\",\"lTime\":%lu}",
        rpm, spd, tps, tmp, gr, inRpm, outRpm, tMin, tMax,
        live_MPC, live_SPC, live_TCC, tpsIndex, rpmIndex, millis(), sState, lastShiftDuration
    );
    
    return String(buf);
}


// WEB SERVER TASK (Core 0)
void WebServerTask(void * parameter) {
    setupWebInterface();
    while(true) {
        handleWebInterface();
        // Yield to prevent watchdog trigger
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void setup(){
    Serial.begin(115200);
    dataMutex = xSemaphoreCreateMutex();
    
    pinMode(UP_SHIFT,INPUT_PULLUP);
    pinMode(DN_SHIFT,INPUT_PULLUP);
    pinMode(SHIFTER_PIN_A,INPUT_PULLUP);
    pinMode(SHIFTER_PIN_B,INPUT_PULLUP);
    pinMode(SHIFTER_PIN_C,INPUT_PULLUP);
    pinMode(SHIFTER_PIN_D,INPUT_PULLUP);

    loadCalibration();
    preferences.begin("tcu",false);

    if(!LittleFS.begin(true)){ LittleFS.format(); LittleFS.begin(); }
    if(!loadMapsFromFS()){ Serial.println("Seeding V5 Defaults"); initDefaultMaps(); }
    loadAdaptiveData();
    loadTempScale();

    analogReadResolution(12);

    int storedGb = preferences.getInt(PREF_GB_KEY,-1);
    if(storedGb==-1){ preferences.putInt(PREF_GB_KEY,0); storedGb=0; }
    if(storedGb==0){ apply_ratios_and_set_sensor_ratio(GB_SMALL_RATIOS); }
    else { apply_ratios_and_set_sensor_ratio(GB_BIG_RATIOS); }

    if (!ledcAttach(MPC_SOL, PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC fail");
    if (!ledcAttach(SPC_SOL, PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC fail");
    if (!ledcAttach(SOL_12,  PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC fail");
    if (!ledcAttach(SOL_23,  PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC fail");
    if (!ledcAttach(SOL_34,  PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC fail");
    if (!ledcAttach(SOL_TCC, PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC fail");

    setupPCNT(PCNT_UNIT_0,N2_SEN);
    setupPCNT(PCNT_UNIT_1,N3_SEN);
    setupPCNT(PCNT_UNIT_2,OUT_SPEED);
    setupPCNT(PCNT_UNIT_3,ENGINE_RPM_SEN);

    // Start Web Server on Core 0
    // UPDATED: Stack Size increased to 16KB for JSON/Strings
    xTaskCreatePinnedToCore(
      WebServerTask, /* Function */
      "WebTask",     /* Name */
      16384,         /* Stack */
      NULL,          /* Param */
      1,             /* Priority */
      &WebTask,      /* Handle */
      0              /* Core 0 */
    );
    
    preferences.end();

    ShifterPosition bootPos = readShifterPosition();
    if(bootPos==SH_POS_P) currentGear=GEAR_PARK;
    else if(bootPos==SH_POS_N) currentGear=GEAR_NEUTRAL;
    else if(bootPos==SH_POS_R_REV) currentGear=GEAR_R2;
    else if(bootPos==SH_POS_D) currentGear=2; // Default to 2nd gear launch in D usually
    else currentGear=GEAR_NEUTRAL;
    
    // Init shared vars
    shared_currentGear = currentGear;

    Serial.println("TCU V5.0 (High Performance) Initialized");
}

// CORE 1 LOOP (High Speed Physics)
void loop(){
    unsigned long now = micros();

    // 1. HIGH SPEED SENSORS (Every 10ms)
    speedSensors(); // Checks its own timer
    
    // 2. CONTROL LOOP (Every 1ms)
    if(now - lastControlLoopMicros >= CONTROL_INTERVAL_MICROS) {
        processTimedPulse();
        garageShiftHandler();

        tpsPercentage=readTPS();
        transTemp=readTemperature();
        
        // Sync vars to shared for Web
        if(xSemaphoreTake(dataMutex, 0)==pdTRUE){
            shared_tpsPercentage = tpsPercentage;
            shared_transTemp = transTemp;
            xSemaphoreGive(dataMutex);
        }

        ShifterPosition nowPos=readShifterPosition();
        if(nowPos!=shifter_pos){
            unsigned long t=millis();
            if(t - last_shifter_change_time > shifter_debounce_ms){
                last_shifter_change_time=t;
                ShifterPosition last=shifter_pos;
                shifter_pos=nowPos;
                handleShifterChange(last,nowPos);
            }
        }

        tpsIndex=constrain(map((int)tpsPercentage,0,100,0,7),0,7);
        rpmIndex=constrain(map(engineRPM,RPM_MIN,RPM_MAX,0,7),0,7);
        
        shared_tpsIndex = tpsIndex;
        shared_rpmIndex = rpmIndex;

        GearShiftScheduler();
        performShiftStateMachine();

        controlSteadyStatePressure();
        controlTCC();
        
        lastControlLoopMicros = now;
    }
    
    // NO DELAY HERE - run at max speed
}
