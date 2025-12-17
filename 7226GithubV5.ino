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
#define PCNT_MAX_COUNT 20000 // Increased buffer for high RPM/PPR
#define PCNT_FILTER_VAL 20   // Reduced filter for high freq signals
#define ANALOG_MAX 4095

// Vehicle / sensor constants
#define RPM_MIN 600
#define RPM_MAX 7000

// UPDATED PPR SETTINGS
#define PPR_ENGINE 58         // 60-2 Trigger Wheel
#define PPR_OUT_SHAFT 24
#define PPR_TURBINE_N2 60      // Check if this is actually higher (e.g. 2 or 12)
#define PPR_TURBINE_N3 60

#define TIRE_CIRCUMFERENCE_M 2.0
#define FINAL_DRIVE_RATIO 3.27

const float N2_N3_GEAR_RATIO[5] = { 2.370f, 1.520f, 1.000f, 0.780f, 0.650f };
const float INPUT_RPM_EMA_ALPHA = 0.3f; 

// Pins
const int MPC_SOL = 12; // Line Pressure
const int SPC_SOL = 23; // Shift Pressure
const int SOL_12  = 4;
const int SOL_23  = 19;
const int SOL_34  = 18;
const int SOL_TCC = 13; // Torque Converter

const int N2_SEN    = 32;
const int N3_SEN    = 33;
const int OUT_SPEED = 34;
const int ENGINE_RPM_SEN = 35;

const int TEMP_SEN  = 5;
const int TPS       = 6;
const int UP_SHIFT  = 2;
const int DN_SHIFT  = 3;

const int SHIFTER_P_PIN = 14;
const int SHIFTER_R_PIN = 25;
const int SHIFTER_N_PIN = 26;
const int SHIFTER_D_PIN = 27;

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
enum ShifterPosition { SH_POS_SIGNAL_NA = -1, SH_POS_P, SH_POS_R, SH_POS_N, SH_POS_D };
enum ShiftSMState { SM_IDLE=0, SM_PREFILL, SM_OPEN_BURST, SM_HOLDING, SM_VERIFY, SM_DONE };

struct GearRatioInfo { float ratio; float ratio_min_drift; float ratio_max_drift; };

struct ShiftParameters {
    int16_t FILL_time[8][8];
    int16_t FILL_duty[8][8];
    int16_t SHIFT_duty[8][8];
    int16_t SHIFT_time[8][8];
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
const unsigned long SHIFT_TIMEOUT_MS = 2500;

void setPWM(int pin, int duty) {
    ledcWrite(pin, duty);
    // Update volatile trackers
    if(pin == MPC_SOL) live_MPC = duty;
    else if(pin == SPC_SOL) live_SPC = duty;
    else if(pin == SOL_TCC) live_TCC = duty;
}

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
const uint8_t MAPS_VERSION = 7; 
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

// ---------------- Persistence --------------------------------------------------
void persistMapsBinary(){
    PersistedData pd;
    pd.version = MAPS_VERSION;
    for(int i=0;i<8;i++) pd.adaptEnabled[i] = adaptEnabled[i]?1:0;
    for(int m=0;m<8;m++) pd.gearMaps[m] = shiftMaps[m];
    pd.genMaps = generalMaps;

    File f = LittleFS.open(MAPS_BIN_FILE,"w");
    if(!f){ DBG("maps write failed\n"); return; }
    f.write((const uint8_t*)&pd,sizeof(pd));
    f.close();
    DBG("V5 Maps persisted\n");
}
bool loadMapsBinary(){
    if(!LittleFS.exists(MAPS_BIN_FILE)) return false;
    File f=LittleFS.open(MAPS_BIN_FILE,"r");
    if(!f) return false;
    PersistedData pd;
    size_t rd = f.readBytes((char*)&pd,sizeof(pd));
    f.close();
    if(rd != sizeof(pd) || pd.version != MAPS_VERSION) return false;
    for(int i=0;i<8;i++) adaptEnabled[i] = (pd.adaptEnabled[i]!=0);
    for(int m=0;m<8;m++) shiftMaps[m] = pd.gearMaps[m];
    generalMaps = pd.genMaps;
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
                shiftMaps[m].SHIFT_time[t][r] = 800;
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
    if(transTemp >= 60.0f) return 1.0f;
    if(transTemp <= -20.0f) return 3.5f;
    if(transTemp < 5.0f) {
        float pos = (transTemp - (-20.0f)) / 25.0f;
        return 3.5f - (pos * 1.5f);
    }
    else if(transTemp < 25.0f) {
        float pos = (transTemp - 5.0f) / 20.0f;
        return 2.0f - (pos * 0.7f);
    }
    else {
        float pos = (transTemp - 25.0f) / 35.0f;
        return 1.3f - (pos * 0.3f);
    }
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
ShifterPosition readShifterPosition(){
    if (digitalRead(SHIFTER_P_PIN) == LOW) return SH_POS_P;
    if (digitalRead(SHIFTER_R_PIN) == LOW) return SH_POS_R;
    if (digitalRead(SHIFTER_N_PIN) == LOW) return SH_POS_N;
    if (digitalRead(SHIFTER_D_PIN) == LOW) return SH_POS_D;
    return SH_POS_SIGNAL_NA;
}

/**
 * Bilinear Interpolation Helper for smooth map lookups
 * 
 * Performs bilinear interpolation on an 8x8 map using TPS percentage and engine RPM.
 * This provides smoother transitions between map cells, avoiding abrupt changes
 * in pressure control that can occur with direct array indexing.
 * 
 * @param map The 8x8 int16_t map to interpolate (e.g., FILL_duty, SHIFT_duty)
 * @param tpsPercent Throttle position as a percentage (0-100)
 * @param rpm Engine RPM value
 * @return Interpolated value as a float
 */
float getInterpolatedMap(int16_t map[8][8], float tpsPercent, int rpm) {
    // Map TPS (0-100%) to continuous float index (0.0 - 7.0)
    float tpsIndexFloat = constrain((tpsPercent / 100.0f) * 7.0f, 0.0f, 7.0f);
    // Map RPM to continuous float index (0.0 - 7.0)
    float rpmIndexFloat = constrain(((float)(rpm - RPM_MIN) / (float)(RPM_MAX - RPM_MIN)) * 7.0f, 0.0f, 7.0f);
    
    // Get integer indices and fractional parts
    int tpsIdx0 = (int)tpsIndexFloat;
    int tpsIdx1 = min(tpsIdx0 + 1, 7);
    float tpsFrac = tpsIndexFloat - (float)tpsIdx0;
    
    int rpmIdx0 = (int)rpmIndexFloat;
    int rpmIdx1 = min(rpmIdx0 + 1, 7);
    float rpmFrac = rpmIndexFloat - (float)rpmIdx0;
    
    // Get the four corner values
    float v00 = (float)map[tpsIdx0][rpmIdx0];
    float v01 = (float)map[tpsIdx0][rpmIdx1];
    float v10 = (float)map[tpsIdx1][rpmIdx0];
    float v11 = (float)map[tpsIdx1][rpmIdx1];
    
    // Bilinear interpolation
    float v0 = v00 * (1.0f - rpmFrac) + v01 * rpmFrac;
    float v1 = v10 * (1.0f - rpmFrac) + v11 * rpmFrac;
    float result = v0 * (1.0f - tpsFrac) + v1 * tpsFrac;
    
    return result;
}

void controlTCC(){
    // TCC Unlock Safety Interlock - Disable TCC during ANY shift event
    if(!normalShifting && !garageShifting && currentGear>=3 && transTemp>=60){
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
        .counter_h_lim=PCNT_MAX_COUNT,
        .counter_l_lim=0,
        .unit=unit,
        .channel=PCNT_CHANNEL_0
    };
    pcnt_unit_config(&c);
    pcnt_set_filter_value(unit,PCNT_FILTER_VAL);
    pcnt_filter_enable(unit);
    pcnt_counter_clear(unit);
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
void speedSensors(){
    unsigned long now = micros();
    if(now - lastSensorReadMicros < SENSOR_INTERVAL_MICROS) return;
    
    int16_t n2=0,n3=0,out=0,eng=0;
    pcnt_get_counter_value(PCNT_UNIT_0,&n2);
    pcnt_get_counter_value(PCNT_UNIT_1,&n3);
    pcnt_get_counter_value(PCNT_UNIT_2,&out);
    pcnt_get_counter_value(PCNT_UNIT_3,&eng);
    pcnt_counter_clear(PCNT_UNIT_0); pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_2); pcnt_counter_clear(PCNT_UNIT_3);
    
    float dt = SENSOR_INTERVAL_MICROS / 1000000.0f; // Should be 0.01s
    
    // 1. Engine RPM (58 PPR)
    float rawEng = ((eng/dt)/PPR_ENGINE)*60.0f;
    // Add simple rolling average for 10ms samples
    engineRPM_averaged = (rawEng * 0.2f) + (engineRPM_averaged * 0.8f);
    engineRPM = (int)engineRPM_averaged;
    
    // 2. Output RPM (24 PPR) - OK at speed, rough at low speed
    outputRPM = ((out/dt)/PPR_OUT_SHAFT)*60.0f;

    int gearForCalc = currentGear;
    if(gearForCalc < 1 || gearForCalc > 5) gearForCalc = 1; 

    float rawInputRPM = calc_turbine_rpm_from_n2n3_v2(
        n2, n3, dt, PPR_TURBINE_N2, PPR_TURBINE_N3, gearForCalc
    );
    inputRPM = filter_input_rpm(rawInputRPM);

    float wheelRPS = (out/dt)/ (PPR_OUT_SHAFT*FINAL_DRIVE_RATIO);
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

const int DESIRED_SHIFT_MS[8] = {450,470,480,500,520,540,560,580};
const int SLIP_TARGET_MAX=220;
const int SLIP_TARGET_MIN=80;
const int FLARE_THRESHOLD=600;
const int BIND_THRESHOLD=50;
const int SOFT_TOLERANCE_MS=120;
const int HARD_TOLERANCE_MS=-120;

// Active Flare Protection Constants
const float MIN_OUTPUT_RPM_FOR_FLARE_DETECTION = 50.0f;
const float FLARE_DETECTION_RPM_THRESHOLD = 500.0f;
const int FLARE_PROTECTION_SPC_BOOST = 25;
const unsigned long FLARE_DETECTION_DELAY_MS = 100;

float computeSlip(float inRpm,float outRpm,int targetGear){
    if(targetGear<1||targetGear>5) return 9999.0f;
    if(outRpm<50.0f) return 9999.0f;
    float expected=gearboxBounds[targetGear-1].ratio;
    return fabs(inRpm - outRpm*expected);
}

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
    unsigned long lastSample=0;
    float peakSlip=0.0f;
    float finalSlip=0.0f;
    bool flareProtectionActive=false;  // Track if flare protection was triggered
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
    
    // Use bilinear interpolation for smoother pressure transitions
    shiftSM.fill_duty = (int)getInterpolatedMap(shiftMaps[mapIdx].FILL_duty, tpsPercentage, engineRPM);
    shiftSM.shift_duty = (int)getInterpolatedMap(shiftMaps[mapIdx].SHIFT_duty, tpsPercentage, engineRPM);
    
    shiftSM.lastSample=0;
    shiftSM.peakSlip=0; shiftSM.finalSlip=0;
    shiftSM.flareProtectionActive=false;

    setPWM(SPC_SOL, shiftSM.fill_duty); 
    int mpc = generalMaps.LINE_duty[tpsIndex][rpmIndex];
    setPWM(MPC_SOL, mpc);
    
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
        setPWM(SPC_SOL, shiftSM.shift_duty);
        int verify_timeout = shiftMaps[shiftSM.mapIndex].SHIFT_time[tpsIndex][rpmIndex];
        verify_timeout = constrain(verify_timeout, 300, 2000);

        // Active Flare Protection - Detect RPM flares during upshifts
        unsigned long holdingDuration = now - shiftSM.stateStart;
        if(!shiftSM.flareProtectionActive && holdingDuration > FLARE_DETECTION_DELAY_MS) {
            // Check if this is an upshift (target gear > current gear) and gear is valid
            if(shiftSM.expectedTargetGear > currentGear && 
               shiftSM.expectedTargetGear >= 1 && shiftSM.expectedTargetGear <= 5 &&
               outputRPM > MIN_OUTPUT_RPM_FOR_FLARE_DETECTION) {
                // Calculate target input RPM based on output RPM and target gear ratio
                float targetInputRPM = outputRPM * gearboxBounds[shiftSM.expectedTargetGear - 1].ratio;
                // Detect flare: input RPM exceeds target by threshold
                if(inputRPM > targetInputRPM + FLARE_DETECTION_RPM_THRESHOLD) {
                    // Boost SPC pressure to arrest the slip
                    shiftSM.shift_duty = constrain(shiftSM.shift_duty + FLARE_PROTECTION_SPC_BOOST, 0, 255);
                    setPWM(SPC_SOL, shiftSM.shift_duty);
                    shiftSM.flareProtectionActive = true;
                    DBG("Flare detected! Boosting SPC to %d\n", shiftSM.shift_duty);
                }
            }
        }

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
        }
        if(now - shiftSM.shiftStart >= SHIFT_TIMEOUT_MS + verify_timeout){
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
            
            if(adaptEnabled[shiftSM.mapIndex]){
                int mapIdx=shiftSM.mapIndex;
                int tIdx=tpsIndex, rIdx=rpmIndex;
                int targetMs=DESIRED_SHIFT_MS[mapIdx];
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
    }
}

void handleShifterChange(ShifterPosition last, ShifterPosition now){
    bool last_controllable=(last==SH_POS_D||last==SH_POS_R);
    bool now_controllable=(now==SH_POS_D||now==SH_POS_R);
    if(!last_controllable && now_controllable){
        garageShiftStart(now==SH_POS_R);
    }else if(last_controllable && !now_controllable){
        if(now==SH_POS_P){ currentGear=GEAR_PARK; targetGear=GEAR_PARK; }
        else if(now==SH_POS_N){ currentGear=GEAR_NEUTRAL; targetGear=GEAR_NEUTRAL; }
        
        if(xSemaphoreTake(dataMutex, 10)==pdTRUE){ shared_currentGear=currentGear; xSemaphoreGive(dataMutex); }
        
        setPWM(SPC_SOL, 80);
        scheduleTimedPulse(SOL_34, holdingPwm, 50);
        pushShiftLog({millis(),(int)last,(int)now,-1,0,true,"move_to_PN"});
    }
}

// V5: Telemetry reads from Shared Volatile vars (Safe for Core 0)
String getExtendedTelemetryJSON() {
    String j = "{";
    // Read protected data
    if(xSemaphoreTake(dataMutex, 50) == pdTRUE) {
        j += "\"rpm\":" + String(shared_engineRPM) + ",";
        j += "\"spd\":" + String(shared_vehicleSpeed, 1) + ",";
        j += "\"tps\":" + String(shared_tpsPercentage, 0) + ",";
        j += "\"tmp\":" + String(shared_transTemp, 0) + ",";
        j += "\"gr\":" + String(shared_currentGear) + ",";
        j += "\"inRpm\":" + String(shared_inputRPM, 0) + ",";
        j += "\"outRpm\":" + String(shared_outputRPM, 0) + ",";
        xSemaphoreGive(dataMutex);
    } else {
        // Fallback if mutex busy
        j += "\"rpm\":0,\"spd\":0,\"gr\":99,";
    }
    
    String sState = "IDLE";
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
    j += "\"sState\":\"" + sState + "\",";
    j += "\"lTime\":" + String(lastShiftDuration) + ",";
    j += "\"mpc\":" + String(live_MPC) + ",";
    j += "\"spc\":" + String(live_SPC) + ",";
    j += "\"tcc\":" + String(live_TCC) + ",";
    j += "\"ti\":" + String(tpsIndex) + ",";
    j += "\"ri\":" + String(rpmIndex) + ",";
    j += "\"uptime\":" + String(millis());
    j += "}";
    return j;
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
    pinMode(SHIFTER_P_PIN,INPUT_PULLUP);
    pinMode(SHIFTER_R_PIN,INPUT_PULLUP);
    pinMode(SHIFTER_N_PIN,INPUT_PULLUP);
    pinMode(SHIFTER_D_PIN,INPUT_PULLUP);

    loadCalibration();
    preferences.begin("tcu",false);

    if(!LittleFS.begin(true)){ LittleFS.format(); LittleFS.begin(); }
    if(!loadMapsFromFS()){ Serial.println("Seeding V5 Defaults"); initDefaultMaps(); }
    loadAdaptiveData();

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
    xTaskCreatePinnedToCore(
      WebServerTask, /* Function */
      "WebTask",     /* Name */
      10000,         /* Stack */
      NULL,          /* Param */
      1,             /* Priority */
      &WebTask,      /* Handle */
      0              /* Core 0 */
    );
    
    preferences.end();

    ShifterPosition bootPos = readShifterPosition();
    if(bootPos==SH_POS_P) currentGear=GEAR_PARK;
    else if(bootPos==SH_POS_N) currentGear=GEAR_NEUTRAL;
    else if(bootPos==SH_POS_R) currentGear=GEAR_R2;
    else if(bootPos==SH_POS_D) currentGear=2;
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
