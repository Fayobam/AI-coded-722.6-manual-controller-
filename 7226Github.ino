/*
Manual TCU Controller for Mercedes 722.6 (ESP32 Core 3.x)
NO-WEB, SLIP-ADAPT VERSION (COMPATIBILITY-HARDENED)

This version fixes compile order errors:
- Ensures readShifterPosition() and speedSensors() are defined BEFORE setup() and loop().
- setupPCNT is also placed before setup().
- All functions referenced by setup() and loop() now appear earlier in the file or have definitions above them.

Features:
- Slip-aware adaptive shift timing (SPC/MPC + single shift solenoid per handover).
- Binary persistence of maps & adaptation flags.
- Garage (P/N → D/R) controlled separately.
- Modern LEDC pin-based API (ledcAttach, ledcWrite) for Core 3.x.
- PCNT read abstraction (int16_t counters for deprecated driver signature).
- Batching of adaptive persistence to reduce flash wear.

Folder/file requirement:
- Sketch folder name must NOT contain extra dots. e.g., use "7226Github" folder and "7226Github.ino" file.
*/

#include <Arduino.h>
#include <driver/pcnt.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <math.h>

// ---------------- Optional debug ----------------------------------------------
#ifndef ENABLE_DEBUG
#define ENABLE_DEBUG 1
#endif
#define DBG(...) do{ if(ENABLE_DEBUG){ Serial.printf(__VA_ARGS__); } }while(0)

// ---------------- Constants ----------------------------------------------------
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define PCNT_MAX_COUNT 10000
#define PCNT_FILTER_VAL 100
#define ANALOG_MAX 4095

// Vehicle / sensor constants
#define RPM_MIN 1000
#define RPM_MAX 7000
#define PPR_ENGINE 1
#define PPR_OUT_SHAFT 24
#define PPR_TURBINE_N2 1
#define PPR_TURBINE_N3 1
#define TIRE_CIRCUMFERENCE_M 2.0
#define FINAL_DRIVE_RATIO 3.27

// Pins
const int MPC_SOL = 12;
const int SPC_SOL = 23;
const int SOL_12  = 4;
const int SOL_23  = 19;
const int SOL_34  = 18;
const int SOL_TCC = 13;

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

// ---------------- Global -------------------------------------------------------
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

unsigned long lastSpeedReadTime = 0;
const int speedReadInterval = 250;
int engineRPM = 0;
float vehicleSpeed = 0;
float inputRPM = 0;
float outputRPM = 0;
float tpsPercentage = 0;
float transTemp = 0;

int tpsMin = 0;
int tpsMax = ANALOG_MAX;

unsigned long last_shift_time = 0;
const unsigned long MIN_SHIFT_INTERVAL_MS = 500;
const unsigned long SHIFT_TIMEOUT_MS = 2500;

// Debounce
enum ShifterPosition { SH_POS_SIGNAL_NA = -1, SH_POS_P, SH_POS_R, SH_POS_N, SH_POS_D };
ShifterPosition shifter_pos = SH_POS_SIGNAL_NA;
unsigned long shifter_debounce_ms = 50;
unsigned long last_shifter_change_time = 0;

// Garage cooldown
unsigned long lastGarageEnd = 0;
const unsigned long GARAGE_COOLDOWN_MS = 300;

// Gear ratios
const float GB_SMALL_RATIOS[5] = {3.93f, 2.41f, 1.49f, 1.00f, 0.83f};
const float GB_BIG_RATIOS[5]   = {3.59f, 2.19f, 1.41f, 1.00f, 0.83f};
const char* PREF_GB_KEY = "gb_type";
struct GearRatioInfo { float ratio; float ratio_min_drift; float ratio_max_drift; };
GearRatioInfo gearboxBounds[5];
float sensor_2_1_ratio = 1.0f;

// Shift maps / adaptation
struct ShiftParameters {
    int16_t MPC_duty[8][8];
    int16_t SPC_duty[8][8];
    int16_t MPC_time[8][8];
    int16_t SPC_time[8][8];
};
ShiftParameters shiftMaps[8];

float adaptEma[8][8][8];
const float ADAPT_ALPHA = 0.20f;
const char *MAPS_BIN_FILE = "/maps.bin";
const uint8_t MAPS_VERSION = 1;
bool adaptEnabled[8] = {true,true,true,true,true,true,true,true};

struct PersistedMaps {
    uint8_t version;
    uint8_t adaptEnabled[8];
    int16_t MPC_duty[8][8][8];
    int16_t SPC_duty[8][8][8];
    int16_t MPC_time[8][8][8];
    int16_t SPC_time[8][8][8];
};

// Shift log
struct ShiftLogEntry {
    unsigned long ts;
    int fromGear;
    int toGear;
    int mapIndex;
    unsigned long durationMs;
    bool success;
    String note;
};
const int SHIFT_LOG_LEN = 256;
ShiftLogEntry shiftLog[SHIFT_LOG_LEN];
int shiftLogHead = 0;
int shiftLogCount = 0;
void pushShiftLog(const ShiftLogEntry &e){
    shiftLog[shiftLogHead] = e;
    shiftLogHead = (shiftLogHead+1) % SHIFT_LOG_LEN;
    if(shiftLogCount < SHIFT_LOG_LEN) shiftLogCount++;
}
void clearShiftLog(){ shiftLogHead=0; shiftLogCount=0; }

// Timed pulse
struct TimedPulse { int pin; int duty; unsigned long endTime; bool active; } tp = {-1,0,0,false};
void scheduleTimedPulse(int pin, int duty, unsigned long ms){
    if(pin < 0) return;
    duty = constrain(duty,0,255);
    tp.pin = pin; tp.duty = duty; tp.endTime = millis()+ms; tp.active = true;
    ledcWrite(pin,duty);
}
void processTimedPulse(){
    if(!tp.active) return;
    if((long)(millis()-tp.endTime) >= 0){
        ledcWrite(tp.pin,0);
        tp.active=false;
    }
}

// ---------------- Ratio / sensor fallback -------------------------------------
void set_sensor_ratio(float ratio){
    sensor_2_1_ratio = (ratio>0.0f)?ratio:1.0f;
    DBG("Sensor 2:1 ratio set to %.4f\n", sensor_2_1_ratio);
}
void apply_ratios_and_set_sensor_ratio(const float ratios[5]){
    for(int i=0;i<5;i++){
        float r=ratios[i];
        gearboxBounds[i].ratio=r;
        gearboxBounds[i].ratio_min_drift=r*0.90f;
        gearboxBounds[i].ratio_max_drift=r*1.10f;
    }
    if(ratios[1]!=0.0f) set_sensor_ratio(ratios[0]/ratios[1]); else set_sensor_ratio(1.0f);
    DBG("Applied ratios: %.3f %.3f %.3f %.3f %.3f\n",
        ratios[0],ratios[1],ratios[2],ratios[3],ratios[4]);
}

// ---------------- Persistence --------------------------------------------------
void persistMapsBinary(){
    PersistedMaps pm;
    pm.version = MAPS_VERSION;
    for(int i=0;i<8;i++) pm.adaptEnabled[i] = adaptEnabled[i]?1:0;
    for(int m=0;m<8;m++) for(int t=0;t<8;t++) for(int r=0;r<8;r++){
        pm.MPC_duty[m][t][r] = shiftMaps[m].MPC_duty[t][r];
        pm.SPC_duty[m][t][r] = shiftMaps[m].SPC_duty[t][r];
        pm.MPC_time[m][t][r] = constrain(shiftMaps[m].MPC_time[t][r],150,1200);
        pm.SPC_time[m][t][r] = constrain(shiftMaps[m].SPC_time[t][r],120,900);
    }
    File f = LittleFS.open(MAPS_BIN_FILE,"w");
    if(!f){ DBG("maps.bin write open failed\n"); return; }
    size_t w = f.write((const uint8_t*)&pm,sizeof(pm));
    f.close();
    DBG("maps.bin persisted (%u bytes)\n",(unsigned)w);
}
bool loadMapsBinary(){
    if(!LittleFS.exists(MAPS_BIN_FILE)) return false;
    File f=LittleFS.open(MAPS_BIN_FILE,"r");
    if(!f) return false;
    PersistedMaps pm;
    size_t rd = f.readBytes((char*)&pm,sizeof(pm));
    f.close();
    if(rd != sizeof(pm) || pm.version != MAPS_VERSION){
        DBG("maps.bin invalid/version mismatch\n");
        return false;
    }
    for(int i=0;i<8;i++) adaptEnabled[i] = (pm.adaptEnabled[i]!=0);
    for(int m=0;m<8;m++) for(int t=0;t<8;t++) for(int r=0;r<8;r++){
        shiftMaps[m].MPC_duty[t][r] = pm.MPC_duty[m][t][r];
        shiftMaps[m].SPC_duty[t][r] = pm.SPC_duty[m][t][r];
        shiftMaps[m].MPC_time[t][r] = constrain(pm.MPC_time[m][t][r],150,1200);
        shiftMaps[m].SPC_time[t][r] = constrain(pm.SPC_time[m][t][r],120,900);
    }
    DBG("maps.bin loaded\n");
    return true;
}
void saveAdaptiveData(){
    Preferences p; p.begin("adapt",false);
    p.putBytes("ema",adaptEma,sizeof(adaptEma));
    p.end();
    DBG("Adaptive EMA saved\n");
}
void loadAdaptiveData(){
    Preferences p; p.begin("adapt",true);
    if(p.isKey("ema")){ p.getBytes("ema",adaptEma,sizeof(adaptEma)); DBG("Loaded adapt EMA\n"); }
    else { memset(adaptEma,0,sizeof(adaptEma)); DBG("Init adapt EMA zeros\n"); }
    p.end();
}
void initRndashLikeDefaults(){
    for(int m=0;m<8;m++) for(int t=0;t<8;t++) for(int r=0;r<8;r++){
        shiftMaps[m].MPC_duty[t][r]=160+(int)(30*(1.0f-(float)t/7.0f));
        shiftMaps[m].SPC_duty[t][r]=190+(int)(40.0f*(1.0f-(float)t/7.0f)*((float)r/7.0f+0.5f));
        shiftMaps[m].MPC_time[t][r]=constrain(700 - r*60 - t*25,300,1200);
        shiftMaps[m].SPC_time[t][r]=constrain(460 - r*35 - t*12,180,900);
    }
    for(int m=4;m<8;m++) for(int t=0;t<8;t++) for(int r=0;r<8;r++){
        shiftMaps[m].SPC_time[t][r]=max(140,shiftMaps[m].SPC_time[t][r]-40);
        shiftMaps[m].SPC_duty[t][r]=min(240,shiftMaps[m].SPC_duty[t][r]+15);
    }
}
void initDefaultMaps(){ initRndashLikeDefaults(); persistMapsBinary(); }
void saveMapsToFS(){ persistMapsBinary(); }
bool loadMapsFromFS(){ return loadMapsBinary(); }

// Calibration
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

// ---------------- Sensors & Shifter (DEFINED BEFORE setup/loop) ---------------
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

// ---------------- Pressure & TCC ----------------------------------------------
void controlTCC(){
    if(currentGear>=3 && vehicleSpeed>60 && tpsPercentage>=20 && tpsPercentage<=80 && transTemp>=60){
        int duty=map((int)tpsPercentage,20,80,100,200);
        ledcWrite(SOL_TCC,duty);
    } else ledcWrite(SOL_TCC,0);
}
void controlSteadyStatePressure(){
    int mpc=map((int)tpsPercentage,0,100,255,0);
    if(transTemp<40) mpc=min(mpc+50,255);
    ledcWrite(MPC_SOL, mpc);
}

// ---------------- PCNT (DEFINED BEFORE setup/loop) ----------------------------
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
float calc_turbine_rpm_from_n2n3(int32_t n2,int32_t n3,float dt,float ppr_n2,float ppr_n3){
    if(dt<=0.0001f || ppr_n2<=0 || ppr_n3<=0) return 0.0f;
    const float MIN_HZ=1.0f/60.0f;
    const float DIFF=5.0f;
    const int DIS=100;
    bool n2ok=(n2>=0), n3ok=(n3>=0);
    if(!n2ok && !n3ok) return 0.0f;
    float n2hz = n2ok? (n2/dt)/ppr_n2 : 0.0f;
    float n3hz = n3ok? (n3/dt)/ppr_n3 : 0.0f;
    if(n2ok && n3ok){
        if(abs(n2-n3)>DIS) DBG("Pulse disagreement\n");
        if(fabs(n2hz-n3hz)<=DIFF) return ((n2hz+n3hz)*0.5f)*60.0f;
        if(n2hz<MIN_HZ && n3hz>=MIN_HZ) return n3hz*sensor_2_1_ratio*60.0f;
        if(n3hz<MIN_HZ && n2hz>=MIN_HZ) return n2hz*sensor_2_1_ratio*60.0f;
        return (max(n2hz,n3hz))*60.0f;
    }
    float s = n2ok?n2hz:n3hz;
    return s*60.0f*sensor_2_1_ratio;
}
void speedSensors(){
    unsigned long now=millis();
    if(now - lastSpeedReadTime < speedReadInterval) return;
    int16_t n2=0,n3=0,out=0,eng=0;
    pcnt_get_counter_value(PCNT_UNIT_0,&n2);
    pcnt_get_counter_value(PCNT_UNIT_1,&n3);
    pcnt_get_counter_value(PCNT_UNIT_2,&out);
    pcnt_get_counter_value(PCNT_UNIT_3,&eng);
    pcnt_counter_clear(PCNT_UNIT_0); pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_2); pcnt_counter_clear(PCNT_UNIT_3);
    float dt = speedReadInterval/1000.0f;
    if(dt<0.001f) dt=0.001f;
    engineRPM = (int)(((eng/dt)/PPR_ENGINE)*60.0f);
    outputRPM = ((out/dt)/PPR_OUT_SHAFT)*60.0f;
    inputRPM  = calc_turbine_rpm_from_n2n3(n2,n3,dt,PPR_TURBINE_N2,PPR_TURBINE_N3);
    float wheelRPS = (out/dt)/ (PPR_OUT_SHAFT*FINAL_DRIVE_RATIO);
    vehicleSpeed = wheelRPS*TIRE_CIRCUMFERENCE_M*3.6f;
    lastSpeedReadTime=now;
    DBG("N2=%d N3=%d In=%.0f Out=%.0f Eng=%d Spd=%.1f\n",n2,n3,inputRPM,outputRPM,engineRPM,vehicleSpeed);
}

// ---------------- Slip & Adapt thresholds -------------------------------------
const int DESIRED_SHIFT_MS[8] = {450,470,480,500,520,540,560,580};
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

// ---------------- Garage shift ------------------------------------------------
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
    int mapIdx = intoReverse?3:0;
    garageState.spc_prefill = constrain(shiftMaps[mapIdx].SPC_duty[tpsIndex][rpmIndex],60,220);
    garageState.mpc_working = constrain(shiftMaps[mapIdx].MPC_duty[tpsIndex][rpmIndex],60,220);
    ledcWrite(SPC_SOL, garageState.spc_prefill/2);
    ledcWrite(SOL_34, openingPwm);
    DBG("Garage shift start\n");
}
void garageShiftHandler(){
    if(!garageState.active) return;
    unsigned long now=millis();
    switch(garageState.phase){
        case GS_OPEN_BURST:
            if(now - garageState.phaseStart >= 50){
                ledcWrite(SOL_34, holdingPwm);
                garageState.phase=GS_HOLDING_RAMP;
                garageState.phaseStart=now;
            }
            break;
        case GS_HOLDING_RAMP:{
            garageState.elapsed = now - garageState.phaseStart;
            float rampFrac = min((float)garageState.elapsed/2500.0f, 2.0f);
            int spc=constrain((int)((garageState.spc_prefill+20)*(1.0f+rampFrac)),0,255);
            int mpc=constrain(garageState.mpc_working+(spc/2),0,255);
            ledcWrite(SPC_SOL,spc); ledcWrite(MPC_SOL,mpc);
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
                ledcWrite(SPC_SOL, garageState.spc_prefill/2);
                ledcWrite(SOL_34,0);
                currentGear=GEAR_NEUTRAL;
                pushShiftLog({millis(),GEAR_PARK,GEAR_NEUTRAL,-1,garageState.elapsed,false,"garage_failed"});
            }else{
                ledcWrite(SPC_SOL,0);
                ledcWrite(SOL_34,0);
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

// ---------------- Shift guards & scheduler ------------------------------------
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
int select_shift_style(){
    float ntps=tpsPercentage/100.0f;
    float rpmFactor=(engineRPM-800.0f)/6000.0f;
    rpmFactor=constrain(rpmFactor,0.0f,1.0f);
    float torqueEstimate=ntps*rpmFactor*100.0f;
    return (torqueEstimate>20.0f)?1:0;
}

// ---------------- Shift State Machine -----------------------------------------
enum ShiftSMState { SM_IDLE=0, SM_PREFILL, SM_OPEN_BURST, SM_HOLDING, SM_VERIFY, SM_DONE };
struct ShiftSM{
    ShiftSMState state=SM_IDLE;
    int mapIndex=-1;
    int shiftSolenoidPin=-1;
    int expectedTargetGear=0;
    unsigned long stateStart=0;
    unsigned long shiftStart=0;
    int confirmationHits=0;
    int chosenStyle=0;
    int spc_prefill=0;
    int mpc_prefill=0;
    unsigned long lastSample=0;
    // slip metrics
    float peakSlip=0.0f;
    float finalSlip=0.0f;
    unsigned long firstSlipSampleMs=0;
    unsigned long slipDecayMs=0;
    bool slipBelowStable=false;
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
    shiftSM.chosenStyle=select_shift_style();
    shiftSM.spc_prefill=constrain(shiftMaps[mapIdx].SPC_duty[tpsIndex][rpmIndex],60,220);
    shiftSM.mpc_prefill=constrain(shiftMaps[mapIdx].MPC_duty[tpsIndex][rpmIndex],60,220);
    shiftSM.lastSample=0;
    shiftSM.peakSlip=0; shiftSM.finalSlip=0;
    shiftSM.firstSlipSampleMs=0; shiftSM.slipDecayMs=0;
    shiftSM.slipBelowStable=false;
    ledcWrite(SPC_SOL,shiftSM.spc_prefill);
    ledcWrite(MPC_SOL,shiftSM.mpc_prefill);
    DBG("Shift init map=%d to G%d SPC=%d MPC=%d style=%s\n",
        mapIdx,expectedGear,shiftSM.spc_prefill,shiftSM.mpc_prefill,
        shiftSM.chosenStyle?"REL":"XOV");
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
        DBG("Batched adapt persistence\n");
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
        int spc_time=constrain(shiftMaps[shiftSM.mapIndex].SPC_time[tpsIndex][rpmIndex],120,900);
        if(shiftSM.chosenStyle==1) spc_time=max(spc_time,80);
        if(now - shiftSM.stateStart >= (unsigned long)spc_time){
            ledcWrite(shiftSM.shiftSolenoidPin, openingPwm);
            shiftSM.stateStart=now;
            shiftSM.state=SM_OPEN_BURST;
        }
        return;
    }
    if(shiftSM.state==SM_OPEN_BURST){
        unsigned long burstMs=(shiftSM.chosenStyle==1)?80:60;
        if(now - shiftSM.stateStart >= burstMs){
            ledcWrite(shiftSM.shiftSolenoidPin, holdingPwm);
            shiftSM.stateStart=now;
            shiftSM.state=SM_HOLDING;
        }
        return;
    }
    if(shiftSM.state==SM_HOLDING){
        int verify_timeout = max(shiftMaps[shiftSM.mapIndex].SPC_time[tpsIndex][rpmIndex],
                                 shiftMaps[shiftSM.mapIndex].MPC_time[tpsIndex][rpmIndex]);
        verify_timeout=constrain(verify_timeout,200,1500);
        if(now - shiftSM.lastSample >= 50){
            shiftSM.lastSample=now;
            if(shiftSM.firstSlipSampleMs==0) shiftSM.firstSlipSampleMs=now;
            float slip=computeSlip(inputRPM,outputRPM,shiftSM.expectedTargetGear);
            if(slip<5000.0f && slip>shiftSM.peakSlip) shiftSM.peakSlip=slip;
            if(!shiftSM.slipBelowStable && slip<SLIP_TARGET_MAX){
                shiftSM.slipBelowStable=true;
                shiftSM.slipDecayMs=now - shiftSM.firstSlipSampleMs;
            }
            int est=estimateGearFromRatioGuarded(inputRPM,outputRPM);
            bool ratioOk=(est==shiftSM.expectedTargetGear);
            bool slipOk=(slip<SLIP_TARGET_MAX);
            int requiredBase=(shiftSM.chosenStyle==1)?3:4;
            int requiredHits = (slip<SLIP_TARGET_MIN)? max(2,requiredBase-1): requiredBase;
            if(ratioOk && slipOk) shiftSM.confirmationHits++; else shiftSM.confirmationHits=0;
            if(shiftSM.confirmationHits >= requiredHits){
                shiftSM.state=SM_VERIFY;
                shiftSM.stateStart=now;
            }
        }
        if(now - shiftSM.shiftStart >= SHIFT_TIMEOUT_MS + verify_timeout){
            ledcWrite(shiftSM.shiftSolenoidPin,0);
            ledcWrite(SPC_SOL,0);
            ledcWrite(MPC_SOL,0);
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
                int durationError=(int)dur - targetMs;
                bool flare=(shiftSM.peakSlip>FLARE_THRESHOLD);
                bool bind=(shiftSM.finalSlip < BIND_THRESHOLD && durationError < HARD_TOLERANCE_MS);
                bool soft=(dur > targetMs + SOFT_TOLERANCE_MS && shiftSM.finalSlip < SLIP_TARGET_MAX);
                bool harsh=(durationError < HARD_TOLERANCE_MS && shiftSM.finalSlip < SLIP_TARGET_MIN);
                if(shiftSM.slipBelowStable && shiftSM.slipDecayMs<120 && !flare) harsh=true;
                if(shiftSM.slipBelowStable && shiftSM.slipDecayMs>800 && !flare) soft=true;
                float prev=adaptEma[mapIdx][tIdx][rIdx];
                if(prev<=0.0f) prev=(float)shiftMaps[mapIdx].SPC_time[tIdx][rIdx];
                float newEma=ADAPT_ALPHA*(float)dur + (1.0f-ADAPT_ALPHA)*prev;
                adaptEma[mapIdx][tIdx][rIdx]=newEma;
                int spcAdjust=0, mpcAdjust=0;
                if(flare){ spcAdjust=-40; mpcAdjust=-20; }
                else if(bind){ spcAdjust=+50; mpcAdjust=+30; }
                else if(soft){ spcAdjust=-25; }
                else if(harsh){ spcAdjust=+30; }
                else {
                    int curSPC=shiftMaps[mapIdx].SPC_time[tIdx][rIdx];
                    int diff=(int)(newEma - curSPC);
                    spcAdjust=constrain((int)(diff*0.15f),-20,20);
                }
                int curSPC=shiftMaps[mapIdx].SPC_time[tIdx][rIdx];
                int curMPC=shiftMaps[mapIdx].MPC_time[tIdx][rIdx];
                int newSPC=constrain(curSPC+spcAdjust,120,900);
                int newMPC=constrain(curMPC+mpcAdjust,150,1200);
                shiftMaps[mapIdx].SPC_time[tIdx][rIdx]=newSPC;
                shiftMaps[mapIdx].MPC_time[tIdx][rIdx]=newMPC;
                adaptSuccessCounter++;
                maybePersistAdapt();
                DBG("ADAPT map=%d t=%d r=%d dur=%lu peakSlip=%.0f finalSlip=%.0f spcΔ=%d mpcΔ=%d\n",
                    mapIdx,tIdx,rIdx,dur,shiftSM.peakSlip,shiftSM.finalSlip,spcAdjust,mpcAdjust);
            }
            ledcWrite(shiftSM.shiftSolenoidPin,0);
            ledcWrite(SPC_SOL,0);
            currentGear=shiftSM.expectedTargetGear;
            last_shift_time=now;
            shiftSM.state=SM_DONE;
        }else{
            pushShiftLog({now,currentGear,shiftSM.expectedTargetGear,shiftSM.mapIndex,0,false,"verify_fail"});
            ledcWrite(shiftSM.shiftSolenoidPin,0);
            ledcWrite(SPC_SOL,0);
            ledcWrite(MPC_SOL,0);
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

// ---------------- Shifter change handler --------------------------------------
void handleShifterChange(ShifterPosition last, ShifterPosition now){
    bool last_controllable=(last==SH_POS_D||last==SH_POS_R);
    bool now_controllable=(now==SH_POS_D||now==SH_POS_R);
    if(!last_controllable && now_controllable){
        garageShiftStart(now==SH_POS_R);
    }else if(last_controllable && !now_controllable){
        if(now==SH_POS_P){ currentGear=GEAR_PARK; targetGear=GEAR_PARK; }
        else if(now==SH_POS_N){ currentGear=GEAR_NEUTRAL; targetGear=GEAR_NEUTRAL; }
        int spc_prefill=shiftMaps[0].SPC_duty[tpsIndex][rpmIndex]/2;
        spc_prefill=constrain(spc_prefill,30,180);
        ledcWrite(SPC_SOL, spc_prefill);
        scheduleTimedPulse(SOL_34, holdingPwm, 50);
        pushShiftLog({millis(),(int)last,(int)now,-1,0,true,"move_to_PN"});
    }
}

// ---------------- Setup --------------------------------------------------------
void setup(){
    Serial.begin(115200);
    pinMode(UP_SHIFT,INPUT_PULLUP);
    pinMode(DN_SHIFT,INPUT_PULLUP);
    pinMode(SHIFTER_P_PIN,INPUT_PULLUP);
    pinMode(SHIFTER_R_PIN,INPUT_PULLUP);
    pinMode(SHIFTER_N_PIN,INPUT_PULLUP);
    pinMode(SHIFTER_D_PIN,INPUT_PULLUP);

    loadCalibration();
    preferences.begin("tcu",false);

    if(!LittleFS.begin(true)){
        Serial.println("LittleFS mount failed - formatting");
        LittleFS.format(); LittleFS.begin();
    }
    if(!loadMapsFromFS()){
        Serial.println("No maps.bin found, seeding defaults");
        initDefaultMaps();
    }
    loadAdaptiveData();

    analogReadResolution(12);

    int storedGb = preferences.getInt(PREF_GB_KEY,-1);
    if(storedGb==-1){
        preferences.putInt(PREF_GB_KEY,0);
        storedGb=0;
        Serial.println("Gearbox unset; defaulting to SMALL NAG");
    }
    if(storedGb==0){ apply_ratios_and_set_sensor_ratio(GB_SMALL_RATIOS); Serial.println("Using gearbox: SMALL"); }
    else { apply_ratios_and_set_sensor_ratio(GB_BIG_RATIOS); Serial.println("Using gearbox: BIG"); }

    // Modern LEDC: attach by pin
    if (!ledcAttach(MPC_SOL, PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC attach MPC_SOL failed");
    if (!ledcAttach(SPC_SOL, PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC attach SPC_SOL failed");
    if (!ledcAttach(SOL_12,  PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC attach SOL_12 failed");
    if (!ledcAttach(SOL_23,  PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC attach SOL_23 failed");
    if (!ledcAttach(SOL_34,  PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC attach SOL_34 failed");
    if (!ledcAttach(SOL_TCC, PWM_FREQ, PWM_RESOLUTION)) Serial.println("LEDC attach SOL_TCC failed");

    setupPCNT(PCNT_UNIT_0,N2_SEN);
    setupPCNT(PCNT_UNIT_1,N3_SEN);
    setupPCNT(PCNT_UNIT_2,OUT_SPEED);
    setupPCNT(PCNT_UNIT_3,ENGINE_RPM_SEN);

    preferences.end();

    // Initialize gear from shifter
    ShifterPosition bootPos = readShifterPosition();
    if(bootPos==SH_POS_P) currentGear=GEAR_PARK;
    else if(bootPos==SH_POS_N) currentGear=GEAR_NEUTRAL;
    else if(bootPos==SH_POS_R) currentGear=GEAR_R2;
    else if(bootPos==SH_POS_D) currentGear=2;
    else currentGear=GEAR_NEUTRAL;

    Serial.println("Manual TCU Controller 722.6 (no-web, slip-adapt, compat) initialized.");
}

// ---------------- Loop ---------------------------------------------------------
void loop(){
    speedSensors();
    processTimedPulse();
    garageShiftHandler();

    tpsPercentage=readTPS();
    transTemp=readTemperature();

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

    GearShiftScheduler();
    performShiftStateMachine();

    controlSteadyStatePressure();
    controlTCC();

    vTaskDelay(1);
}