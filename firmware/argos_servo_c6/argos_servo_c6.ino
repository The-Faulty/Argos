// Argos servo firmware — ESP32-C6 dev kit build.
//
// Same logic as ../argos_servo/argos_servo.ino, with hardware constants
// retargeted from the classic ESP32 (HUZZAH32 Feather) to the ESP32-C6
// dev kit. Source of truth for everything else (gait math, IK, animation
// playback, serial protocol) is identical — keep the two files in sync
// when changing logic.
//
// Wiring:
//   GPIO 6  → PCA9685 SDA + LSM9DS0 SDA  (shared I2C bus)
//   GPIO 7  → PCA9685 SCL + LSM9DS0 SCL
//   3V3     → PCA9685 VCC (logic side) + LSM9DS0 VCC
//   GND     → PCA9685 GND, LSM9DS0 GND (and battery/PSU GND, common ground required)
//   5–7.4V  → PCA9685 V+ (servo power, NOT from the ESP32's 3V3 rail)
//   GPIO 4  → optional analog gas/battery sensor (ADC1_CH4); leave floating if unused
//   LSM9DS0 SDO_XM, SDO_G → tied HIGH (I2C addresses 0x1D + 0x6B). Optional;
//   firmware logs a one-line warning at boot if the chip is absent and
//   carries on with imu.present = false in telemetry.
//
// Board: "ESP32C6 Dev Module" in Arduino IDE.
// USB CDC On Boot: Enabled (so Serial = USB-CDC and the dashboard can connect over USB).
// Upload mode: USB-OTG / native USB (default for the dev kit).

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NUM_LEGS 4
#define INVALID_CHANNEL 255
#define SERIAL_BAUD 921600
#define SERIAL_BUFFER_SIZE 320
#define MAX_NAME_LEN 32
#define MAX_ERROR_LEN 96
#define MAX_ANIM_KEYFRAMES 32
// 200 ms (5 Hz) state telemetry. The dashboard smooths this fine; halving
// the rate cuts TX pressure and gives the RX driver more headroom between
// telemetry bursts. Pair this with the single-buffer sendStateMessage() —
// the chained Serial.print() implementation at 100 ms was overflowing the
// RX FIFO mid-frame and dropping inbound set_leg_servo_angles commands.
#define TELEMETRY_INTERVAL_MS 200
#define BUILTIN_STATUS_INTERVAL_MS 400
#define ANIMATION_STATUS_INTERVAL_MS 250
// IMU broadcast cadence. The chip is polled at IMU_POLL_INTERVAL_MS (50 Hz)
// regardless; this is just how often we emit a separate {"type":"imu",...}
// JSON line to the host. 20 ms gives the dashboard's gait stabilizer the
// same rate it had with the RealSense IMU but from the body-frame LSM9DS0.
#define IMU_TELEMETRY_INTERVAL_MS 20

static const float SERVO_SUPPLY_VOLTS = 7.4f;
static const float SERVO_SPEED_SAFETY_FACTOR = 0.80f;
static const float MOTION_TIME_SCALE = 1.15f;
static const float DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC = 180.0f;
static const float HIP_CENTER_DEG = 90.0f;
static const float THIGH_CENTER_DEG = 90.0f;
static const float CALF_CENTER_DEG = 90.0f;
static const float LEFT_HIP_SIGN = 1.0f;
static const float LEFT_THIGH_SIGN = 1.0f;
static const float LEFT_CALF_SIGN = 1.0f;
static const float RIGHT_HIP_SIGN = -1.0f;
static const float RIGHT_THIGH_SIGN = -1.0f;
static const float RIGHT_CALF_SIGN = -1.0f;
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float DEFAULT_THIGH_MIN_DEG = -90.0f;
static const float DEFAULT_THIGH_MAX_DEG = 0.0f;
static const float DEFAULT_CALF_MIN_DEG = -90.0f;
static const float DEFAULT_CALF_MAX_DEG = 90.0f;
static const int COUPLED_LIMIT_SAMPLE_COUNT = 7;
static const float COUPLED_THIGH_SAMPLES_DEG[COUPLED_LIMIT_SAMPLE_COUNT] = {-90.0f, -50.0f, -38.0f, -35.0f, -25.0f, -13.0f, 0.0f};
static const float COUPLED_CALF_MIN_SAMPLES_DEG[COUPLED_LIMIT_SAMPLE_COUNT] = {-90.0f, -90.0f, -90.0f, -90.0f, -70.0f, -50.0f, -25.0f};
static const float COUPLED_CALF_MAX_SAMPLES_DEG[COUPLED_LIMIT_SAMPLE_COUNT] = {-90.0f, 5.0f, 30.0f, 37.0f, 60.0f, 90.0f, 90.0f};
static const uint8_t PCA9685_ADDRESS = 0x40;
static const float DEFAULT_PCA9685_FREQUENCY_HZ = 150.0f;
static const float MIN_PCA9685_FREQUENCY_HZ = 40.0f;
static const float MAX_PCA9685_FREQUENCY_HZ = 200.0f;
// ESP32-C6-DevKitC-1 (and most ESP32-C6 dev kits) — the Arduino-ESP32
// core's default I2C pins are GPIO 6 (SDA) and GPIO 7 (SCL). Unlike the
// classic ESP32 these are NOT wired to internal SPI flash on the C6, so
// they're safe to use. Avoid GPIO 12/13 (USB D-/D+), GPIO 8 (onboard LED
// on most C6 dev kits), and GPIO 9 (BOOT strapping pin).
static const uint8_t I2C_SDA_PIN = 6;
static const uint8_t I2C_SCL_PIN = 7;
// ESP32-C6 has no input-only pins; ADC1 channels live on GPIO 0..6.
// GPIO 6/7 are I2C here, so the MQ-131 ozone sensor's A0 line goes on
// GPIO 4 (ADC1_CH4) via a 10k/20k voltage divider (A0 swings 0–5V; ESP
// ADC max is 3.3V — direct connection will damage the chip).
// Telemetry reports ozone as `o3Ppb` (mq131GetO3Ppb()); R0 calibration
// constants are in the MQ-131 block below. If no analog sensor is
// wired, leave the pin floating — readings degenerate to 0 ppb and the
// dashboard treats that as "no event."
static const int GAS_ADC_PIN = 4;
static const float SERVO_MIN_PULSE_US = 500.0f;
static const float SERVO_MAX_PULSE_US = 2500.0f;
static const float NEUTRAL_THIGH_THETA = 0.0f;
static const float NEUTRAL_SERVO_THETA = -2.768896484375f;
static const float SERVO_SEARCH_STEP_DEG = 5.0f;
static const float SERVO_REFINE_STEPS_DEG[] = {2.0f, 1.0f, 0.5f, 0.25f};

// ─── NeoPixel LED strip ───────────────────────────────────────────────────
// 46 LEDs daisy-chained on GPIO 5.
//   LEDs  0–39  → underbelly strip
//   LEDs 40–42  → left  signal (3 LEDs)
//   LEDs 43–45  → right signal (3 LEDs)
//
// Configurable signal flash colors — change these to taste:
#define SIGNAL_LEFT_R 255
#define SIGNAL_LEFT_G 165
#define SIGNAL_LEFT_B 0 // amber/orange by default

#define SIGNAL_RIGHT_R 255
#define SIGNAL_RIGHT_G 165
#define SIGNAL_RIGHT_B 0 // amber/orange by default

static const uint8_t NEOPIXEL_PIN = 5;
static const uint16_t NEOPIXEL_COUNT = 46;
static const uint8_t NEOPIXEL_BRIGHTNESS = 80;

// Underbelly LED index range (inclusive)
static const uint16_t BELLY_FIRST = 0;
static const uint16_t BELLY_LAST = 39;

// Signal LED index ranges (3 LEDs each, inclusive)
static const uint16_t LEFT_SIG_FIRST = 40;
static const uint16_t LEFT_SIG_LAST = 42;
static const uint16_t RIGHT_SIG_FIRST = 43;
static const uint16_t RIGHT_SIG_LAST = 45;

// Flash period for turn signals and reverse (ms per half-cycle)
static const uint32_t SIGNAL_FLASH_HALF_MS = 250; // 2 Hz

typedef struct
{
    float x;
    float y;
} Point2f;

typedef struct
{
    Point2f hip;
    Point2f servoPivot;
    Point2f knee;
    Point2f servoHornEnd;
    Point2f bellArmA;
    Point2f bellArmB;
    Point2f calfAttach;
    Point2f foot;
    float thetaThigh;
    float thetaServo;
    float thetaCalf;
    bool valid;
} LegGeometry;

typedef struct
{
    float thetaThigh;
    float thetaServo;
    float thighServoDeg;
    float calfServoDeg;
    float footError;
    Point2f resolvedFoot;
    bool hasSolution;
    bool success;
} LegIKResult;

typedef struct
{
    float modelThighDeg;
    float modelCalfDeg;
    float thetaThigh;
    float thetaServo;
    float thetaCalf;
    LegGeometry geometry;
    float error;
    float servoPenalty;
    float continuityPenalty;
    bool valid;
} FootServoCandidate;

typedef struct
{
    float modelCalfDeg;
    float thetaServo;
    float thetaCalf;
    LegGeometry geometry;
    float error;
    float servoPenalty;
    float continuityPenalty;
    bool valid;
} JointServoCandidate;

typedef struct
{
    uint32_t count;
    uint32_t successCount;
    uint32_t lastUs;
    uint32_t maxUs;
    double totalUs;
} SolverTimingStats;

typedef struct
{
    float thighDeg;
    float calfDeg;
} JointAnglesDeg;

typedef struct
{
    float min;
    float max;
} JointLimitRangeDeg;

typedef struct
{
    JointLimitRangeDeg thighDeg;
    JointLimitRangeDeg calfDeg;
} JointLimitsDeg;

typedef struct
{
    float hip;
    float thigh;
    float calf;
} ServoAnglesDeg;

typedef struct
{
    const char *id;
    uint8_t hipChannel;
    uint8_t thighChannel;
    uint8_t calfChannel;
    float hipSign;
    float thighSign;
    float calfSign;
} LegHardwareConfig;

typedef struct
{
    uint8_t channel;
    bool attached;
    float minDeg;
    float maxDeg;
    float centerDeg;
    float sign;
    float hardwareMaxSpeedDegPerSec;
    float speedLimitDegPerSec;
    float startDeg;
    float targetDeg;
    float estimatedDeg;
    uint32_t moveStartUs;
    uint32_t moveDurationUs;
    bool moving;
} SmoothServo;

typedef struct
{
    SmoothServo hip;
    SmoothServo thigh;
    SmoothServo calf;
    Point2f desiredFoot;
    JointAnglesDeg desiredJointAngles;
    ServoAnglesDeg desiredServoAngles;
    Point2f currentFoot;
    JointAnglesDeg currentJointAngles;
    ServoAnglesDeg currentServoAngles;
    uint8_t hipChannel;
    uint8_t thighChannel;
    uint8_t calfChannel;
    JointLimitsDeg jointLimits;
    ServoAnglesDeg servoSpeedLimitDegPerSec;
    float lastThetaThigh;
    float lastThetaServo;
    char lastError[MAX_ERROR_LEN];
} RobotLegState;

typedef struct
{
    float time;
    float x;
    float y;
} AnimationFrame;

typedef struct
{
    AnimationFrame frames[MAX_ANIM_KEYFRAMES];
    int count;
} LegTrack;

typedef struct
{
    char name[MAX_NAME_LEN];
    float duration;
    LegTrack tracks[NUM_LEGS];
    bool ready;
    bool playing;
    bool paused;
    uint32_t startMs;
    float pausedTimeSec;
} AnimationClip;

typedef enum
{
    MODE_IDLE,
    MODE_DIRECT_FOOT_XY,
    MODE_DIRECT_JOINT_ANGLES,
    MODE_DIRECT_SERVO_ANGLES,
    MODE_BUILTIN_WALK,
    MODE_BUILTIN_CROUCH,
    MODE_ANIMATION_PLAYBACK
} RobotMode;

typedef enum
{
    WALK_STOP,
    WALK_FORWARD,
    WALK_BACKWARD,
    WALK_TURN_LEFT,
    WALK_TURN_RIGHT
} WalkIntent;

static const float L_THIGH = 127.0f;
static const float L_CALF = 127.0f;
static const float HORN = 20.0f;
static const float LINK_SHORT = 30.0f;
static const float BELL = 40.0f;
static const float LINK_LONG = 150.0f;
static const float CALF_ATTACH_OFFSET = 30.0f;
static const Point2f HIP_PIVOT = {0.0f, 0.0f};
static const Point2f SERVO_PIVOT = {-20.0f, -22.0f};
static const Point2f FOOT_ORIGIN_OFFSET = {40.0f, -140.0f};

static const LegHardwareConfig LEG_CONFIGS[NUM_LEGS] = {
    {"front_left", 8, 0, 1, LEFT_HIP_SIGN, LEFT_THIGH_SIGN, LEFT_CALF_SIGN},
    {"front_right", 9, 2, 3, RIGHT_HIP_SIGN, RIGHT_THIGH_SIGN, RIGHT_CALF_SIGN},
    {"rear_left", 10, 4, 5, LEFT_HIP_SIGN, LEFT_THIGH_SIGN, LEFT_CALF_SIGN},
    {"rear_right", 11, 6, 7, RIGHT_HIP_SIGN, RIGHT_THIGH_SIGN, RIGHT_CALF_SIGN}};

static Adafruit_PWMServoDriver g_pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
static Adafruit_NeoPixel g_ledStrip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
static RobotLegState g_legs[NUM_LEGS];
static AnimationClip g_animation;
static RobotMode g_mode = MODE_IDLE;
static WalkIntent g_walkIntent = WALK_STOP;
static char g_builtinName[MAX_NAME_LEN] = "";
static char g_activeAnimationName[MAX_NAME_LEN] = "";
static float g_thighNeutralAngle = NEUTRAL_THIGH_THETA;
static float g_servoNeutralAngle = NEUTRAL_SERVO_THETA;
static bool g_neutralInitialized = true;
static char g_serialBuffer[SERIAL_BUFFER_SIZE];
static int g_serialIndex = 0;
static uint32_t g_modeStartMs = 0;
static uint32_t g_lastTelemetryMs = 0;
static uint32_t g_lastBuiltinStatusMs = 0;
static uint32_t g_lastAnimationStatusMs = 0;
static bool g_servosReleased = false;
static float g_pwmFrequencyHz = DEFAULT_PCA9685_FREQUENCY_HZ;
static SolverTimingStats g_footSolveStats;
static SolverTimingStats g_jointSolveStats;

static bool i2cDevicePresent(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

static float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

static float clampAngle(float angle)
{
    const float twoPi = 2.0f * (float)M_PI;
    float value = fmodf(angle, twoPi);
    if (value <= -(float)M_PI)
        value += twoPi;
    if (value > (float)M_PI)
        value -= twoPi;
    return value;
}

static float radToDeg(float radians)
{
    return radians * 180.0f / (float)M_PI;
}

static float degToRad(float degrees)
{
    return degrees * (float)M_PI / 180.0f;
}

// ─── MQ-131 ozone sensor (analog) ────────────────────────────────────────
// Converts the GAS_ADC_PIN reading to ozone PPB using the datasheet
// power-law curve. The wiring assumes a 10k/20k voltage divider on A0
// (the module outputs 0–5V; ESP32 ADC tops out at 3.3V), so the analog
// reading is back-scaled by MQ131_DIVIDER_RATIO before computing Rs.
//
// Calibration: R0 must be measured in clean air. Power the sensor for
// ~24 h to stabilize, then set MQ131_R0_KOHMS so that
// `mq131GetO3Ppb()` reads ≈10 ppb in clean indoor air. The defaults
// below are datasheet-typical and will be order-of-magnitude correct
// out of the box but not lab-accurate. Tune in your environment.
//
// Curve: Rs/R0 = 1.0 → ~10 ppb, 0.4 → ~100 ppb, 0.3 → ~300 ppb
// (low-concentration variant). Solved as ppb = A · (Rs/R0)^B with
// A = 10, B ≈ -2.51 from a two-point log-log fit.
static const float MQ131_VCC_V             = 5.0f;
static const float MQ131_DIVIDER_RATIO     = 5.0f / 3.3f; // A0 → ESP-pin scale-back
static const float MQ131_RL_KOHMS          = 10.0f;       // module load resistor
static const float MQ131_R0_KOHMS          = 240.0f;      // CALIBRATE in clean air
static const float MQ131_CURVE_A           = 10.0f;       // ppb at Rs/R0 = 1
static const float MQ131_CURVE_B           = -2.51f;      // log-log slope
static const float MQ131_ADC_VREF_V        = 3.3f;        // ESP32-C6 default
static const int   MQ131_ADC_MAX_COUNTS    = 4095;        // 12-bit ADC
static const float O3_MG_M3_PER_PPM        = 48.0f / 24.45f;

static void mq131ReadO3(int *rawOut, float *ppmOut, float *ppbOut, float *mgM3Out, float *ugM3Out) {
    *rawOut = 0;
    *ppmOut = 0.0f;
    *ppbOut = 0.0f;
    *mgM3Out = 0.0f;
    *ugM3Out = 0.0f;
    int raw = analogRead(GAS_ADC_PIN);
    *rawOut = raw;
    if (raw <= 0) return;
    float vPin = ((float)raw / (float)MQ131_ADC_MAX_COUNTS) * MQ131_ADC_VREF_V;
    float vA0  = vPin * MQ131_DIVIDER_RATIO;
    if (vA0 < 0.01f || vA0 >= MQ131_VCC_V) return;
    float rsKOhms = MQ131_RL_KOHMS * (MQ131_VCC_V - vA0) / vA0;
    float ratio   = rsKOhms / MQ131_R0_KOHMS;
    if (ratio <= 0.0f) return;
    float ppb = MQ131_CURVE_A * powf(ratio, MQ131_CURVE_B);
    if (!isfinite(ppb) || ppb < 0.0f) return;
    *ppbOut = ppb;
    *ppmOut = ppb / 1000.0f;
    *mgM3Out = *ppmOut * O3_MG_M3_PER_PPM;
    *ugM3Out = *mgM3Out * 1000.0f;
}

// ─── LSM9DS0 IMU (LSM303D accel+mag + L3GD20H gyro) ──────────────────────
// Shares the I2C bus with the PCA9685 — addresses 0x1D (accel/mag) and
// 0x6B (gyro), both with SDO pulled high. Init failure is non-fatal: the
// imu.present field in telemetry goes false and the dashboard's gait
// stabilizer just won't apply tilt correction. Polled at ~50 Hz from
// loop() (see g_lastImuPollMs in maybeSendPeriodicEvents). Roll/pitch
// come from accel directly; yaw is gyro-z integrated with a long-stall
// dt clamp, so it drifts (no magnetometer fusion yet — swap in a
// Madgwick/Mahony filter if drift becomes a problem).
// Default to SDO-HIGH addresses, but imuInit() probes both pairs at boot
// and updates these to whichever pair the chip actually responds at.
// Mutable on purpose — the hardware's SDO state isn't always known.
static uint8_t LSM303D_ADDR = 0x1D;     // SDO_XM HIGH; falls back to 0x1E if LOW
static uint8_t L3GD20H_ADDR = 0x6B;     // SDO_G  HIGH; falls back to 0x6A if LOW
static const uint8_t LSM303D_WHO_AM_I_OK = 0x49;
static const uint8_t L3GD20H_WHO_AM_I_OK = 0xD4;
// Sensitivities at the configured full-scale ranges (datasheet typical).
static const float LSM303D_ACCEL_LSB_TO_G = 0.000061f;  // ±2g
static const float LSM303D_MAG_LSB_TO_GAUSS = 0.00008f; // ±2 gauss high-res
static const float L3GD20H_GYRO_LSB_TO_DPS = 0.00875f;  // ±245 dps
static const float GRAVITY_MS2_F = 9.80665f;
static const uint32_t IMU_POLL_INTERVAL_MS = 20; // 50 Hz

static bool     g_imuPresent           = false;
static float    g_imuAccelMs2[3]       = {0.0f, 0.0f, 0.0f};   // ax, ay, az (m/s²)
static float    g_imuGyroRads[3]       = {0.0f, 0.0f, 0.0f};   // gx, gy, gz (rad/s)
static float    g_imuMagGauss[3]       = {0.0f, 0.0f, 0.0f};   // mx, my, mz (gauss)
static float    g_imuRollRad           = 0.0f;
static float    g_imuPitchRad          = 0.0f;
static float    g_imuYawRad            = 0.0f;
static uint32_t g_imuLastSampleMs      = 0;
static uint32_t g_lastImuPollMs        = 0;
static uint32_t g_lastImuTelemetryMs   = 0;

static const float MADGWICK_BETA = 0.1f;

static float g_q0 = 1.0f;
static float g_q1 = 0.0f;
static float g_q2 = 0.0f;
static float g_q3 = 0.0f;

static float invSqrt(float x)
{
    // Fast inverse square root (Quake III / Madgwick reference implementation).
    // Falls back to 1/sqrtf on platforms where the bit-cast trick is unsafe.
    if (x <= 0.0f)
        return 0.0f;
    union
    {
        float f;
        uint32_t i;
    } conv = {x};
    conv.i = 0x5F3759DFu - (conv.i >> 1u);
    float y = conv.f;
    y *= 1.5f - (0.5f * x * y * y); // one Newton-Raphson step
    return y;
}

static void madgwickUpdate(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float mx, float my, float mz,
                           float dt)
{
    if (dt <= 0.0f)
        return;

    float q0 = g_q0, q1 = g_q1, q2 = g_q2, q3 = g_q3;

    // ── Normalise accelerometer ──────────────────────────────────────────
    float aNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (aNorm == 0.0f)
        return; // degenerate — skip this sample
    ax *= aNorm;
    ay *= aNorm;
    az *= aNorm;

    // ── Normalise magnetometer ───────────────────────────────────────────
    float mNorm = invSqrt(mx * mx + my * my + mz * mz);
    if (mNorm == 0.0f)
    {
        // Magnetometer reading is zero (sensor missing or saturated).
        // Fall back to 6-DOF (accel + gyro only): zero out mag gradient term
        // by passing a unit vector aligned with the reference field direction
        // so the mag correction contributes nothing new.
        mx = 0.0f;
        my = 0.0f;
        mz = 1.0f;
        mNorm = 1.0f;
    }
    else
    {
        mx *= mNorm;
        my *= mNorm;
        mz *= mNorm;
    }

    // ── Reference direction of Earth's magnetic field ────────────────────
    // Rotate normalised mag measurement into Earth frame using current q,
    // then project onto the horizontal plane (bx, bz only) to decouple
    // heading from dip angle.
    float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
    float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));
    float hz = 2.0f * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5f - q1 * q1 - q2 * q2));

    float bx = sqrtf(hx * hx + hy * hy); // horizontal magnitude
    float bz = hz;                       // vertical component (dip)

    // ── Gradient descent objective function ─────────────────────────────
    // f1..f3: accel objective  (align gravity direction with sensor z-down)
    // f4..f6: mag objective    (align mag reference with sensor reading)

    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _2bx = 2.0f * bx, _2bz = 2.0f * bz;
    float _4bx = 4.0f * bx, _4bz = 4.0f * bz;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q2 = q1 * q2, q1q3 = q1 * q3, q2q3 = q2 * q3;

    // Gravity objective (expected: [0,0,1] in body frame)
    float f1 = 2.0f * (q1q3 - q0q2) - ax;
    float f2 = 2.0f * (q0q1 + q2q3) - ay;
    float f3 = 2.0f * (0.5f - q1q1 - q2q2) - az;

    // Magnetic objective (expected: [bx,0,bz] in body frame)
    float f4 = _2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx;
    float f5 = _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my;
    float f6 = _2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz;

    // Jacobian transposed times objective — the gradient direction
    float J11 = -_2q2, J12 = _2q1, J13 = 0.0f;
    float J14 = -_2bz * q2, J15 = -_2bx * q3 + _2bz * q1;
    float J16 = _2bx * q2;

    float J21 = _2q3, J22 = _2q0, J23 = -4.0f * q1;
    float J24 = _2bx * q2 + _2bz * q0;
    float J25 = _2bx * q3 + _2bz * q1; // wait, recompute carefully below
    float J26 = _2bx * q0 - _4bz * q2;

    // (Re-derive cleanly to avoid copy-paste slips — Madgwick eq. 25)
    float s0 = -_2q2 * f1 + _2q1 * f2 + (-_2bz * q2) * f4 + (-_2bx * q3 + _2bz * q1) * f5 + (_2bx * q2) * f6;

    float s1 = _2q3 * f1 + _2q0 * f2 - 4.0f * q1 * f3 + (_2bz * q3) * f4 + (_2bx * q2 + _2bz * q0) * f5 + (_2bx * q3 - _4bx * q1) * f6; // note: _4bx not _4bz for last
    // Corrected s1 using the reference paper formula:
    s1 = _2q3 * f1 + _2q0 * f2 - 4.0f * q1 * f3 + (_2bz * q3) * f4 + (_2bx * q2 + _2bz * q0) * f5 + (_2bx * q3 - _4bx * q1) * f6;

    float s2 = -_2q0 * f1 + _2q3 * f2 - 4.0f * q2 * f3 + (-_4bx * q2 - _2bz * q0) * f4 + (_2bx * q1 + _2bz * q3) * f5 + (_2bx * q0 - _4bz * q2) * f6;

    float s3 = _2q1 * f1 + _2q2 * f2 + (-_4bx * q3 + _2bz * q1) * f4 + (-_2bx * q0 + _2bz * q2) * f5 + (_2bx * q1) * f6;

    // Normalise gradient
    float sNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sNorm > 0.0f)
    {
        s0 *= sNorm;
        s1 *= sNorm;
        s2 *= sNorm;
        s3 *= sNorm;
    }

    // ── Quaternion rate from gyro ────────────────────────────────────────
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - MADGWICK_BETA * s0;
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - MADGWICK_BETA * s1;
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - MADGWICK_BETA * s2;
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - MADGWICK_BETA * s3;

    // ── Integrate ────────────────────────────────────────────────────────
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // ── Re-normalise quaternion ──────────────────────────────────────────
    float qNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    g_q0 = q0 * qNorm;
    g_q1 = q1 * qNorm;
    g_q2 = q2 * qNorm;
    g_q3 = q3 * qNorm;

    // Suppress unused-variable warnings from the intermediate Jacobian terms
    // that were superseded by the direct s0..s3 calculation above.
    (void)J11;
    (void)J12;
    (void)J13;
    (void)J14;
    (void)J15;
    (void)J16;
    (void)J21;
    (void)J22;
    (void)J23;
    (void)J24;
    (void)J25;
    (void)J26;
}

static void quaternionToEuler(void)
{
    float q0 = g_q0, q1 = g_q1, q2 = g_q2, q3 = g_q3;

    // Roll  (rotation about X)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    g_imuRollRad = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (rotation about Y) — clamp to avoid NaN at ±90°
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = clampf(sinp, -1.0f, 1.0f);
    g_imuPitchRad = asinf(sinp);

    // Yaw   (rotation about Z)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    g_imuYawRad = atan2f(siny_cosp, cosy_cosp);
}

static bool imuI2cWrite(uint8_t addr, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool imuI2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n)
{
    Wire.beginTransmission(addr);
    // MSB of subaddress = auto-increment for multi-byte reads on both chips.
    Wire.write((uint8_t)(reg | 0x80));
    if (Wire.endTransmission(false) != 0)
        return false;
    size_t got = Wire.requestFrom((int)addr, (int)n);
    if (got != n)
        return false;
    for (size_t i = 0; i < n; ++i)
        buf[i] = Wire.read();
    return true;
}

static bool imuReadByte(uint8_t addr, uint8_t reg, uint8_t *out)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((int)addr, 1) != 1)
        return false;
    *out = Wire.read();
    return true;
}

// Scan the I2C bus and print every address that ACKs. Diagnostic-only —
// pulled in once at boot so the host log shows what's actually on the
// bus when imuInit fails. Skips the reserved 0x00..0x07 and 0x78..0x7F
// ranges per the I2C spec.
static void scanI2cBus(void)
{
    Serial.print("[i2c-scan] devices on bus:");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            Serial.print(" 0x");
            if (addr < 0x10) Serial.print("0");
            Serial.print(addr, HEX);
            ++found;
        }
    }
    if (found == 0) Serial.print(" (none)");
    Serial.println();
}

static bool imuInit(void)
{
    uint8_t who = 0;

    // ── LSM303D (accel + magnetometer) ──
    // Try SDO HIGH (0x1D) first, fall back to SDO LOW (0x1E). The two
    // hardware variants are identical except for which address responds,
    // so we just rebind the global and proceed.
    if (!imuReadByte(0x1D, 0x0F, &who) || who != LSM303D_WHO_AM_I_OK)
    {
        if (!imuReadByte(0x1E, 0x0F, &who) || who != LSM303D_WHO_AM_I_OK)
            return false;
        LSM303D_ADDR = 0x1E;
    }
    else
    {
        LSM303D_ADDR = 0x1D;
    }
    Serial.print("[imu] LSM303D found at 0x");
    Serial.println(LSM303D_ADDR, HEX);
    // CTRL1 = 0x57: ODR=50 Hz, BDU=0, all axes enabled (0101 0111)
    if (!imuI2cWrite(LSM303D_ADDR, 0x20, 0x57))
        return false;
    // CTRL2 = 0x00: ±2g, no anti-alias filter override
    if (!imuI2cWrite(LSM303D_ADDR, 0x21, 0x00))
        return false;
    // CTRL5 = 0x70: temp off, mag high-res, mag ODR=50 Hz, no INT (0111 0000)
    if (!imuI2cWrite(LSM303D_ADDR, 0x24, 0x70))
        return false;
    // CTRL6 = 0x00: mag ±2 gauss
    if (!imuI2cWrite(LSM303D_ADDR, 0x25, 0x00))
        return false;
    // CTRL7 = 0x00: mag continuous-conversion mode
    if (!imuI2cWrite(LSM303D_ADDR, 0x26, 0x00))
        return false;

    // ── L3GD20H (gyroscope) ──
    // Same SDO-HIGH-vs-LOW probe as the LSM303D above.
    if (!imuReadByte(0x6B, 0x0F, &who) || who != L3GD20H_WHO_AM_I_OK)
    {
        if (!imuReadByte(0x6A, 0x0F, &who) || who != L3GD20H_WHO_AM_I_OK)
            return false;
        L3GD20H_ADDR = 0x6A;
    }
    else
    {
        L3GD20H_ADDR = 0x6B;
    }
    Serial.print("[imu] L3GD20H found at 0x");
    Serial.println(L3GD20H_ADDR, HEX);

    // CTRL1 = 0x0F: power up (Normal mode), all axes, 100 Hz ODR (0000 1111)
    if (!imuI2cWrite(L3GD20H_ADDR, 0x20, 0x0F))
        return false;
    // CTRL4 = 0x00: ±245 dps, BDU=0, little-endian
    if (!imuI2cWrite(L3GD20H_ADDR, 0x23, 0x00))
        return false;

    return true;
}

static void imuPoll(void)
{
    if (!g_imuPresent)
        return;
    uint8_t buf[6];

    if (imuI2cRead(LSM303D_ADDR, 0x28, buf, 6))
    {
        int16_t ax = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        int16_t ay = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
        int16_t az = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
        g_imuAccelMs2[0] = (float)ax * LSM303D_ACCEL_LSB_TO_G * GRAVITY_MS2_F;
        g_imuAccelMs2[1] = (float)ay * LSM303D_ACCEL_LSB_TO_G * GRAVITY_MS2_F;
        g_imuAccelMs2[2] = (float)az * LSM303D_ACCEL_LSB_TO_G * GRAVITY_MS2_F;
    }

    if (imuI2cRead(LSM303D_ADDR, 0x08, buf, 6))
    {
        int16_t mx = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        int16_t my = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
        int16_t mz = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
        g_imuMagGauss[0] = (float)mx * LSM303D_MAG_LSB_TO_GAUSS;
        g_imuMagGauss[1] = (float)my * LSM303D_MAG_LSB_TO_GAUSS;
        g_imuMagGauss[2] = (float)mz * LSM303D_MAG_LSB_TO_GAUSS;
    }

    if (imuI2cRead(L3GD20H_ADDR, 0x28, buf, 6))
    {
        int16_t gx = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        int16_t gy = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
        int16_t gz = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
        g_imuGyroRads[0] = degToRad((float)gx * L3GD20H_GYRO_LSB_TO_DPS);
        g_imuGyroRads[1] = degToRad((float)gy * L3GD20H_GYRO_LSB_TO_DPS);
        g_imuGyroRads[2] = degToRad((float)gz * L3GD20H_GYRO_LSB_TO_DPS);
    }

    // Roll/pitch from accel; yaw integrated from gyro-z with a long-stall
    // clamp so a USB hiccup doesn't lurch yaw by hundreds of degrees.
    uint32_t now = millis();
    float dt = (g_imuLastSampleMs > 0) ? (float)(now - g_imuLastSampleMs) * 0.001f : 0.0f;
    g_imuLastSampleMs = now;
    if (dt > 0.5f)
        dt = 0.0f;

    madgwickUpdate(
        g_imuGyroRads[0], g_imuGyroRads[1], g_imuGyroRads[2],
        g_imuAccelMs2[0], g_imuAccelMs2[1], g_imuAccelMs2[2],
        g_imuMagGauss[0], g_imuMagGauss[1], g_imuMagGauss[2],
        dt);

    quaternionToEuler();
}

static float thetaToModelServoDegInline(float theta, float neutralTheta)
{
    return 90.0f + radToDeg(theta - neutralTheta);
}

static float servoRangePenalty(float modelDeg)
{
    if (modelDeg < 0.0f)
    {
        return -modelDeg;
    }
    if (modelDeg > 180.0f)
    {
        return modelDeg - 180.0f;
    }
    return 0.0f;
}

static float servoSolutionPenalty(float thetaThigh, float thetaServo)
{
    return servoRangePenalty(thetaToModelServoDegInline(thetaThigh, g_thighNeutralAngle)) +
           servoRangePenalty(thetaToModelServoDegInline(thetaServo, g_servoNeutralAngle));
}

static bool footCandidateBetter(float servoPenalty, float error, float bestServoPenalty, float bestError)
{
    if (error < (bestError - 0.5f))
    {
        return true;
    }
    if (error > (bestError + 0.5f))
    {
        return false;
    }
    if ((servoPenalty + 0.000001f) < bestServoPenalty)
    {
        return true;
    }
    if (servoPenalty > (bestServoPenalty + 0.000001f))
    {
        return false;
    }
    return error < bestError;
}

static bool jointCandidateBetter(float servoPenalty, float error, float bestServoPenalty, float bestError)
{
    float tolerance = degToRad(0.75f);
    if (error < (bestError - tolerance))
    {
        return true;
    }
    if (error > (bestError + tolerance))
    {
        return false;
    }
    if ((servoPenalty + 0.000001f) < bestServoPenalty)
    {
        return true;
    }
    if (servoPenalty > (bestServoPenalty + 0.000001f))
    {
        return false;
    }
    return error < bestError;
}

static float footContinuityPenalty(float modelThighDeg, float modelCalfDeg, float startModelThighDeg, float startModelCalfDeg)
{
    return fabsf(modelThighDeg - startModelThighDeg) + fabsf(modelCalfDeg - startModelCalfDeg);
}

static bool footServoCandidateBetter(const FootServoCandidate *candidate, const FootServoCandidate *best)
{
    if (!candidate->valid)
    {
        return false;
    }
    if (!best->valid)
    {
        return true;
    }
    if (footCandidateBetter(candidate->servoPenalty, candidate->error, best->servoPenalty, best->error))
    {
        return true;
    }
    if (footCandidateBetter(best->servoPenalty, best->error, candidate->servoPenalty, candidate->error))
    {
        return false;
    }
    return candidate->continuityPenalty < best->continuityPenalty;
}

static bool jointServoCandidateBetter(const JointServoCandidate *candidate, const JointServoCandidate *best)
{
    if (!candidate->valid)
    {
        return false;
    }
    if (!best->valid)
    {
        return true;
    }
    if (jointCandidateBetter(candidate->servoPenalty, candidate->error, best->servoPenalty, best->error))
    {
        return true;
    }
    if (jointCandidateBetter(best->servoPenalty, best->error, candidate->servoPenalty, candidate->error))
    {
        return false;
    }
    return candidate->continuityPenalty < best->continuityPenalty;
}

static void resetSolverTimingStats(SolverTimingStats *stats)
{
    memset(stats, 0, sizeof(*stats));
}

static void recordSolverTiming(SolverTimingStats *stats, uint32_t elapsedUs, bool success)
{
    stats->count += 1;
    if (success)
    {
        stats->successCount += 1;
    }
    stats->lastUs = elapsedUs;
    if (elapsedUs > stats->maxUs)
    {
        stats->maxUs = elapsedUs;
    }
    stats->totalUs += (double)elapsedUs;
}

static double solverTimingAverageUs(const SolverTimingStats *stats)
{
    if (stats->count == 0)
    {
        return 0.0;
    }
    return stats->totalUs / (double)stats->count;
}

static JointLimitsDeg normalizeJointLimits(JointLimitsDeg limits)
{
    if (limits.thighDeg.min > limits.thighDeg.max)
    {
        float tmp = limits.thighDeg.min;
        limits.thighDeg.min = limits.thighDeg.max;
        limits.thighDeg.max = tmp;
    }

    if (limits.calfDeg.min > limits.calfDeg.max)
    {
        float tmp = limits.calfDeg.min;
        limits.calfDeg.min = limits.calfDeg.max;
        limits.calfDeg.max = tmp;
    }

    return limits;
}

static JointLimitsDeg defaultJointLimits(void)
{
    JointLimitsDeg limits;
    limits.thighDeg.min = DEFAULT_THIGH_MIN_DEG;
    limits.thighDeg.max = DEFAULT_THIGH_MAX_DEG;
    limits.calfDeg.min = DEFAULT_CALF_MIN_DEG;
    limits.calfDeg.max = DEFAULT_CALF_MAX_DEG;
    return limits;
}

static float interpCoupledLimit(float x, const float *xs, const float *ys, int count)
{
    if (x <= xs[0])
    {
        return ys[0];
    }
    if (x >= xs[count - 1])
    {
        return ys[count - 1];
    }

    for (int i = 1; i < count; ++i)
    {
        if (x <= xs[i])
        {
            float span = xs[i] - xs[i - 1];
            float t = span == 0.0f ? 0.0f : (x - xs[i - 1]) / span;
            return lerpf(ys[i - 1], ys[i], t);
        }
    }

    return ys[count - 1];
}

static JointLimitRangeDeg coupledCalfLimitsForThighDeg(float thighDeg, const JointLimitsDeg *limits)
{
    float limitedThighDeg = clampf(thighDeg, limits->thighDeg.min, limits->thighDeg.max);
    JointLimitRangeDeg range;
    range.min = interpCoupledLimit(
        limitedThighDeg,
        COUPLED_THIGH_SAMPLES_DEG,
        COUPLED_CALF_MIN_SAMPLES_DEG,
        COUPLED_LIMIT_SAMPLE_COUNT);
    range.max = interpCoupledLimit(
        limitedThighDeg,
        COUPLED_THIGH_SAMPLES_DEG,
        COUPLED_CALF_MAX_SAMPLES_DEG,
        COUPLED_LIMIT_SAMPLE_COUNT);
    range.min = fmaxf(range.min, limits->calfDeg.min);
    range.max = fminf(range.max, limits->calfDeg.max);

    if (range.min > range.max)
    {
        float mid = 0.5f * (range.min + range.max);
        range.min = mid;
        range.max = mid;
    }

    return range;
}

static bool jointAnglesWithinLimits(float thighDeg, float calfDeg, const JointLimitsDeg *limits)
{
    if (
        thighDeg < limits->thighDeg.min ||
        thighDeg > limits->thighDeg.max ||
        calfDeg < limits->calfDeg.min ||
        calfDeg > limits->calfDeg.max)
    {
        return false;
    }

    JointLimitRangeDeg coupledCalf = coupledCalfLimitsForThighDeg(thighDeg, limits);
    return calfDeg >= coupledCalf.min && calfDeg <= coupledCalf.max;
}

static bool geometryWithinJointLimits(const LegGeometry *geometry, const JointLimitsDeg *limits)
{
    if (!geometry->valid)
    {
        return false;
    }

    float thighDeg = radToDeg(geometry->thetaThigh);
    float calfDeg = radToDeg(geometry->thetaCalf);
    return jointAnglesWithinLimits(thighDeg, calfDeg, limits);
}

static JointAnglesDeg clampJointAnglesToLimits(JointAnglesDeg jointAngles, const JointLimitsDeg *limits)
{
    jointAngles.thighDeg = clampf(jointAngles.thighDeg, limits->thighDeg.min, limits->thighDeg.max);
    JointLimitRangeDeg calfLimits = coupledCalfLimitsForThighDeg(jointAngles.thighDeg, limits);
    jointAngles.calfDeg = clampf(jointAngles.calfDeg, calfLimits.min, calfLimits.max);
    return jointAngles;
}

static float dist2f(Point2f a, Point2f b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

static Point2f footRelativeToAbsolute(Point2f relativeFoot)
{
    Point2f absoluteFoot;
    absoluteFoot.x = relativeFoot.x + FOOT_ORIGIN_OFFSET.x;
    absoluteFoot.y = relativeFoot.y + FOOT_ORIGIN_OFFSET.y;
    return absoluteFoot;
}

static Point2f footAbsoluteToRelative(Point2f absoluteFoot)
{
    Point2f relativeFoot;
    relativeFoot.x = absoluteFoot.x - FOOT_ORIGIN_OFFSET.x;
    relativeFoot.y = absoluteFoot.y - FOOT_ORIGIN_OFFSET.y;
    return relativeFoot;
}

static float servoSpeedDegPerSecFromVoltage(float volts)
{
    float secPer60;
    if (volts >= 8.4f)
    {
        secPer60 = 0.10f;
    }
    else if (volts >= 7.4f)
    {
        float t = (volts - 7.4f) / (8.4f - 7.4f);
        secPer60 = 0.11f + t * (0.10f - 0.11f);
    }
    else if (volts >= 5.0f)
    {
        float t = (volts - 5.0f) / (7.4f - 5.0f);
        secPer60 = 0.13f + t * (0.11f - 0.13f);
    }
    else
    {
        secPer60 = 0.13f;
    }
    return 60.0f / secPer60;
}

static float easeInOutSmoothStep(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static bool circleIntersections(Point2f c0, float r0, Point2f c1, float r1, Point2f *out1, Point2f *out2)
{
    float dx = c1.x - c0.x;
    float dy = c1.y - c0.y;
    float d = sqrtf(dx * dx + dy * dy);
    if (d == 0.0f || d > (r0 + r1) || d < fabsf(r0 - r1))
    {
        return false;
    }

    float a = (r0 * r0 - r1 * r1 + d * d) / (2.0f * d);
    float hSquared = r0 * r0 - a * a;
    if (hSquared < 0.0f)
    {
        return false;
    }

    float h = sqrtf(hSquared);
    float xm = c0.x + (a * dx) / d;
    float ym = c0.y + (a * dy) / d;
    float rx = (-dy * h) / d;
    float ry = (dx * h) / d;
    out1->x = xm + rx;
    out1->y = ym + ry;
    out2->x = xm - rx;
    out2->y = ym - ry;
    return true;
}

static Point2f chooseBellPoint(Point2f p1, Point2f p2)
{
    return (p1.y > p2.y) ? p1 : p2;
}

static Point2f chooseCalfAttachPoint(Point2f p1, Point2f p2, Point2f knee)
{
    float score1 = (knee.y - p1.y) * 3.0f + fabsf(knee.x - p1.x);
    float score2 = (knee.y - p2.y) * 3.0f + fabsf(knee.x - p2.x);
    return (score1 < score2) ? p1 : p2;
}

static LegGeometry solveGeometry(float thetaThigh, float thetaServo)
{
    LegGeometry g;
    g.valid = true;
    g.hip = HIP_PIVOT;
    g.servoPivot = SERVO_PIVOT;
    g.thetaThigh = thetaThigh;
    g.thetaServo = thetaServo;

    g.knee.x = g.hip.x + L_THIGH * cosf(thetaThigh);
    g.knee.y = g.hip.y + L_THIGH * sinf(thetaThigh);
    g.servoHornEnd.x = g.servoPivot.x + HORN * cosf(thetaServo);
    g.servoHornEnd.y = g.servoPivot.y + HORN * sinf(thetaServo);

    Point2f bellI1, bellI2;
    if (!circleIntersections(g.hip, BELL, g.servoHornEnd, LINK_SHORT, &bellI1, &bellI2))
    {
        g.valid = false;
        return g;
    }

    g.bellArmA = chooseBellPoint(bellI1, bellI2);
    float bellAngle = atan2f(g.bellArmA.y - g.hip.y, g.bellArmA.x - g.hip.x);
    g.bellArmB.x = g.hip.x + BELL * cosf(bellAngle - (float)M_PI / 2.0f);
    g.bellArmB.y = g.hip.y + BELL * sinf(bellAngle - (float)M_PI / 2.0f);

    Point2f calfI1, calfI2;
    if (!circleIntersections(g.bellArmB, LINK_LONG, g.knee, CALF_ATTACH_OFFSET, &calfI1, &calfI2))
    {
        g.valid = false;
        return g;
    }

    g.calfAttach = chooseCalfAttachPoint(calfI1, calfI2, g.knee);
    g.thetaCalf = atan2f(g.knee.y - g.calfAttach.y, g.knee.x - g.calfAttach.x);
    g.foot.x = g.knee.x + L_CALF * cosf(g.thetaCalf);
    g.foot.y = g.knee.y + L_CALF * sinf(g.thetaCalf);
    return g;
}

static FootServoCandidate evaluateFootServoCandidate(
    float modelThighDeg,
    float modelCalfDeg,
    Point2f target,
    const JointLimitsDeg *limits,
    float startModelThighDeg,
    float startModelCalfDeg)
{
    FootServoCandidate candidate;
    memset(&candidate, 0, sizeof(candidate));
    candidate.valid = false;

    if (
        modelThighDeg < SERVO_MIN_DEG ||
        modelThighDeg > SERVO_MAX_DEG ||
        modelCalfDeg < SERVO_MIN_DEG ||
        modelCalfDeg > SERVO_MAX_DEG)
    {
        return candidate;
    }

    candidate.modelThighDeg = modelThighDeg;
    candidate.modelCalfDeg = modelCalfDeg;
    candidate.thetaThigh = g_thighNeutralAngle + degToRad(modelThighDeg - 90.0f);
    candidate.thetaServo = g_servoNeutralAngle + degToRad(modelCalfDeg - 90.0f);
    candidate.geometry = solveGeometry(candidate.thetaThigh, candidate.thetaServo);

    if (!geometryWithinJointLimits(&candidate.geometry, limits))
    {
        return candidate;
    }

    candidate.thetaCalf = candidate.geometry.thetaCalf;
    candidate.error = dist2f(candidate.geometry.foot, target);
    candidate.servoPenalty = servoSolutionPenalty(candidate.thetaThigh, candidate.thetaServo);
    candidate.continuityPenalty = footContinuityPenalty(modelThighDeg, modelCalfDeg, startModelThighDeg, startModelCalfDeg);
    candidate.valid = true;
    return candidate;
}

static JointServoCandidate evaluateJointServoCandidate(
    float thetaThigh,
    float modelCalfDeg,
    float targetThetaCalf,
    const JointLimitsDeg *limits,
    float startModelCalfDeg)
{
    JointServoCandidate candidate;
    memset(&candidate, 0, sizeof(candidate));
    candidate.valid = false;

    if (modelCalfDeg < SERVO_MIN_DEG || modelCalfDeg > SERVO_MAX_DEG)
    {
        return candidate;
    }

    candidate.modelCalfDeg = modelCalfDeg;
    candidate.thetaServo = g_servoNeutralAngle + degToRad(modelCalfDeg - 90.0f);
    candidate.geometry = solveGeometry(thetaThigh, candidate.thetaServo);

    if (!geometryWithinJointLimits(&candidate.geometry, limits))
    {
        return candidate;
    }

    candidate.thetaCalf = candidate.geometry.thetaCalf;
    candidate.error = fabsf(clampAngle(candidate.geometry.thetaCalf - targetThetaCalf));
    candidate.servoPenalty = servoSolutionPenalty(thetaThigh, candidate.thetaServo);
    candidate.continuityPenalty = fabsf(modelCalfDeg - startModelCalfDeg);
    candidate.valid = true;
    return candidate;
}

static FootServoCandidate refineFootServoCandidate(
    FootServoCandidate best,
    Point2f target,
    const JointLimitsDeg *limits,
    float startModelThighDeg,
    float startModelCalfDeg)
{
    if (!best.valid)
    {
        return best;
    }

    const size_t refineCount = sizeof(SERVO_REFINE_STEPS_DEG) / sizeof(SERVO_REFINE_STEPS_DEG[0]);
    for (size_t i = 0; i < refineCount; ++i)
    {
        float step = SERVO_REFINE_STEPS_DEG[i];
        bool improved = true;

        while (improved)
        {
            improved = false;
            const float neighbors[8][2] = {
                {best.modelThighDeg + step, best.modelCalfDeg},
                {best.modelThighDeg - step, best.modelCalfDeg},
                {best.modelThighDeg, best.modelCalfDeg + step},
                {best.modelThighDeg, best.modelCalfDeg - step},
                {best.modelThighDeg + step, best.modelCalfDeg + step},
                {best.modelThighDeg + step, best.modelCalfDeg - step},
                {best.modelThighDeg - step, best.modelCalfDeg + step},
                {best.modelThighDeg - step, best.modelCalfDeg - step}};

            for (int neighborIndex = 0; neighborIndex < 8; ++neighborIndex)
            {
                FootServoCandidate candidate = evaluateFootServoCandidate(
                    neighbors[neighborIndex][0],
                    neighbors[neighborIndex][1],
                    target,
                    limits,
                    startModelThighDeg,
                    startModelCalfDeg);
                if (footServoCandidateBetter(&candidate, &best))
                {
                    best = candidate;
                    improved = true;
                }
            }
        }
    }

    return best;
}

static JointServoCandidate refineJointServoCandidate(
    JointServoCandidate best,
    float thetaThigh,
    float targetThetaCalf,
    const JointLimitsDeg *limits,
    float startModelCalfDeg)
{
    if (!best.valid)
    {
        return best;
    }

    const size_t refineCount = sizeof(SERVO_REFINE_STEPS_DEG) / sizeof(SERVO_REFINE_STEPS_DEG[0]);
    for (size_t i = 0; i < refineCount; ++i)
    {
        float step = SERVO_REFINE_STEPS_DEG[i];
        bool improved = true;

        while (improved)
        {
            improved = false;
            const float neighbors[2] = {
                best.modelCalfDeg + step,
                best.modelCalfDeg - step};

            for (int neighborIndex = 0; neighborIndex < 2; ++neighborIndex)
            {
                JointServoCandidate candidate = evaluateJointServoCandidate(
                    thetaThigh,
                    neighbors[neighborIndex],
                    targetThetaCalf,
                    limits,
                    startModelCalfDeg);
                if (jointServoCandidateBetter(&candidate, &best))
                {
                    best = candidate;
                    improved = true;
                }
            }
        }
    }

    return best;
}

static void solveAnglesForAbsoluteFoot(Point2f target, float startThetaThigh, float startThetaServo, const JointLimitsDeg *limits, float *outThetaThigh, float *outThetaServo, float *outFootError, bool *outHasSolution, bool *outSuccess)
{
    float startModelThighDeg = clampf(thetaToModelServoDegInline(startThetaThigh, g_thighNeutralAngle), SERVO_MIN_DEG, SERVO_MAX_DEG);
    float startModelCalfDeg = clampf(thetaToModelServoDegInline(startThetaServo, g_servoNeutralAngle), SERVO_MIN_DEG, SERVO_MAX_DEG);
    FootServoCandidate best;
    memset(&best, 0, sizeof(best));
    best.valid = false;

    for (float modelThighDeg = SERVO_MIN_DEG; modelThighDeg <= SERVO_MAX_DEG; modelThighDeg += SERVO_SEARCH_STEP_DEG)
    {
        for (float modelCalfDeg = SERVO_MIN_DEG; modelCalfDeg <= SERVO_MAX_DEG; modelCalfDeg += SERVO_SEARCH_STEP_DEG)
        {
            FootServoCandidate candidate = evaluateFootServoCandidate(
                modelThighDeg,
                modelCalfDeg,
                target,
                limits,
                startModelThighDeg,
                startModelCalfDeg);
            if (footServoCandidateBetter(&candidate, &best))
            {
                best = candidate;
            }
        }
    }

    best = refineFootServoCandidate(best, target, limits, startModelThighDeg, startModelCalfDeg);

    if (!best.valid)
    {
        LegGeometry fallbackGeometry = solveGeometry(startThetaThigh, startThetaServo);
        *outThetaThigh = startThetaThigh;
        *outThetaServo = startThetaServo;
        *outFootError = FLT_MAX;
        *outHasSolution = false;
        *outSuccess = false;
        (void)fallbackGeometry;
        return;
    }

    *outThetaThigh = best.thetaThigh;
    *outThetaServo = best.thetaServo;
    *outFootError = best.error;
    *outHasSolution = best.geometry.valid && isfinite(best.error);
    *outSuccess = best.geometry.valid && isfinite(best.error) && (best.error < 10.0f);
}

static void solveServoForJointAngles(float thetaThigh, float targetThetaCalf, float startThetaServo, const JointLimitsDeg *limits, float *outThetaServo, float *outJointError, bool *outHasSolution, bool *outSuccess)
{
    float startModelCalfDeg = clampf(thetaToModelServoDegInline(startThetaServo, g_servoNeutralAngle), SERVO_MIN_DEG, SERVO_MAX_DEG);
    JointServoCandidate best;
    memset(&best, 0, sizeof(best));
    best.valid = false;

    for (float modelCalfDeg = SERVO_MIN_DEG; modelCalfDeg <= SERVO_MAX_DEG; modelCalfDeg += SERVO_SEARCH_STEP_DEG)
    {
        JointServoCandidate candidate = evaluateJointServoCandidate(
            thetaThigh,
            modelCalfDeg,
            targetThetaCalf,
            limits,
            startModelCalfDeg);
        if (jointServoCandidateBetter(&candidate, &best))
        {
            best = candidate;
        }
    }

    best = refineJointServoCandidate(best, thetaThigh, targetThetaCalf, limits, startModelCalfDeg);

    if (!best.valid)
    {
        *outThetaServo = startThetaServo;
        *outJointError = FLT_MAX;
        *outHasSolution = false;
        *outSuccess = false;
        return;
    }

    *outThetaServo = best.thetaServo;
    *outJointError = best.error;
    *outHasSolution = best.geometry.valid && isfinite(best.error);
    *outSuccess = best.geometry.valid && isfinite(best.error) && (best.error < degToRad(3.0f));
}

static void runSolverBenchmark(uint32_t iterations, SolverTimingStats *footStats, SolverTimingStats *jointStats)
{
    static const Point2f footTargets[] = {
        {-60.0f, 0.0f},
        {-30.0f, 0.0f},
        {0.0f, 0.0f},
        {20.0f, -8.0f},
        {30.0f, 0.0f}};
    static const JointAnglesDeg jointTargets[] = {
        {-42.0f, -154.0f},
        {-30.0f, -140.0f},
        {-18.0f, -120.0f},
        {-55.0f, -163.0f},
        {-48.0f, -150.0f}};

    RobotLegState *leg = &g_legs[0];
    float thetaThigh = leg->lastThetaThigh;
    float thetaServo = leg->lastThetaServo;

    resetSolverTimingStats(footStats);
    resetSolverTimingStats(jointStats);

    for (uint32_t iteration = 0; iteration < iterations; ++iteration)
    {
        for (size_t targetIndex = 0; targetIndex < (sizeof(footTargets) / sizeof(footTargets[0])); ++targetIndex)
        {
            uint32_t startedUs = micros();
            LegIKResult ik = robotDogLegIK(
                footTargets[targetIndex].x,
                footTargets[targetIndex].y,
                thetaThigh,
                thetaServo,
                &leg->jointLimits);
            uint32_t elapsedUs = micros() - startedUs;
            recordSolverTiming(footStats, elapsedUs, ik.hasSolution);
            if (ik.hasSolution)
            {
                thetaThigh = ik.thetaThigh;
                thetaServo = ik.thetaServo;
            }
        }
    }

    thetaThigh = leg->lastThetaThigh;
    thetaServo = leg->lastThetaServo;

    for (uint32_t iteration = 0; iteration < iterations; ++iteration)
    {
        for (size_t targetIndex = 0; targetIndex < (sizeof(jointTargets) / sizeof(jointTargets[0])); ++targetIndex)
        {
            float solvedThetaServo = thetaServo;
            float jointError = 0.0f;
            bool hasSolution = false;
            bool success = false;
            uint32_t startedUs = micros();
            solveServoForJointAngles(
                degToRad(jointTargets[targetIndex].thighDeg),
                degToRad(jointTargets[targetIndex].calfDeg),
                thetaServo,
                &leg->jointLimits,
                &solvedThetaServo,
                &jointError,
                &hasSolution,
                &success);
            uint32_t elapsedUs = micros() - startedUs;
            recordSolverTiming(jointStats, elapsedUs, hasSolution && success);
            if (hasSolution)
            {
                thetaThigh = degToRad(jointTargets[targetIndex].thighDeg);
                thetaServo = solvedThetaServo;
            }
        }
    }
}

static void robotDogLegInitNeutral(void)
{
    g_thighNeutralAngle = NEUTRAL_THIGH_THETA;
    g_servoNeutralAngle = NEUTRAL_SERVO_THETA;
    g_neutralInitialized = true;
}

static float thetaToModelServoDeg(float theta, float neutralTheta)
{
    return 90.0f + radToDeg(theta - neutralTheta);
}

static float modelServoDegToTheta(float modelDeg, float neutralTheta)
{
    return neutralTheta + degToRad(modelDeg - 90.0f);
}

static LegIKResult robotDogLegIK(float footX, float footY, float startThetaThigh, float startThetaServo, const JointLimitsDeg *limits)
{
    if (!g_neutralInitialized)
    {
        robotDogLegInitNeutral();
    }

    Point2f relativeTarget = {footX, footY};
    Point2f absoluteTarget = footRelativeToAbsolute(relativeTarget);
    float thetaThigh;
    float thetaServo;
    float footError;
    bool hasSolution;
    bool success;

    solveAnglesForAbsoluteFoot(
        absoluteTarget,
        startThetaThigh,
        startThetaServo,
        limits,
        &thetaThigh,
        &thetaServo,
        &footError,
        &hasSolution,
        &success);

    LegGeometry geometry = solveGeometry(thetaThigh, thetaServo);
    LegIKResult result;
    result.thetaThigh = thetaThigh;
    result.thetaServo = thetaServo;
    result.thighServoDeg = thetaToModelServoDeg(thetaThigh, g_thighNeutralAngle);
    result.calfServoDeg = thetaToModelServoDeg(thetaServo, g_servoNeutralAngle);
    result.footError = footError;
    result.resolvedFoot = geometry.valid ? footAbsoluteToRelative(geometry.foot) : relativeTarget;
    result.hasSolution = hasSolution;
    result.success = success;
    return result;
}

static float applyServoCalibration(float modelDeg, float centerDeg, float sign)
{
    return centerDeg + sign * (modelDeg - 90.0f);
}

static float removeServoCalibration(float rawDeg, float centerDeg, float sign)
{
    if (sign == 0.0f)
    {
        return 90.0f;
    }
    return 90.0f + (rawDeg - centerDeg) / sign;
}

static uint16_t servoDegToPcaTicks(float servoDeg)
{
    float clamped = clampf(servoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    float ratio = clamped / 180.0f;
    float pulseUs = SERVO_MIN_PULSE_US + ratio * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    float periodUs = 1000000.0f / g_pwmFrequencyHz;
    float ticks = (pulseUs / periodUs) * 4096.0f;
    return (uint16_t)roundf(ticks);
}

static void applyPca9685Frequency(float hz)
{
    g_pwmFrequencyHz = clampf(hz, MIN_PCA9685_FREQUENCY_HZ, MAX_PCA9685_FREQUENCY_HZ);
    g_pwm.setPWMFreq(g_pwmFrequencyHz);
}

static float smoothServoReadEstimate(const SmoothServo *s);

static void smoothServoRelease(SmoothServo *s)
{
    s->estimatedDeg = smoothServoReadEstimate(s);
    s->startDeg = s->estimatedDeg;
    s->targetDeg = s->estimatedDeg;
    s->moveDurationUs = 1;
    s->moving = false;
    if (s->channel != INVALID_CHANNEL)
    {
        g_pwm.setPWM(s->channel, 0, 4096);
    }
    s->attached = false;
}

static void smoothServoWriteRaw(SmoothServo *s, float rawDeg)
{
    if (s->attached)
    {
        g_pwm.setPWM(s->channel, 0, servoDegToPcaTicks(rawDeg));
    }
}

static void smoothServoWriteEstimate(SmoothServo *s)
{
    smoothServoWriteRaw(s, smoothServoReadEstimate(s));
}

static float smoothServoEffectiveMaxSpeedDegPerSec(const SmoothServo *s)
{
    float limit = s->speedLimitDegPerSec;
    if (limit <= 0.0f)
    {
        limit = s->hardwareMaxSpeedDegPerSec;
    }
    if (limit > s->hardwareMaxSpeedDegPerSec)
    {
        limit = s->hardwareMaxSpeedDegPerSec;
    }
    if (limit < 1.0f)
    {
        limit = 1.0f;
    }
    return limit;
}

static void smoothServoBegin(SmoothServo *s, uint8_t channel, float initialDeg, float minDeg, float maxDeg, float centerDeg, float sign, float supplyVolts)
{
    s->channel = channel;
    s->attached = (channel != INVALID_CHANNEL);
    s->minDeg = minDeg;
    s->maxDeg = maxDeg;
    s->centerDeg = centerDeg;
    s->sign = sign;
    s->hardwareMaxSpeedDegPerSec = servoSpeedDegPerSecFromVoltage(supplyVolts) * SERVO_SPEED_SAFETY_FACTOR;
    s->speedLimitDegPerSec = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
    s->startDeg = clampf(initialDeg, minDeg, maxDeg);
    s->targetDeg = s->startDeg;
    s->estimatedDeg = s->startDeg;
    s->moveStartUs = micros();
    s->moveDurationUs = 1;
    s->moving = false;
    smoothServoWriteEstimate(s);
}

static float smoothServoReadEstimate(const SmoothServo *s)
{
    return s->estimatedDeg;
}

static void smoothServoCommandRaw(SmoothServo *s, float rawTargetDeg)
{
    if (s->channel != INVALID_CHANNEL)
    {
        s->attached = true;
    }
    rawTargetDeg = clampf(rawTargetDeg, s->minDeg, s->maxDeg);
    float currentEstimate = s->estimatedDeg;
    s->estimatedDeg = currentEstimate;
    s->startDeg = currentEstimate;
    s->targetDeg = rawTargetDeg;
    s->moveStartUs = micros();
    s->moveDurationUs = 1;
    s->moving = fabsf(s->targetDeg - s->estimatedDeg) > 0.01f;
    smoothServoWriteEstimate(s);
}

static void smoothServoCommandRawImmediate(SmoothServo *s, float rawTargetDeg)
{
    if (s->channel != INVALID_CHANNEL)
    {
        s->attached = true;
    }
    rawTargetDeg = clampf(rawTargetDeg, s->minDeg, s->maxDeg);
    s->startDeg = rawTargetDeg;
    s->targetDeg = rawTargetDeg;
    s->estimatedDeg = rawTargetDeg;
    s->moveStartUs = micros();
    s->moveDurationUs = 1;
    s->moving = false;
    smoothServoWriteRaw(s, rawTargetDeg);
}

static void smoothServoCommandModelDeg(SmoothServo *s, float modelDeg)
{
    float raw = applyServoCalibration(modelDeg, s->centerDeg, s->sign);
    smoothServoCommandRaw(s, raw);
}

static void smoothServoCommandModelDegImmediate(SmoothServo *s, float modelDeg)
{
    float raw = applyServoCalibration(modelDeg, s->centerDeg, s->sign);
    smoothServoCommandRawImmediate(s, raw);
}

static void smoothServoSetSpeedLimitDegPerSec(SmoothServo *s, float speedLimitDegPerSec)
{
    s->speedLimitDegPerSec = speedLimitDegPerSec;
    if (!s->attached)
    {
        return;
    }

    float currentEstimate = s->estimatedDeg;
    s->estimatedDeg = currentEstimate;
    s->startDeg = currentEstimate;
    s->moveStartUs = micros();
    s->moveDurationUs = 1;
    s->moving = fabsf(s->targetDeg - s->estimatedDeg) > 0.01f;
    smoothServoWriteEstimate(s);
}

static void smoothServoUpdate(SmoothServo *s)
{
    uint32_t nowUs = micros();

    if (s->moving)
    {
        uint32_t elapsedUs = nowUs - s->moveStartUs;
        if (elapsedUs > 0)
        {
            float effectiveMaxSpeed = smoothServoEffectiveMaxSpeedDegPerSec(s) / MOTION_TIME_SCALE;
            float maxDelta = effectiveMaxSpeed * ((float)elapsedUs / 1000000.0f);
            float remaining = s->targetDeg - s->estimatedDeg;
            float step = clampf(remaining, -maxDelta, maxDelta);

            if (fabsf(remaining) <= fmaxf(0.01f, maxDelta))
            {
                s->estimatedDeg = s->targetDeg;
                s->moving = false;
            }
            else
            {
                s->estimatedDeg = clampf(s->estimatedDeg + step, s->minDeg, s->maxDeg);
            }
        }
    }

    s->startDeg = s->estimatedDeg;
    s->moveStartUs = nowUs;
    s->moveDurationUs = 1;
    smoothServoWriteRaw(s, s->estimatedDeg);
}

static ServoAnglesDeg getCurrentServoAngles(const RobotLegState *leg)
{
    ServoAnglesDeg result;
    result.hip = removeServoCalibration(leg->hip.estimatedDeg, leg->hip.centerDeg, leg->hip.sign);
    result.thigh = removeServoCalibration(leg->thigh.estimatedDeg, leg->thigh.centerDeg, leg->thigh.sign);
    result.calf = removeServoCalibration(leg->calf.estimatedDeg, leg->calf.centerDeg, leg->calf.sign);
    return result;
}

static void updateLegDerivedStateFromServo(RobotLegState *leg, ServoAnglesDeg servoAngles, bool desiredState)
{
    float thetaThigh = modelServoDegToTheta(servoAngles.thigh, g_thighNeutralAngle);
    float thetaServo = modelServoDegToTheta(servoAngles.calf, g_servoNeutralAngle);
    LegGeometry geometry = solveGeometry(thetaThigh, thetaServo);

    if (!geometry.valid)
    {
        return;
    }

    if (desiredState)
    {
        leg->desiredServoAngles = servoAngles;
        leg->desiredJointAngles.thighDeg = radToDeg(geometry.thetaThigh);
        leg->desiredJointAngles.calfDeg = radToDeg(geometry.thetaCalf);
        leg->desiredFoot = footAbsoluteToRelative(geometry.foot);
    }
    else
    {
        leg->currentServoAngles = servoAngles;
        leg->currentJointAngles.thighDeg = radToDeg(geometry.thetaThigh);
        leg->currentJointAngles.calfDeg = radToDeg(geometry.thetaCalf);
        leg->currentFoot = footAbsoluteToRelative(geometry.foot);
    }
}

static int legIndexFromId(const char *id)
{
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        if (strcmp(id, LEG_CONFIGS[i].id) == 0)
        {
            return i;
        }
    }
    return -1;
}

static const char *modeToString(RobotMode mode)
{
    switch (mode)
    {
    case MODE_IDLE:
        return "idle";
    case MODE_DIRECT_FOOT_XY:
        return "direct_foot_xy";
    case MODE_DIRECT_JOINT_ANGLES:
        return "direct_joint_angles";
    case MODE_DIRECT_SERVO_ANGLES:
        return "direct_servo_angles";
    case MODE_BUILTIN_WALK:
        return "builtin_walk";
    case MODE_BUILTIN_CROUCH:
        return "builtin_crouch";
    case MODE_ANIMATION_PLAYBACK:
        return "animation_playback";
    default:
        return "idle";
    }
}

static void setMode(RobotMode mode)
{
    g_mode = mode;
    g_modeStartMs = millis();

    if (mode != MODE_BUILTIN_WALK && mode != MODE_BUILTIN_CROUCH)
    {
        g_builtinName[0] = '\0';
    }

    // Clear walk intent whenever we leave walking mode so LED state is clean.
    if (mode != MODE_BUILTIN_WALK)
    {
        g_walkIntent = WALK_STOP;
    }

    if (mode != MODE_ANIMATION_PLAYBACK)
    {
        g_animation.playing = false;
        g_animation.paused = false;
    }
}

static void clearLegError(RobotLegState *leg)
{
    leg->lastError[0] = '\0';
}

static void setLegError(RobotLegState *leg, const char *message)
{
    strncpy(leg->lastError, message, MAX_ERROR_LEN - 1);
    leg->lastError[MAX_ERROR_LEN - 1] = '\0';
}

static void robotLegBegin(RobotLegState *leg, const LegHardwareConfig *config)
{
    smoothServoBegin(&leg->hip, config->hipChannel, HIP_CENTER_DEG, SERVO_MIN_DEG, SERVO_MAX_DEG, HIP_CENTER_DEG, config->hipSign, SERVO_SUPPLY_VOLTS);
    smoothServoBegin(&leg->thigh, config->thighChannel, THIGH_CENTER_DEG, SERVO_MIN_DEG, SERVO_MAX_DEG, THIGH_CENTER_DEG, config->thighSign, SERVO_SUPPLY_VOLTS);
    smoothServoBegin(&leg->calf, config->calfChannel, CALF_CENTER_DEG, SERVO_MIN_DEG, SERVO_MAX_DEG, CALF_CENTER_DEG, config->calfSign, SERVO_SUPPLY_VOLTS);
    leg->hipChannel = config->hipChannel;
    leg->thighChannel = config->thighChannel;
    leg->calfChannel = config->calfChannel;
    leg->jointLimits = defaultJointLimits();
    leg->servoSpeedLimitDegPerSec.hip = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
    leg->servoSpeedLimitDegPerSec.thigh = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
    leg->servoSpeedLimitDegPerSec.calf = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
    leg->lastThetaThigh = g_thighNeutralAngle;
    leg->lastThetaServo = g_servoNeutralAngle;
    leg->desiredFoot.x = 0.0f;
    leg->desiredFoot.y = 0.0f;
    leg->desiredServoAngles.hip = 90.0f;
    leg->desiredServoAngles.thigh = 90.0f;
    leg->desiredServoAngles.calf = 90.0f;
    leg->desiredJointAngles.thighDeg = radToDeg(g_thighNeutralAngle);
    leg->desiredJointAngles.calfDeg = -90.0f;
    leg->currentFoot = leg->desiredFoot;
    leg->currentServoAngles = leg->desiredServoAngles;
    leg->currentJointAngles = leg->desiredJointAngles;
    clearLegError(leg);
    updateLegDerivedStateFromServo(leg, leg->desiredServoAngles, true);
    updateLegDerivedStateFromServo(leg, leg->desiredServoAngles, false);
}

// Front-right hip is mechanically/electrically locked out — anything that
// tries to command it (gait planner, dashboard direct command, animation,
// builtin walk) gets overridden here to a fixed 90°. Re-asserted EVERY
// tick before smoothServoUpdate runs, and desiredServoAngles.hip is
// pinned so the dashboard panel reflects 90° too. Flip FR_HIP_LOCKED
// false to release.
static const bool  FR_HIP_LOCKED      = true;
static const float FR_HIP_LOCK_DEG    = 90.0f;
static const int   FR_LEG_INDEX       = 1; // matches LEG_CONFIGS[1] = "front_right"

static void robotLegUpdate(RobotLegState *leg)
{
    if (FR_HIP_LOCKED && leg == &g_legs[FR_LEG_INDEX])
    {
        leg->hip.targetDeg     = FR_HIP_LOCK_DEG;
        leg->hip.startDeg      = FR_HIP_LOCK_DEG;
        leg->hip.estimatedDeg  = FR_HIP_LOCK_DEG;
        leg->hip.moving        = false;
        leg->hip.moveDurationUs = 0;
        leg->desiredServoAngles.hip = FR_HIP_LOCK_DEG;
    }
    smoothServoUpdate(&leg->hip);
    smoothServoUpdate(&leg->thigh);
    smoothServoUpdate(&leg->calf);
    ServoAnglesDeg currentServoAngles = getCurrentServoAngles(leg);
    updateLegDerivedStateFromServo(leg, currentServoAngles, false);
}

static bool robotLegCommandJointAngles(int legIndex, float thighDeg, float calfDeg);

static void robotLegSetServoChannels(int legIndex, uint8_t hipChannel, uint8_t thighChannel, uint8_t calfChannel)
{
    RobotLegState *leg = &g_legs[legIndex];
    leg->hip.channel = hipChannel;
    leg->thigh.channel = thighChannel;
    leg->calf.channel = calfChannel;
    leg->hip.attached = (hipChannel != INVALID_CHANNEL);
    leg->thigh.attached = (thighChannel != INVALID_CHANNEL);
    leg->calf.attached = (calfChannel != INVALID_CHANNEL);
    leg->hipChannel = hipChannel;
    leg->thighChannel = thighChannel;
    leg->calfChannel = calfChannel;
    smoothServoWriteEstimate(&leg->hip);
    smoothServoWriteEstimate(&leg->thigh);
    smoothServoWriteEstimate(&leg->calf);
}

static void releaseAllServos(void)
{
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        smoothServoRelease(&g_legs[i].hip);
        smoothServoRelease(&g_legs[i].thigh);
        smoothServoRelease(&g_legs[i].calf);
    }
    g_activeAnimationName[0] = '\0';
    setMode(MODE_IDLE);
    g_servosReleased = true;
}

static void robotLegSetJointLimits(int legIndex, JointLimitsDeg limits)
{
    RobotLegState *leg = &g_legs[legIndex];
    leg->jointLimits = normalizeJointLimits(limits);
    robotLegCommandJointAngles(legIndex, leg->desiredJointAngles.thighDeg, leg->desiredJointAngles.calfDeg);
}

static void robotLegSetServoSpeedLimit(int legIndex, float hipDegPerSec, float thighDegPerSec, float calfDegPerSec)
{
    RobotLegState *leg = &g_legs[legIndex];
    leg->servoSpeedLimitDegPerSec.hip = hipDegPerSec;
    leg->servoSpeedLimitDegPerSec.thigh = thighDegPerSec;
    leg->servoSpeedLimitDegPerSec.calf = calfDegPerSec;
    smoothServoSetSpeedLimitDegPerSec(&leg->hip, hipDegPerSec);
    smoothServoSetSpeedLimitDegPerSec(&leg->thigh, thighDegPerSec);
    smoothServoSetSpeedLimitDegPerSec(&leg->calf, calfDegPerSec);
}

static void robotLegCommandServo(int legIndex, float hipServoDeg, float thighServoDeg, float calfServoDeg)
{
    RobotLegState *leg = &g_legs[legIndex];
    g_servosReleased = false;
    smoothServoCommandModelDeg(&leg->hip, hipServoDeg);
    smoothServoCommandModelDeg(&leg->thigh, thighServoDeg);
    smoothServoCommandModelDeg(&leg->calf, calfServoDeg);
    leg->desiredServoAngles.hip = clampf(hipServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    leg->desiredServoAngles.thigh = clampf(thighServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    leg->desiredServoAngles.calf = clampf(calfServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    updateLegDerivedStateFromServo(leg, leg->desiredServoAngles, true);
}

static void robotLegCommandServoImmediate(int legIndex, float hipServoDeg, float thighServoDeg, float calfServoDeg)
{
    RobotLegState *leg = &g_legs[legIndex];
    g_servosReleased = false;
    smoothServoCommandModelDegImmediate(&leg->hip, hipServoDeg);
    smoothServoCommandModelDegImmediate(&leg->thigh, thighServoDeg);
    smoothServoCommandModelDegImmediate(&leg->calf, calfServoDeg);
    leg->desiredServoAngles.hip = clampf(hipServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    leg->desiredServoAngles.thigh = clampf(thighServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    leg->desiredServoAngles.calf = clampf(calfServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    updateLegDerivedStateFromServo(leg, leg->desiredServoAngles, true);
}

static bool robotLegCommandFoot(int legIndex, float footX, float footY)
{
    RobotLegState *leg = &g_legs[legIndex];
    uint32_t startedUs = micros();
    LegIKResult ik = robotDogLegIK(footX, footY, leg->lastThetaThigh, leg->lastThetaServo, &leg->jointLimits);
    recordSolverTiming(&g_footSolveStats, micros() - startedUs, ik.hasSolution && ik.success);
    if (!ik.hasSolution)
    {
        setLegError(leg, "IK failed for foot target");
        return false;
    }

    robotLegCommandServo(legIndex, leg->desiredServoAngles.hip, ik.thighServoDeg, ik.calfServoDeg);
    leg->desiredFoot = ik.resolvedFoot;
    leg->lastThetaThigh = ik.thetaThigh;
    leg->lastThetaServo = ik.thetaServo;
    clearLegError(leg);
    return true;
}

static bool robotLegCommandJointAngles(int legIndex, float thighDeg, float calfDeg)
{
    RobotLegState *leg = &g_legs[legIndex];
    JointAnglesDeg requestedAngles;
    requestedAngles.thighDeg = thighDeg;
    requestedAngles.calfDeg = calfDeg;
    JointAnglesDeg limitedAngles = clampJointAnglesToLimits(requestedAngles, &leg->jointLimits);
    float thetaThigh = degToRad(limitedAngles.thighDeg);
    float thetaServo;
    float jointError;
    bool hasSolution;
    bool success;
    uint32_t startedUs = micros();

    solveServoForJointAngles(thetaThigh, degToRad(limitedAngles.calfDeg), leg->lastThetaServo, &leg->jointLimits, &thetaServo, &jointError, &hasSolution, &success);
    recordSolverTiming(&g_jointSolveStats, micros() - startedUs, hasSolution && success);
    if (!hasSolution)
    {
        setLegError(leg, "Joint angle solve failed");
        return false;
    }

    float thighServoDeg = thetaToModelServoDeg(thetaThigh, g_thighNeutralAngle);
    float calfServoDeg = thetaToModelServoDeg(thetaServo, g_servoNeutralAngle);
    robotLegCommandServo(legIndex, leg->desiredServoAngles.hip, thighServoDeg, calfServoDeg);
    leg->desiredJointAngles.thighDeg = limitedAngles.thighDeg;
    leg->desiredJointAngles.calfDeg = limitedAngles.calfDeg;
    leg->lastThetaThigh = thetaThigh;
    leg->lastThetaServo = thetaServo;
    clearLegError(leg);
    return true;
}

static void animationReset(AnimationClip *clip)
{
    memset(clip, 0, sizeof(AnimationClip));
}

static void animationBeginUpload(AnimationClip *clip, const char *name, float duration)
{
    animationReset(clip);
    strncpy(clip->name, name, MAX_NAME_LEN - 1);
    clip->name[MAX_NAME_LEN - 1] = '\0';
    clip->duration = clampf(duration, 0.1f, 30.0f);
}

static bool animationAddFrame(AnimationClip *clip, int legIndex, float time, float x, float y)
{
    if (legIndex < 0 || legIndex >= NUM_LEGS)
    {
        return false;
    }

    LegTrack *track = &clip->tracks[legIndex];
    if (track->count >= MAX_ANIM_KEYFRAMES)
    {
        return false;
    }

    track->frames[track->count].time = clampf(time, 0.0f, clip->duration);
    track->frames[track->count].x = x;
    track->frames[track->count].y = y;
    track->count += 1;
    return true;
}

static void animationCommit(AnimationClip *clip)
{
    clip->ready = true;
    clip->playing = false;
    clip->paused = false;
}

static Point2f interpolateTrack(const LegTrack *track, float timeSec)
{
    Point2f result = {0.0f, 0.0f};
    if (track->count <= 0)
    {
        return result;
    }

    if (timeSec <= track->frames[0].time)
    {
        result.x = track->frames[0].x;
        result.y = track->frames[0].y;
        return result;
    }

    if (timeSec >= track->frames[track->count - 1].time)
    {
        result.x = track->frames[track->count - 1].x;
        result.y = track->frames[track->count - 1].y;
        return result;
    }

    for (int i = 0; i < track->count - 1; ++i)
    {
        AnimationFrame a = track->frames[i];
        AnimationFrame b = track->frames[i + 1];
        if (timeSec >= a.time && timeSec <= b.time)
        {
            float span = b.time - a.time;
            float t = (span > 0.0f) ? (timeSec - a.time) / span : 0.0f;
            result.x = lerpf(a.x, b.x, t);
            result.y = lerpf(a.y, b.y, t);
            return result;
        }
    }

    result.x = track->frames[track->count - 1].x;
    result.y = track->frames[track->count - 1].y;
    return result;
}

static Point2f builtinWalkTarget(int legIndex, float timeSec)
{
    static const float phaseOffsets[NUM_LEGS] = {0.0f, 0.5f, 0.5f, 0.0f};
    float cycle = fmodf((timeSec / 1.6f) + phaseOffsets[legIndex], 1.0f);
    float x = lerpf(-18.0f, 18.0f, cycle);
    float y = (cycle < 0.45f) ? (10.0f * sinf((cycle / 0.45f) * (float)M_PI)) : (-6.0f * sinf(((cycle - 0.45f) / 0.55f) * (float)M_PI));
    Point2f foot = {x, y};
    return foot;
}

static Point2f builtinCrouchTarget(float timeSec)
{
    float t = clampf(timeSec / 0.9f, 0.0f, 1.0f);
    float eased = easeInOutSmoothStep(t);
    Point2f foot = {0.0f, lerpf(0.0f, -28.0f, eased)};
    return foot;
}

static void jsonPrintEscaped(const char *value)
{
    while (*value)
    {
        if (*value == '"' || *value == '\\')
        {
            Serial.print('\\');
        }
        Serial.print(*value);
        value += 1;
    }
}

static void sendAck(int seq, const char *message)
{
    Serial.print("{\"type\":\"ack\",\"seq\":");
    Serial.print(seq);
    Serial.print(",\"message\":\"");
    jsonPrintEscaped(message);
    Serial.println("\"}");
}

static void sendError(int seq, const char *message)
{
    Serial.print("{\"type\":\"error\",\"seq\":");
    Serial.print(seq);
    Serial.print(",\"message\":\"");
    jsonPrintEscaped(message);
    Serial.println("\"}");
}

static void sendBuiltinStatus(void)
{
    Serial.print("{\"type\":\"builtin_status\",\"name\":\"");
    jsonPrintEscaped(g_builtinName);
    Serial.print("\",\"mode\":\"");
    Serial.print(modeToString(g_mode));
    Serial.println("\"}");
}

// Standalone IMU JSON message — separate from sendStateMessage so the
// dashboard's gait stabilizer can consume IMU at a higher rate
// (IMU_TELEMETRY_INTERVAL_MS = 20 ms) without dragging along the full
// 2.3 KB state payload. Skipped silently when imu.present is false so the
// host never sees zeroed accel/gyro readings that the gait stabilizer
// would mistake for the bot lying perfectly level. Now that the Madgwick
// filter populates g_imuRollRad/g_imuPitchRad/g_imuYawRad as fused euler
// angles, this channel is what the dashboard's gait stabilizer reads.
static void sendImuMessage(void)
{
    if (!g_imuPresent) return;
    char buf[384];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"imu\",\"payload\":{"
        "\"present\":true,"
        "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
        "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
        "\"mx\":%.4f,\"my\":%.4f,\"mz\":%.4f,"
        "\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f,"
        "\"firmwareMs\":%lu}}\n",
        g_imuAccelMs2[0], g_imuAccelMs2[1], g_imuAccelMs2[2],
        g_imuGyroRads[0], g_imuGyroRads[1], g_imuGyroRads[2],
        g_imuMagGauss[0], g_imuMagGauss[1], g_imuMagGauss[2],
        g_imuRollRad, g_imuPitchRad, g_imuYawRad,
        (unsigned long)millis());
    if (n > 0) Serial.write((const uint8_t *)buf, (size_t)n);
}

static void sendAnimationProgress(void)
{
    float elapsedSec = 0.0f;
    if (g_animation.playing && g_animation.duration > 0.0f)
    {
        elapsedSec = fmodf((millis() - g_animation.startMs) / 1000.0f, g_animation.duration);
    }
    else if (g_animation.paused)
    {
        elapsedSec = g_animation.pausedTimeSec;
    }

    Serial.print("{\"type\":\"animation_progress\",\"name\":\"");
    jsonPrintEscaped(g_activeAnimationName);
    Serial.print("\",\"time\":");
    Serial.print(elapsedSec, 3);
    Serial.println("}");
}

static void sendSolverTimingStatsJson(const char *name, const SolverTimingStats *stats)
{
    Serial.print("\"");
    Serial.print(name);
    Serial.print("\":{\"count\":");
    Serial.print(stats->count);
    Serial.print(",\"successCount\":");
    Serial.print(stats->successCount);
    Serial.print(",\"lastUs\":");
    Serial.print(stats->lastUs);
    Serial.print(",\"maxUs\":");
    Serial.print(stats->maxUs);
    Serial.print(",\"avgUs\":");
    Serial.print(solverTimingAverageUs(stats), 3);
    Serial.print("}");
}

static void sendBenchmarkResult(int seq, uint32_t iterations, const SolverTimingStats *footStats, const SolverTimingStats *jointStats)
{
    Serial.print("{\"type\":\"benchmark_result\",\"seq\":");
    Serial.print(seq);
    Serial.print(",\"iterations\":");
    Serial.print(iterations);
    Serial.print(",\"results\":{");
    sendSolverTimingStatsJson("foot", footStats);
    Serial.print(",");
    sendSolverTimingStatsJson("joint", jointStats);
    Serial.println("}}");
}

// Append `value` to `buf` (capacity `cap`) starting at `*pos`, escaping JSON
// special characters. Updates `*pos`. Stops cleanly at the buffer edge so the
// caller's snprintf safety chain stays valid even if the input string is too
// long. Returns the new position.
static int jsonAppendEscaped(char *buf, size_t cap, int pos, const char *value)
{
    if (!buf || pos < 0)
        return pos;
    while (*value && (size_t)pos + 2 < cap)
    {
        char c = *value++;
        if (c == '"' || c == '\\')
        {
            buf[pos++] = '\\';
        }
        buf[pos++] = c;
    }
    return pos;
}

// Bounded snprintf wrapper. snprintf returns the *desired* byte count which
// can exceed remaining capacity; if we naively `pos += snprintf(...)` and
// then call again with `cap - pos`, the second size argument can become
// negative and underflow to a huge size_t — which lets snprintf overrun the
// buffer. This wrapper saturates at `cap` so subsequent calls stay safe.
#define BUF_PRINTF(buf, cap, pos, ...)                                              \
    do                                                                              \
    {                                                                               \
        if ((pos) < (cap))                                                          \
        {                                                                           \
            int _n = snprintf((buf) + (pos), (size_t)((cap) - (pos)), __VA_ARGS__); \
            if (_n < 0)                                                             \
            { /* encoding error */                                                  \
            }                                                                       \
            else if ((pos) + _n >= (cap))                                           \
                (pos) = (cap);                                                      \
            else                                                                    \
                (pos) += _n;                                                        \
        }                                                                           \
    } while (0)

// State telemetry. Built into a single static buffer and shipped with one
// Serial.write() call so the ESP32 UART driver makes one TX hand-off instead
// of ~160. The previous chained Serial.print() implementation reproducibly
// starved the RX driver task during the burst, which overflowed the 4 KB RX
// FIFO mid-frame and corrupted incoming set_leg_servo_angles commands (host
// log: `[serial] unknown command type [parsed='' raw='oDeg":...']`). Buffer
// is sized for the worst-case payload (~2.3 KB) with margin; TX ring is
// 8192 B (see setup()), so the write is a memcpy and returns immediately.
static void sendStateMessage(const char *typeName)
{
    static char buf[3584];
    int pos = 0;
    const int cap = (int)sizeof(buf);
    int gasRaw = 0;
    float gasPpm = 0.0f;
    float gasPpb = 0.0f;
    float gasMgM3 = 0.0f;
    float gasUgM3 = 0.0f;
    mq131ReadO3(&gasRaw, &gasPpm, &gasPpb, &gasMgM3, &gasUgM3);

    BUF_PRINTF(buf, cap, pos,
               "{\"type\":\"%s\",\"payload\":{\"mode\":\"%s\",\"activeAnimation\":\"",
               typeName, modeToString(g_mode));
    pos = jsonAppendEscaped(buf, cap, pos, g_activeAnimationName);
    // IMU fields used to ride along here, but they're now emitted on their
    // own {"type":"imu",...} channel at 50 Hz (sendImuMessage()) so the gait
    // stabilizer doesn't have to wait for the 5 Hz state frame. We still
    // include "imuPresent" as a one-bit availability hint for the dashboard
    // panel.
    BUF_PRINTF(buf, cap, pos,
               "\",\"servosReleased\":%s,\"servoUpdateRateHz\":%.3f,"
               "\"o3Ppb\":%.2f,"
               "\"gas\":{\"species\":\"o3\",\"raw\":%d,\"ppm\":%.6f,\"ppb\":%.2f,\"mgM3\":%.6f,\"ugM3\":%.2f},"
               "\"imuPresent\":%s,"
               "\"firmwareMs\":%lu,\"legs\":{",
               g_servosReleased ? "true" : "false",
               g_pwmFrequencyHz,
               gasPpb,
               gasRaw, gasPpm, gasPpb, gasMgM3, gasUgM3,
               g_imuPresent ? "true" : "false",
               (unsigned long)millis());

    for (int i = 0; i < NUM_LEGS; ++i)
    {
        if (i > 0 && pos < cap - 1)
        {
            buf[pos++] = ',';
        }
        RobotLegState *leg = &g_legs[i];
        BUF_PRINTF(buf, cap, pos,
                   "\"%s\":{\"status\":\"%s\",\"lastError\":\"",
                   LEG_CONFIGS[i].id, modeToString(g_mode));
        pos = jsonAppendEscaped(buf, cap, pos, leg->lastError);
        BUF_PRINTF(buf, cap, pos,
                   "\",\"servoChannelMap\":{\"hip\":%d,\"thigh\":%d,\"calf\":%d},"
                   "\"servoSpeedLimitDegPerSec\":{\"hip\":%.3f,\"thigh\":%.3f,\"calf\":%.3f},"
                   "\"jointLimits\":{\"thighDeg\":{\"min\":%.3f,\"max\":%.3f},"
                   "\"calfDeg\":{\"min\":%.3f,\"max\":%.3f}},"
                   "\"desired\":{\"servoAnglesDeg\":{\"hip\":%.3f,\"thigh\":%.3f,\"calf\":%.3f}},"
                   "\"current\":{\"servoAnglesDeg\":{\"hip\":%.3f,\"thigh\":%.3f,\"calf\":%.3f}}}",
                   (int)leg->hipChannel, (int)leg->thighChannel, (int)leg->calfChannel,
                   leg->hip.speedLimitDegPerSec, leg->thigh.speedLimitDegPerSec, leg->calf.speedLimitDegPerSec,
                   leg->jointLimits.thighDeg.min, leg->jointLimits.thighDeg.max,
                   leg->jointLimits.calfDeg.min, leg->jointLimits.calfDeg.max,
                   leg->desiredServoAngles.hip, leg->desiredServoAngles.thigh, leg->desiredServoAngles.calf,
                   leg->currentServoAngles.hip, leg->currentServoAngles.thigh, leg->currentServoAngles.calf);
        if (pos >= cap - 1)
            break;
    }

    BUF_PRINTF(buf, cap, pos, "}}}\n");

    Serial.write((const uint8_t *)buf, (size_t)pos);
}

static bool jsonExtractString(const char *json, const char *key, char *out, size_t outSize)
{
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *start = strstr(json, pattern);
    if (!start)
    {
        return false;
    }
    start += strlen(pattern);
    const char *end = strchr(start, '"');
    if (!end)
    {
        return false;
    }
    size_t len = (size_t)(end - start);
    if (len >= outSize)
    {
        len = outSize - 1;
    }
    memcpy(out, start, len);
    out[len] = '\0';
    return true;
}

static bool jsonExtractFloat(const char *json, const char *key, float *outValue)
{
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *start = strstr(json, pattern);
    if (!start)
    {
        return false;
    }
    start += strlen(pattern);
    *outValue = strtof(start, NULL);
    return true;
}

static bool jsonExtractInt(const char *json, const char *key, int *outValue)
{
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *start = strstr(json, pattern);
    if (!start)
    {
        return false;
    }
    start += strlen(pattern);
    *outValue = (int)strtol(start, NULL, 10);
    return true;
}

static void refreshCurrentState(void)
{
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        g_legs[i].currentServoAngles = getCurrentServoAngles(&g_legs[i]);
        updateLegDerivedStateFromServo(&g_legs[i], g_legs[i].currentServoAngles, false);
    }
}

static void runBuiltinsAndAnimation(void)
{
    float elapsedSec = (millis() - g_modeStartMs) / 1000.0f;

    if (g_mode == MODE_BUILTIN_WALK)
    {
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            Point2f foot = builtinWalkTarget(i, elapsedSec);
            robotLegCommandFoot(i, foot.x, foot.y);
        }
    }

    if (g_mode == MODE_BUILTIN_CROUCH)
    {
        Point2f foot = builtinCrouchTarget(elapsedSec);
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            robotLegCommandFoot(i, foot.x, foot.y);
        }
    }

    if (g_mode == MODE_ANIMATION_PLAYBACK && g_animation.ready && g_animation.playing && !g_animation.paused)
    {
        float timeSec = fmodf((millis() - g_animation.startMs) / 1000.0f, g_animation.duration);
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            Point2f foot = interpolateTrack(&g_animation.tracks[i], timeSec);
            robotLegCommandFoot(i, foot.x, foot.y);
        }
    }
}

static void maybeSendPeriodicEvents(void)
{
    uint32_t nowMs = millis();
    if (g_imuPresent && (nowMs - g_lastImuPollMs) >= IMU_POLL_INTERVAL_MS)
    {
        g_lastImuPollMs = nowMs;
        imuPoll();
    }
    if (g_imuPresent && (nowMs - g_lastImuTelemetryMs) >= IMU_TELEMETRY_INTERVAL_MS)
    {
        g_lastImuTelemetryMs = nowMs;
        sendImuMessage();
    }
    if ((nowMs - g_lastTelemetryMs) >= TELEMETRY_INTERVAL_MS)
    {
        g_lastTelemetryMs = nowMs;
        refreshCurrentState();
        sendStateMessage("state");
    }

    if ((g_mode == MODE_BUILTIN_WALK || g_mode == MODE_BUILTIN_CROUCH) && (nowMs - g_lastBuiltinStatusMs) >= BUILTIN_STATUS_INTERVAL_MS)
    {
        g_lastBuiltinStatusMs = nowMs;
        sendBuiltinStatus();
    }

    if (g_mode == MODE_ANIMATION_PLAYBACK && (nowMs - g_lastAnimationStatusMs) >= ANIMATION_STATUS_INTERVAL_MS)
    {
        g_lastAnimationStatusMs = nowMs;
        sendAnimationProgress();
    }
}

static void handleModeCommand(const char *modeString)
{
    if (strcmp(modeString, "idle") == 0)
        setMode(MODE_IDLE);
    if (strcmp(modeString, "direct_foot_xy") == 0)
        setMode(MODE_DIRECT_FOOT_XY);
    if (strcmp(modeString, "direct_joint_angles") == 0)
        setMode(MODE_DIRECT_JOINT_ANGLES);
    if (strcmp(modeString, "direct_servo_angles") == 0)
        setMode(MODE_DIRECT_SERVO_ANGLES);
    if (strcmp(modeString, "builtin_walk") == 0)
        setMode(MODE_BUILTIN_WALK);
    if (strcmp(modeString, "builtin_crouch") == 0)
        setMode(MODE_BUILTIN_CROUCH);
    if (strcmp(modeString, "animation_playback") == 0)
        setMode(MODE_ANIMATION_PLAYBACK);
}

static void playAnimationByName(const char *name)
{
    if (!g_animation.ready || strcmp(g_animation.name, name) != 0)
    {
        return;
    }
    strncpy(g_activeAnimationName, name, MAX_NAME_LEN - 1);
    g_activeAnimationName[MAX_NAME_LEN - 1] = '\0';
    g_animation.playing = true;
    g_animation.paused = false;
    g_animation.startMs = millis();
    setMode(MODE_ANIMATION_PLAYBACK);
}

static void processCommandLine(const char *line)
{
    char type[32] = "";
    int seq = 0;
    jsonExtractString(line, "type", type, sizeof(type));
    jsonExtractInt(line, "seq", &seq);

    if (strcmp(type, "hello") == 0)
    {
        refreshCurrentState();
        sendStateMessage("hello_ack");
        sendAck(seq, "hello");
        return;
    }

    if (strcmp(type, "get_state") == 0)
    {
        refreshCurrentState();
        sendStateMessage("state");
        sendAck(seq, "state");
        return;
    }

    if (strcmp(type, "set_mode") == 0)
    {
        char modeString[32] = "";
        if (!jsonExtractString(line, "mode", modeString, sizeof(modeString)))
        {
            sendError(seq, "set_mode missing mode");
            return;
        }
        handleModeCommand(modeString);
        sendAck(seq, "mode updated");
        return;
    }

    if (strcmp(type, "set_servo_update_rate_hz") == 0)
    {
        float hz = DEFAULT_PCA9685_FREQUENCY_HZ;
        if (!jsonExtractFloat(line, "hz", &hz))
        {
            sendError(seq, "set_servo_update_rate_hz missing hz");
            return;
        }

        if (hz < MIN_PCA9685_FREQUENCY_HZ || hz > MAX_PCA9685_FREQUENCY_HZ)
        {
            sendError(seq, "servo update rate must be between 40 and 200 hz");
            return;
        }

        applyPca9685Frequency(hz);
        sendAck(seq, "servo update rate updated");
        return;
    }

    if (strcmp(type, "release_servos") == 0)
    {
        releaseAllServos();
        sendAck(seq, "servos released");
        return;
    }

    if (strcmp(type, "benchmark_solver") == 0)
    {
        int iterations = 200;
        jsonExtractInt(line, "iterations", &iterations);
        if (iterations <= 0 || iterations > 2000)
        {
            sendError(seq, "benchmark iterations must be between 1 and 2000");
            return;
        }

        SolverTimingStats footStats;
        SolverTimingStats jointStats;
        runSolverBenchmark((uint32_t)iterations, &footStats, &jointStats);
        sendBenchmarkResult(seq, (uint32_t)iterations, &footStats, &jointStats);
        sendAck(seq, "solver benchmark complete");
        return;
    }

    if (strcmp(type, "run_builtin") == 0)
    {
        char name[MAX_NAME_LEN] = "";
        if (!jsonExtractString(line, "name", name, sizeof(name)))
        {
            sendError(seq, "run_builtin missing name");
            return;
        }

        strncpy(g_builtinName, name, MAX_NAME_LEN - 1);
        g_builtinName[MAX_NAME_LEN - 1] = '\0';
        if (strcmp(name, "walk") == 0)
        {
            // Optional "intent" field: "forward" | "backward" | "turn_left" | "turn_right" | "stop"
            char intentStr[16] = "forward";
            jsonExtractString(line, "intent", intentStr, sizeof(intentStr));
            if (strcmp(intentStr, "backward") == 0)
                g_walkIntent = WALK_BACKWARD;
            else if (strcmp(intentStr, "turn_left") == 0)
                g_walkIntent = WALK_TURN_LEFT;
            else if (strcmp(intentStr, "turn_right") == 0)
                g_walkIntent = WALK_TURN_RIGHT;
            else if (strcmp(intentStr, "stop") == 0)
                g_walkIntent = WALK_STOP;
            else
                g_walkIntent = WALK_FORWARD;
            setMode(MODE_BUILTIN_WALK);
            sendAck(seq, "builtin walk");
            return;
        }

        if (strcmp(name, "crouch") == 0)
        {
            setMode(MODE_BUILTIN_CROUCH);
            sendAck(seq, "builtin crouch");
            return;
        }

        sendError(seq, "unknown builtin");
        return;
    }

    if (strcmp(type, "set_leg_foot_xy") == 0)
    {
        char legId[24] = "";
        float x = 0.0f;
        float y = 0.0f;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "x", &x) || !jsonExtractFloat(line, "y", &y))
        {
            sendError(seq, "foot command missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0)
        {
            sendError(seq, "unknown leg");
            return;
        }

        setMode(MODE_DIRECT_FOOT_XY);
        if (!robotLegCommandFoot(legIndex, x, y))
        {
            sendError(seq, g_legs[legIndex].lastError);
            return;
        }
        sendAck(seq, "foot target accepted");
        return;
    }

    if (strcmp(type, "set_leg_joint_angles") == 0)
    {
        char legId[24] = "";
        float thighDeg = 0.0f;
        float calfDeg = 0.0f;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "thighDeg", &thighDeg) || !jsonExtractFloat(line, "calfDeg", &calfDeg))
        {
            sendError(seq, "joint command missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0)
        {
            sendError(seq, "unknown leg");
            return;
        }

        setMode(MODE_DIRECT_JOINT_ANGLES);
        if (!robotLegCommandJointAngles(legIndex, thighDeg, calfDeg))
        {
            sendError(seq, g_legs[legIndex].lastError);
            return;
        }
        sendAck(seq, "joint target accepted");
        return;
    }

    if (strcmp(type, "set_leg_servo_angles") == 0)
    {
        char legId[24] = "";
        float hipServoDeg = 90.0f;
        float thighServoDeg = 0.0f;
        float calfServoDeg = 0.0f;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "thighServoDeg", &thighServoDeg) || !jsonExtractFloat(line, "calfServoDeg", &calfServoDeg))
        {
            sendError(seq, "servo command missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0)
        {
            sendError(seq, "unknown leg");
            return;
        }

        hipServoDeg = g_legs[legIndex].desiredServoAngles.hip;
        jsonExtractFloat(line, "hipServoDeg", &hipServoDeg);
        setMode(MODE_DIRECT_SERVO_ANGLES);
        robotLegCommandServo(legIndex, hipServoDeg, thighServoDeg, calfServoDeg);
        sendAck(seq, "servo target accepted");
        return;
    }

    if (strcmp(type, "set_leg_servo_channel_map") == 0)
    {
        char legId[24] = "";
        int hipChannel = 0;
        int thighChannel = 0;
        int calfChannel = 0;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractInt(line, "thighChannel", &thighChannel) || !jsonExtractInt(line, "calfChannel", &calfChannel))
        {
            sendError(seq, "servo channel map missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0)
        {
            sendError(seq, "unknown leg");
            return;
        }

        hipChannel = g_legs[legIndex].hipChannel;
        jsonExtractInt(line, "hipChannel", &hipChannel);

        if (hipChannel < 0 || hipChannel > 15 || thighChannel < 0 || thighChannel > 15 || calfChannel < 0 || calfChannel > 15)
        {
            sendError(seq, "servo channels must be between 0 and 15");
            return;
        }

        robotLegSetServoChannels(legIndex, (uint8_t)hipChannel, (uint8_t)thighChannel, (uint8_t)calfChannel);
        sendAck(seq, "servo channel map updated");
        return;
    }

    if (strcmp(type, "set_leg_joint_limits") == 0)
    {
        char legId[24] = "";
        float thighMinDeg = DEFAULT_THIGH_MIN_DEG;
        float thighMaxDeg = DEFAULT_THIGH_MAX_DEG;
        float calfMinDeg = DEFAULT_CALF_MIN_DEG;
        float calfMaxDeg = DEFAULT_CALF_MAX_DEG;
        if (
            !jsonExtractString(line, "legId", legId, sizeof(legId)) ||
            !jsonExtractFloat(line, "thighMinDeg", &thighMinDeg) ||
            !jsonExtractFloat(line, "thighMaxDeg", &thighMaxDeg) ||
            !jsonExtractFloat(line, "calfMinDeg", &calfMinDeg) ||
            !jsonExtractFloat(line, "calfMaxDeg", &calfMaxDeg))
        {
            sendError(seq, "joint limit command missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0)
        {
            sendError(seq, "unknown leg");
            return;
        }

        JointLimitsDeg limits;
        limits.thighDeg.min = thighMinDeg;
        limits.thighDeg.max = thighMaxDeg;
        limits.calfDeg.min = calfMinDeg;
        limits.calfDeg.max = calfMaxDeg;
        robotLegSetJointLimits(legIndex, limits);
        sendAck(seq, "joint limits updated");
        return;
    }

    if (strcmp(type, "set_leg_servo_speed_limit") == 0)
    {
        char legId[24] = "";
        float hipDegPerSec = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
        float thighDegPerSec = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
        float calfDegPerSec = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
        if (
            !jsonExtractString(line, "legId", legId, sizeof(legId)) ||
            !jsonExtractFloat(line, "thighDegPerSec", &thighDegPerSec) ||
            !jsonExtractFloat(line, "calfDegPerSec", &calfDegPerSec))
        {
            sendError(seq, "servo speed limit command missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0)
        {
            sendError(seq, "unknown leg");
            return;
        }

        hipDegPerSec = g_legs[legIndex].servoSpeedLimitDegPerSec.hip;
        jsonExtractFloat(line, "hipDegPerSec", &hipDegPerSec);

        if (hipDegPerSec <= 0.0f || thighDegPerSec <= 0.0f || calfDegPerSec <= 0.0f)
        {
            sendError(seq, "servo speed limits must be positive");
            return;
        }

        robotLegSetServoSpeedLimit(legIndex, hipDegPerSec, thighDegPerSec, calfDegPerSec);
        sendAck(seq, "servo speed limit updated");
        return;
    }

    if (strcmp(type, "upload_animation") == 0)
    {
        char stage[16] = "";
        if (!jsonExtractString(line, "stage", stage, sizeof(stage)))
        {
            sendError(seq, "upload_animation missing stage");
            return;
        }

        if (strcmp(stage, "begin") == 0)
        {
            char name[MAX_NAME_LEN] = "";
            float duration = 1.0f;
            jsonExtractString(line, "name", name, sizeof(name));
            jsonExtractFloat(line, "duration", &duration);
            animationBeginUpload(&g_animation, name, duration);
            sendAck(seq, "animation upload begin");
            return;
        }

        if (strcmp(stage, "frame") == 0)
        {
            char legId[24] = "";
            float time = 0.0f;
            float x = 0.0f;
            float y = 0.0f;
            if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "time", &time) || !jsonExtractFloat(line, "x", &x) || !jsonExtractFloat(line, "y", &y))
            {
                sendError(seq, "animation frame missing fields");
                return;
            }

            int legIndex = legIndexFromId(legId);
            if (!animationAddFrame(&g_animation, legIndex, time, x, y))
            {
                sendError(seq, "animation frame rejected");
                return;
            }
            sendAck(seq, "animation frame");
            return;
        }

        if (strcmp(stage, "commit") == 0)
        {
            animationCommit(&g_animation);
            strncpy(g_activeAnimationName, g_animation.name, MAX_NAME_LEN - 1);
            g_activeAnimationName[MAX_NAME_LEN - 1] = '\0';
            sendAck(seq, "animation uploaded");
            return;
        }

        sendError(seq, "unknown upload stage");
        return;
    }

    if (strcmp(type, "play_animation") == 0)
    {
        char name[MAX_NAME_LEN] = "";
        if (!jsonExtractString(line, "name", name, sizeof(name)))
        {
            sendError(seq, "play_animation missing name");
            return;
        }

        if (!g_animation.ready || strcmp(g_animation.name, name) != 0)
        {
            sendError(seq, "animation not ready");
            return;
        }

        playAnimationByName(name);
        sendAck(seq, "animation playing");
        return;
    }

    if (strcmp(type, "pause_animation") == 0)
    {
        if (g_animation.playing)
        {
            g_animation.playing = false;
            g_animation.paused = true;
            g_animation.pausedTimeSec = (millis() - g_animation.startMs) / 1000.0f;
        }
        sendAck(seq, "animation paused");
        return;
    }

    if (strcmp(type, "stop_animation") == 0)
    {
        g_animation.playing = false;
        g_animation.paused = false;
        setMode(MODE_IDLE);
        sendAck(seq, "animation stopped");
        return;
    }

    // Diagnostic: include what we parsed as `type` and the first 60 chars
    // of the raw line, so the host can see whether the command came in
    // mangled (RX overflow / desync) or whether the type is genuinely
    // unrecognized.
    char detail[128];
    snprintf(detail, sizeof(detail), "unknown command type [parsed='%s' raw='%.60s']", type, line);
    sendError(seq, detail);
}

// ─── LED strip helpers ────────────────────────────────────────────────────

// Fill a contiguous range of LEDs with one colour.
static void ledFillRange(uint16_t first, uint16_t last, uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = first; i <= last; ++i)
        g_ledStrip.setPixelColor(i, g_ledStrip.Color(r, g, b));
}

// Clear (turn off) a contiguous range.
static void ledClearRange(uint16_t first, uint16_t last)
{
    ledFillRange(first, last, 0, 0, 0);
}

// Idle ring-chase effect — a bright green head orbits the underbelly with a
// short fading tail. Runs entirely off millis(); no state variables needed.
//   IDLE_RING_PERIOD_MS  — time (ms) for one full lap of the underbelly
//   IDLE_RING_TAIL       — number of LEDs in the fading tail (excluding head)
static const uint32_t IDLE_RING_PERIOD_MS = 1200;
static const uint16_t IDLE_RING_TAIL = 6;

static void ledIdleRing(void)
{
    uint32_t nowMs = millis();
    uint16_t len = BELLY_LAST - BELLY_FIRST + 1; // 40 LEDs
    // Head position: advances smoothly around the ring
    uint16_t head = (uint16_t)((nowMs % IDLE_RING_PERIOD_MS) * len / IDLE_RING_PERIOD_MS);

    for (uint16_t i = 0; i < len; ++i)
    {
        // Distance behind the head (wrapping)
        uint16_t dist = (head >= i) ? (head - i) : (len - i + head);

        uint8_t brightness;
        if (dist == 0)
        {
            brightness = 255; // full-bright head
        }
        else if (dist <= IDLE_RING_TAIL)
        {
            // Linear fade from 180 down to ~20 across the tail
            brightness = (uint8_t)(180 - (160 * (dist - 1)) / IDLE_RING_TAIL);
        }
        else
        {
            brightness = 0; // dark
        }

        g_ledStrip.setPixelColor(BELLY_FIRST + i, g_ledStrip.Color(0, brightness, 0));
    }
}

// Called from loop() — updates the strip to match the current robot state.
// Uses millis() internally for non-blocking animation; never calls delay().
static void updateLedStrip(void)
{
    uint32_t nowMs = millis();

    bool flashOn_signal = ((nowMs / SIGNAL_FLASH_HALF_MS) & 1) == 0;

    // --- Left signal ---
    if (g_walkIntent == WALK_TURN_LEFT)
        ledFillRange(LEFT_SIG_FIRST, LEFT_SIG_LAST,
                     flashOn_signal ? SIGNAL_LEFT_R : 0,
                     flashOn_signal ? SIGNAL_LEFT_G : 0,
                     flashOn_signal ? SIGNAL_LEFT_B : 0);
    else if (g_walkIntent == WALK_BACKWARD)
        ledFillRange(LEFT_SIG_FIRST, LEFT_SIG_LAST,
                     flashOn_signal ? 255 : 0, 0, 0);
    else
        ledClearRange(LEFT_SIG_FIRST, LEFT_SIG_LAST);

    // --- Right signal ---
    if (g_walkIntent == WALK_TURN_RIGHT)
        ledFillRange(RIGHT_SIG_FIRST, RIGHT_SIG_LAST,
                     flashOn_signal ? SIGNAL_RIGHT_R : 0,
                     flashOn_signal ? SIGNAL_RIGHT_G : 0,
                     flashOn_signal ? SIGNAL_RIGHT_B : 0);
    else if (g_walkIntent == WALK_BACKWARD)
        ledFillRange(RIGHT_SIG_FIRST, RIGHT_SIG_LAST,
                     flashOn_signal ? 255 : 0, 0, 0);
    else
        ledClearRange(RIGHT_SIG_FIRST, RIGHT_SIG_LAST);

    // --- Underbelly ---
    if (g_mode == MODE_BUILTIN_CROUCH)
    {
        // Crouching → solid blue
        ledFillRange(BELLY_FIRST, BELLY_LAST, 0, 0, 255);
    }
    else if (g_mode == MODE_BUILTIN_WALK && g_walkIntent == WALK_BACKWARD)
    {
        // Moving backwards → solid red
        ledFillRange(BELLY_FIRST, BELLY_LAST, 255, 0, 0);
    }
    else if (g_mode == MODE_BUILTIN_WALK)
    {
        // Moving (forward / turn) → solid green
        ledFillRange(BELLY_FIRST, BELLY_LAST, 0, 255, 0);
    }
    else
    {
        // Idle → green ring-chase orbiting the underbelly
        ledIdleRing();
    }

    g_ledStrip.show();
}

void setup()
{
    // Buffer sizing for 921600 baud telemetry + command bursts.
    //
    // RX: the host sends 4× set_leg_servo_angles (~480 B) per gait tick.
    // TX: state telemetry is ~3.5 KB every 100 ms. With the default 0-byte
    // TX ring, Serial.print blocks for ~35 ms while the FIFO drains. During
    // that block the RX driver task is starved, the 128-byte UART FIFO
    // overflows, and a chunk in the middle of the 4th inbound command goes
    // missing — observed as `{"t...servo_angles"`. Sizing TX large enough
    // to swallow a whole telemetry frame makes Serial.print return
    // immediately and keeps the RX task fed.
#if defined(ESP32)
    Serial.setRxBufferSize(4096);
    Serial.setTxBufferSize(8192);
#endif
    Serial.begin(SERIAL_BAUD);
    unsigned long serialStartMs = millis();
    while (!Serial && (millis() - serialStartMs) < 5000)
    {
        delay(10);
    }
    delay(100);

#if defined(ESP32)
    Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
#endif
    Wire.begin();

    // Boot-time scan: print every I2C address that ACKs so a missing
    // chip is instantly diagnosable from the host log without unwiring.
    scanI2cBus();

    if (!i2cDevicePresent(PCA9685_ADDRESS))
    {
        Serial.print("PCA9685 not found on I2C at 0x");
        Serial.println(PCA9685_ADDRESS, HEX);
        Serial.println("Check SDA/SCL wiring, board power, and address pins.");
        while (true)
        {
            delay(1000);
        }
    }

    // LSM9DS0 IMU on the same I2C bus — non-fatal if missing.
    g_imuPresent = imuInit();
    if (!g_imuPresent)
    {
        Serial.println("LSM9DS0 not detected (skipping IMU; gait stabilizer will run un-fused)");
        Serial.println("  expected addresses: 0x1D + 0x6B (SDO HIGH) or 0x1E + 0x6A (SDO LOW)");
        Serial.println("  see [i2c-scan] line above for what's actually on the bus");
    }

    g_pwm.begin();
    applyPca9685Frequency(DEFAULT_PCA9685_FREQUENCY_HZ);
    robotDogLegInitNeutral();
    animationReset(&g_animation);

    for (int i = 0; i < NUM_LEGS; ++i)
    {
        robotLegBegin(&g_legs[i], &LEG_CONFIGS[i]);
        robotLegCommandFoot(i, 0.0f, 0.0f);
    }

    g_lastTelemetryMs = millis();
    g_lastBuiltinStatusMs = millis();
    g_lastAnimationStatusMs = millis();
    sendStateMessage("state");

    // Initialise LED strip and run a boot self-test — flashes every
    // pixel R, G, B, W in turn at full brightness so an operator can
    // confirm the strip is wired and powered correctly. If this
    // sequence shows nothing on the strip, the issue is hardware
    // (wiring, 3V3-vs-5V data level, wrong strip type, brown-out, dead
    // strip) and not firmware logic. If this shows colors but the
    // normal updateLedStrip() pattern doesn't, the issue is downstream
    // in the per-mode LED logic.
    g_ledStrip.begin();
    g_ledStrip.setBrightness(255);  // full brightness for self-test
    Serial.println("[led] self-test: R -> G -> B -> W (2 s)");
    const uint32_t selfTestColors[4] = {
        g_ledStrip.Color(255, 0, 0),  // red
        g_ledStrip.Color(0, 255, 0),  // green
        g_ledStrip.Color(0, 0, 255),  // blue
        g_ledStrip.Color(255, 255, 255), // white
    };
    for (int phase = 0; phase < 4; ++phase) {
        for (uint16_t i = 0; i < NEOPIXEL_COUNT; ++i) {
            g_ledStrip.setPixelColor(i, selfTestColors[phase]);
        }
        g_ledStrip.show();
        delay(500);
    }
    g_ledStrip.setBrightness(NEOPIXEL_BRIGHTNESS);
    g_ledStrip.clear();
    g_ledStrip.show();
}

void loop()
{
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();
        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            g_serialBuffer[g_serialIndex] = '\0';
            if (g_serialIndex > 0)
            {
                processCommandLine(g_serialBuffer);
            }
            g_serialIndex = 0;
            continue;
        }

        if (g_serialIndex < (SERIAL_BUFFER_SIZE - 1))
        {
            g_serialBuffer[g_serialIndex++] = c;
        }
    }

    runBuiltinsAndAnimation();

    for (int i = 0; i < NUM_LEGS; ++i)
    {
        robotLegUpdate(&g_legs[i]);
    }

    updateLedStrip();

    maybeSendPeriodicEvents();
}
