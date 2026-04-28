#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
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
#define SERIAL_BAUD 460800
#define SERIAL_BUFFER_SIZE 320
#define MAX_NAME_LEN 32
#define MAX_ERROR_LEN 96
#define MAX_ANIM_KEYFRAMES 32
#define TELEMETRY_INTERVAL_MS 250
#define BUILTIN_STATUS_INTERVAL_MS 400
#define ANIMATION_STATUS_INTERVAL_MS 250
#define STREAMING_ACK_INTERVAL_MS 500

static const float SERVO_SUPPLY_VOLTS = 7.4f;
static const float SERVO_SPEED_SAFETY_FACTOR = 0.80f;
static const float MOTION_TIME_SCALE = 1.15f;
static const float SERVO_MIN_MOVE_DURATION_SEC = 0.075f;
static const float SERVO_TARGET_EPSILON_DEG = 0.05f;
static const uint32_t SERVO_STREAMING_LINEAR_DURATION_US = 120000;
static const float DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC = 180.0f;
static const float HIP_YAW_CENTER_DEG = 90.0f;
static const float THIGH_CENTER_DEG = 90.0f;
static const float CALF_CENTER_DEG = 90.0f;
static const float LEFT_HIP_YAW_SIGN = 1.0f;
static const float RIGHT_HIP_YAW_SIGN = -1.0f;
static const float THIGH_SIGN = 1.0f;
static const float CALF_SIGN = 1.0f;
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float DEFAULT_HIP_YAW_MIN_DEG = -35.0f;
static const float DEFAULT_HIP_YAW_MAX_DEG = 35.0f;
static const float DEFAULT_THIGH_MIN_DEG = -145.0f;
static const float DEFAULT_THIGH_MAX_DEG = 15.0f;
static const float DEFAULT_CALF_MIN_DEG = -165.0f;
static const float DEFAULT_CALF_MAX_DEG = -25.0f;
static const uint8_t PCA9685_ADDRESS = 0x40;
static const float PCA9685_FREQUENCY_HZ = 150.0f;
static const char MICROCONTROLLER_BOARD_NAME[] = "Adafruit ESP32 Feather";
static const uint8_t I2C_SDA_PIN = 23;
static const uint8_t I2C_SCL_PIN = 22;
static const float SERVO_MIN_PULSE_US = 500.0f;
static const float SERVO_MAX_PULSE_US = 2500.0f;

typedef struct {
    float min;
    float max;
} JointLimitRangeDeg;

typedef struct {
    JointLimitRangeDeg hipYawDeg;
    JointLimitRangeDeg thighDeg;
    JointLimitRangeDeg calfDeg;
} JointLimitsDeg;

typedef struct {
    float hipYaw;
    float thigh;
    float calf;
} ServoAnglesDeg;

typedef struct {
    const char *id;
    uint8_t hipYawChannel;
    uint8_t thighChannel;
    uint8_t calfChannel;
    float hipYawSign;
} LegHardwareConfig;

typedef struct {
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

typedef struct {
    SmoothServo hipYaw;
    SmoothServo thigh;
    SmoothServo calf;
    ServoAnglesDeg desiredServoAngles;
    ServoAnglesDeg currentServoAngles;
    uint8_t hipYawChannel;
    uint8_t thighChannel;
    uint8_t calfChannel;
    JointLimitsDeg jointLimits;
    char lastError[MAX_ERROR_LEN];
} RobotLegState;

typedef struct {
    float time;
    float x;
    float y;
    ServoAnglesDeg servoAnglesDeg;
    bool hasServoAngles;
} AnimationFrame;

typedef struct {
    AnimationFrame frames[MAX_ANIM_KEYFRAMES];
    int count;
} LegTrack;

typedef struct {
    char name[MAX_NAME_LEN];
    float duration;
    LegTrack tracks[NUM_LEGS];
    bool ready;
    bool playing;
    bool paused;
    uint32_t startMs;
    float pausedTimeSec;
} AnimationClip;

typedef enum {
    MODE_IDLE,
    MODE_DIRECT_SERVO_ANGLES,
    MODE_ANIMATION_PLAYBACK
} RobotMode;

static const LegHardwareConfig LEG_CONFIGS[NUM_LEGS] = {
    {"front_left", 8, 0, 1, LEFT_HIP_YAW_SIGN},
    {"front_right", 9, 2, 3, RIGHT_HIP_YAW_SIGN},
    {"rear_left", 10, 4, 5, LEFT_HIP_YAW_SIGN},
    {"rear_right", 11, 6, 7, RIGHT_HIP_YAW_SIGN}
};

static Adafruit_PWMServoDriver g_pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
static RobotLegState g_legs[NUM_LEGS];
static AnimationClip g_animation;
static RobotMode g_mode = MODE_IDLE;
static char g_activeAnimationName[MAX_NAME_LEN] = "";
static char g_serialBuffer[SERIAL_BUFFER_SIZE];
static int g_serialIndex = 0;
static uint32_t g_modeStartMs = 0;
static uint32_t g_lastTelemetryMs = 0;
static uint32_t g_lastAnimationStatusMs = 0;
static uint32_t g_lastStreamingAckMs = 0;
static bool g_servosReleased = false;
static uint8_t g_activeI2cSdaPin = I2C_SDA_PIN;
static uint8_t g_activeI2cSclPin = I2C_SCL_PIN;

static void beginI2cBus(uint8_t sdaPin, uint8_t sclPin) {
#if defined(ESP32)
    Wire.end();
    Wire.setPins(sdaPin, sclPin);
#endif
    Wire.begin();
}

static bool i2cDevicePresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

static bool selectServoI2cBus(void) {
    beginI2cBus(I2C_SDA_PIN, I2C_SCL_PIN);
    if (i2cDevicePresent(PCA9685_ADDRESS)) {
        g_activeI2cSdaPin = I2C_SDA_PIN;
        g_activeI2cSclPin = I2C_SCL_PIN;
        return true;
    }

#if defined(ESP32)
    if (SDA != I2C_SDA_PIN || SCL != I2C_SCL_PIN) {
        beginI2cBus(SDA, SCL);
        if (i2cDevicePresent(PCA9685_ADDRESS)) {
            g_activeI2cSdaPin = SDA;
            g_activeI2cSclPin = SCL;
            Serial.print("PCA9685 found on board default I2C pins instead of configured pins. Using SDA=");
            Serial.print(g_activeI2cSdaPin);
            Serial.print(", SCL=");
            Serial.println(g_activeI2cSclPin);
            return true;
        }
    }
#endif

    g_activeI2cSdaPin = I2C_SDA_PIN;
    g_activeI2cSclPin = I2C_SCL_PIN;
    return false;
}

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float lerpf(float a, float b, float t) {
    return a + (b - a) * t;
}

static JointLimitsDeg normalizeJointLimits(JointLimitsDeg limits) {
    if (limits.hipYawDeg.min > limits.hipYawDeg.max) {
        float tmp = limits.hipYawDeg.min;
        limits.hipYawDeg.min = limits.hipYawDeg.max;
        limits.hipYawDeg.max = tmp;
    }

    if (limits.thighDeg.min > limits.thighDeg.max) {
        float tmp = limits.thighDeg.min;
        limits.thighDeg.min = limits.thighDeg.max;
        limits.thighDeg.max = tmp;
    }

    if (limits.calfDeg.min > limits.calfDeg.max) {
        float tmp = limits.calfDeg.min;
        limits.calfDeg.min = limits.calfDeg.max;
        limits.calfDeg.max = tmp;
    }

    return limits;
}

static JointLimitsDeg defaultJointLimits(void) {
    JointLimitsDeg limits;
    limits.hipYawDeg.min = DEFAULT_HIP_YAW_MIN_DEG;
    limits.hipYawDeg.max = DEFAULT_HIP_YAW_MAX_DEG;
    limits.thighDeg.min = DEFAULT_THIGH_MIN_DEG;
    limits.thighDeg.max = DEFAULT_THIGH_MAX_DEG;
    limits.calfDeg.min = DEFAULT_CALF_MIN_DEG;
    limits.calfDeg.max = DEFAULT_CALF_MAX_DEG;
    return limits;
}

static float servoSpeedDegPerSecFromVoltage(float volts) {
    float secPer60;
    if (volts >= 8.4f) {
        secPer60 = 0.10f;
    } else if (volts >= 7.4f) {
        float t = (volts - 7.4f) / (8.4f - 7.4f);
        secPer60 = 0.11f + t * (0.10f - 0.11f);
    } else if (volts >= 5.0f) {
        float t = (volts - 5.0f) / (7.4f - 5.0f);
        secPer60 = 0.13f + t * (0.11f - 0.13f);
    } else {
        secPer60 = 0.13f;
    }
    return 60.0f / secPer60;
}

static float easeInOutSmoothStep(float t) {
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static float applyServoCalibration(float modelDeg, float centerDeg, float sign) {
    return centerDeg + sign * (modelDeg - 90.0f);
}

static float removeServoCalibration(float rawDeg, float centerDeg, float sign) {
    if (sign == 0.0f) {
        return 90.0f;
    }
    return 90.0f + (rawDeg - centerDeg) / sign;
}

static uint16_t servoDegToPcaTicks(float servoDeg) {
    float clamped = clampf(servoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    float ratio = clamped / 180.0f;
    float pulseUs = SERVO_MIN_PULSE_US + ratio * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    float periodUs = 1000000.0f / PCA9685_FREQUENCY_HZ;
    float ticks = (pulseUs / periodUs) * 4096.0f;
    return (uint16_t)roundf(ticks);
}

static float smoothServoReadEstimate(const SmoothServo *s);

static void smoothServoRelease(SmoothServo *s) {
    s->estimatedDeg = smoothServoReadEstimate(s);
    s->startDeg = s->estimatedDeg;
    s->targetDeg = s->estimatedDeg;
    s->moveDurationUs = 1;
    s->moving = false;
    if (s->channel != INVALID_CHANNEL) {
        g_pwm.setPWM(s->channel, 0, 4096);
    }
    s->attached = false;
}

static void smoothServoWriteRaw(SmoothServo *s, float rawDeg) {
    if (s->attached) {
        g_pwm.setPWM(s->channel, 0, servoDegToPcaTicks(rawDeg));
    }
}

static void smoothServoWriteEstimate(SmoothServo *s) {
    smoothServoWriteRaw(s, smoothServoReadEstimate(s));
}

static float smoothServoEffectiveMaxSpeedDegPerSec(const SmoothServo *s) {
    float limit = s->speedLimitDegPerSec;
    if (limit <= 0.0f) {
        limit = s->hardwareMaxSpeedDegPerSec;
    }
    if (limit > s->hardwareMaxSpeedDegPerSec) {
        limit = s->hardwareMaxSpeedDegPerSec;
    }
    if (limit < 1.0f) {
        limit = 1.0f;
    }
    return limit;
}

static void smoothServoBegin(SmoothServo *s, uint8_t channel, float initialDeg, float minDeg, float maxDeg, float centerDeg, float sign, float supplyVolts) {
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

static float smoothServoReadEstimate(const SmoothServo *s) {
    if (!s->moving) {
        return s->estimatedDeg;
    }

    uint32_t nowUs = micros();
    uint32_t elapsedUs = nowUs - s->moveStartUs;
    if (elapsedUs >= s->moveDurationUs) {
        return s->targetDeg;
    }

    float t = (float)elapsedUs / (float)s->moveDurationUs;
    float eased = (s->moveDurationUs <= SERVO_STREAMING_LINEAR_DURATION_US) ? t : easeInOutSmoothStep(t);
    return lerpf(s->startDeg, s->targetDeg, eased);
}

static void smoothServoCommandRaw(SmoothServo *s, float rawTargetDeg) {
    if (s->channel != INVALID_CHANNEL) {
        s->attached = true;
    }
    rawTargetDeg = clampf(rawTargetDeg, s->minDeg, s->maxDeg);
    if (s->moving && fabsf(rawTargetDeg - s->targetDeg) <= SERVO_TARGET_EPSILON_DEG) {
        return;
    }

    float currentEstimate = smoothServoReadEstimate(s);
    s->estimatedDeg = currentEstimate;
    s->startDeg = currentEstimate;
    s->targetDeg = rawTargetDeg;
    s->moveStartUs = micros();

    float delta = fabsf(s->targetDeg - s->startDeg);
    float durationSec = 0.0f;
    float effectiveMaxSpeed = smoothServoEffectiveMaxSpeedDegPerSec(s);
    if (effectiveMaxSpeed > 0.0f) {
        durationSec = delta / effectiveMaxSpeed;
    }
    durationSec *= MOTION_TIME_SCALE;

    if (delta > 0.1f && durationSec < SERVO_MIN_MOVE_DURATION_SEC) {
        durationSec = SERVO_MIN_MOVE_DURATION_SEC;
    }

    if (delta <= 0.01f) {
        s->moveDurationUs = 1;
        s->moving = false;
        s->estimatedDeg = s->targetDeg;
        smoothServoWriteEstimate(s);
        return;
    }

    s->moveDurationUs = (uint32_t)(durationSec * 1000000.0f);
    if (s->moveDurationUs < 1000) {
        s->moveDurationUs = 1000;
    }

    s->moving = true;
    smoothServoWriteEstimate(s);
}

static void smoothServoCommandModelDeg(SmoothServo *s, float modelDeg) {
    float raw = applyServoCalibration(modelDeg, s->centerDeg, s->sign);
    smoothServoCommandRaw(s, raw);
}

static void smoothServoSetSpeedLimitDegPerSec(SmoothServo *s, float speedLimitDegPerSec) {
    s->speedLimitDegPerSec = speedLimitDegPerSec;
    if (!s->attached) {
        return;
    }

    float currentEstimate = smoothServoReadEstimate(s);
    s->estimatedDeg = currentEstimate;
    s->startDeg = currentEstimate;
    s->moveStartUs = micros();

    float delta = fabsf(s->targetDeg - s->startDeg);
    if (delta <= 0.01f) {
        s->moveDurationUs = 1;
        s->moving = false;
        s->estimatedDeg = s->targetDeg;
        smoothServoWriteEstimate(s);
        return;
    }

    float durationSec = delta / smoothServoEffectiveMaxSpeedDegPerSec(s);
    durationSec *= MOTION_TIME_SCALE;
    if (durationSec < 0.04f) {
        durationSec = 0.04f;
    }

    s->moveDurationUs = (uint32_t)(durationSec * 1000000.0f);
    if (s->moveDurationUs < 1000) {
        s->moveDurationUs = 1000;
    }
    s->moving = true;
    smoothServoWriteEstimate(s);
}

static void smoothServoUpdate(SmoothServo *s) {
    float est = smoothServoReadEstimate(s);
    s->estimatedDeg = est;

    if (s->moving) {
        uint32_t nowUs = micros();
        if ((nowUs - s->moveStartUs) >= s->moveDurationUs) {
            s->estimatedDeg = s->targetDeg;
            s->moving = false;
        }
    }
    smoothServoWriteEstimate(s);
}

static ServoAnglesDeg getCurrentServoAngles(const RobotLegState *leg) {
    ServoAnglesDeg result;
    result.hipYaw = removeServoCalibration(leg->hipYaw.estimatedDeg, leg->hipYaw.centerDeg, leg->hipYaw.sign);
    result.thigh = removeServoCalibration(leg->thigh.estimatedDeg, leg->thigh.centerDeg, leg->thigh.sign);
    result.calf = removeServoCalibration(leg->calf.estimatedDeg, leg->calf.centerDeg, leg->calf.sign);
    return result;
}

static int legIndexFromId(const char *id) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (strcmp(id, LEG_CONFIGS[i].id) == 0) {
            return i;
        }
    }
    return -1;
}

static const char *modeToString(RobotMode mode) {
    switch (mode) {
        case MODE_IDLE: return "idle";
        case MODE_DIRECT_SERVO_ANGLES: return "direct_servo_angles";
        case MODE_ANIMATION_PLAYBACK: return "animation_playback";
        default: return "idle";
    }
}

static void setMode(RobotMode mode) {
    g_mode = mode;
    g_modeStartMs = millis();

    if (mode != MODE_ANIMATION_PLAYBACK) {
        g_animation.playing = false;
        g_animation.paused = false;
    }
}

static void clearLegError(RobotLegState *leg) {
    leg->lastError[0] = '\0';
}

static void setLegError(RobotLegState *leg, const char *message) {
    strncpy(leg->lastError, message, MAX_ERROR_LEN - 1);
    leg->lastError[MAX_ERROR_LEN - 1] = '\0';
}

static void robotLegBegin(RobotLegState *leg, const LegHardwareConfig *config) {
    smoothServoBegin(&leg->hipYaw, config->hipYawChannel, HIP_YAW_CENTER_DEG, SERVO_MIN_DEG, SERVO_MAX_DEG, HIP_YAW_CENTER_DEG, config->hipYawSign, SERVO_SUPPLY_VOLTS);
    smoothServoBegin(&leg->thigh, config->thighChannel, THIGH_CENTER_DEG, SERVO_MIN_DEG, SERVO_MAX_DEG, THIGH_CENTER_DEG, THIGH_SIGN, SERVO_SUPPLY_VOLTS);
    smoothServoBegin(&leg->calf, config->calfChannel, CALF_CENTER_DEG, SERVO_MIN_DEG, SERVO_MAX_DEG, CALF_CENTER_DEG, CALF_SIGN, SERVO_SUPPLY_VOLTS);
    leg->hipYawChannel = config->hipYawChannel;
    leg->thighChannel = config->thighChannel;
    leg->calfChannel = config->calfChannel;
    leg->jointLimits = defaultJointLimits();
    leg->desiredServoAngles.hipYaw = 90.0f;
    leg->desiredServoAngles.thigh = 90.0f;
    leg->desiredServoAngles.calf = 90.0f;
    leg->currentServoAngles = leg->desiredServoAngles;
    clearLegError(leg);
}

static void robotLegUpdate(RobotLegState *leg) {
    smoothServoUpdate(&leg->hipYaw);
    smoothServoUpdate(&leg->thigh);
    smoothServoUpdate(&leg->calf);
    leg->currentServoAngles = getCurrentServoAngles(leg);
}

static void robotLegSetServoChannels(int legIndex, uint8_t hipYawChannel, uint8_t thighChannel, uint8_t calfChannel) {
    RobotLegState *leg = &g_legs[legIndex];
    leg->hipYaw.channel = hipYawChannel;
    leg->thigh.channel = thighChannel;
    leg->calf.channel = calfChannel;
    leg->hipYaw.attached = (hipYawChannel != INVALID_CHANNEL);
    leg->thigh.attached = (thighChannel != INVALID_CHANNEL);
    leg->calf.attached = (calfChannel != INVALID_CHANNEL);
    leg->hipYawChannel = hipYawChannel;
    leg->thighChannel = thighChannel;
    leg->calfChannel = calfChannel;
    smoothServoWriteEstimate(&leg->hipYaw);
    smoothServoWriteEstimate(&leg->thigh);
    smoothServoWriteEstimate(&leg->calf);
}

static void releaseAllServos(void) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        smoothServoRelease(&g_legs[i].hipYaw);
        smoothServoRelease(&g_legs[i].thigh);
        smoothServoRelease(&g_legs[i].calf);
    }
    g_activeAnimationName[0] = '\0';
    setMode(MODE_IDLE);
    g_servosReleased = true;
}

static void robotLegSetJointLimits(int legIndex, JointLimitsDeg limits) {
    RobotLegState *leg = &g_legs[legIndex];
    leg->jointLimits = normalizeJointLimits(limits);
}

static void robotLegSetServoSpeedLimit(int legIndex, float hipYawDegPerSec, float thighDegPerSec, float calfDegPerSec) {
    RobotLegState *leg = &g_legs[legIndex];
    smoothServoSetSpeedLimitDegPerSec(&leg->hipYaw, hipYawDegPerSec);
    smoothServoSetSpeedLimitDegPerSec(&leg->thigh, thighDegPerSec);
    smoothServoSetSpeedLimitDegPerSec(&leg->calf, calfDegPerSec);
}

static void robotLegCommandServo(int legIndex, float hipYawServoDeg, float thighServoDeg, float calfServoDeg) {
    RobotLegState *leg = &g_legs[legIndex];
    g_servosReleased = false;
    smoothServoCommandModelDeg(&leg->hipYaw, hipYawServoDeg);
    smoothServoCommandModelDeg(&leg->thigh, thighServoDeg);
    smoothServoCommandModelDeg(&leg->calf, calfServoDeg);
    leg->desiredServoAngles.hipYaw = clampf(hipYawServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    leg->desiredServoAngles.thigh = clampf(thighServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    leg->desiredServoAngles.calf = clampf(calfServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    clearLegError(leg);
}

static void animationReset(AnimationClip *clip) {
    memset(clip, 0, sizeof(AnimationClip));
}

static void animationBeginUpload(AnimationClip *clip, const char *name, float duration) {
    animationReset(clip);
    strncpy(clip->name, name, MAX_NAME_LEN - 1);
    clip->name[MAX_NAME_LEN - 1] = '\0';
    clip->duration = clampf(duration, 0.1f, 30.0f);
}

static bool animationAddFrame(AnimationClip *clip, int legIndex, float time, float x, float y, bool hasServoAngles, ServoAnglesDeg servoAnglesDeg) {
    if (legIndex < 0 || legIndex >= NUM_LEGS) {
        return false;
    }

    LegTrack *track = &clip->tracks[legIndex];
    if (track->count >= MAX_ANIM_KEYFRAMES) {
        return false;
    }

    track->frames[track->count].time = clampf(time, 0.0f, clip->duration);
    track->frames[track->count].x = x;
    track->frames[track->count].y = y;
    track->frames[track->count].hasServoAngles = hasServoAngles;
    track->frames[track->count].servoAnglesDeg = servoAnglesDeg;
    track->count += 1;
    return true;
}

static void animationCommit(AnimationClip *clip) {
    clip->ready = true;
    clip->playing = false;
    clip->paused = false;
}

static bool interpolateTrackServoAngles(const LegTrack *track, float timeSec, ServoAnglesDeg *outServoAnglesDeg) {
    if (track->count <= 0) {
        return false;
    }

    if (timeSec <= track->frames[0].time) {
        if (!track->frames[0].hasServoAngles) {
            return false;
        }
        *outServoAnglesDeg = track->frames[0].servoAnglesDeg;
        return true;
    }

    if (timeSec >= track->frames[track->count - 1].time) {
        if (!track->frames[track->count - 1].hasServoAngles) {
            return false;
        }
        *outServoAnglesDeg = track->frames[track->count - 1].servoAnglesDeg;
        return true;
    }

    for (int i = 0; i < track->count - 1; ++i) {
        AnimationFrame a = track->frames[i];
        AnimationFrame b = track->frames[i + 1];
        if (timeSec >= a.time && timeSec <= b.time) {
            if (!a.hasServoAngles || !b.hasServoAngles) {
                return false;
            }

            float span = b.time - a.time;
            float t = (span > 0.0f) ? (timeSec - a.time) / span : 0.0f;
            outServoAnglesDeg->hipYaw = lerpf(a.servoAnglesDeg.hipYaw, b.servoAnglesDeg.hipYaw, t);
            outServoAnglesDeg->thigh = lerpf(a.servoAnglesDeg.thigh, b.servoAnglesDeg.thigh, t);
            outServoAnglesDeg->calf = lerpf(a.servoAnglesDeg.calf, b.servoAnglesDeg.calf, t);
            return true;
        }
    }

    return false;
}

static void jsonPrintEscaped(const char *value) {
    while (*value) {
        if (*value == '"' || *value == '\\') {
            Serial.print('\\');
        }
        Serial.print(*value);
        value += 1;
    }
}

static void sendAck(int seq, const char *message) {
    Serial.print("{\"type\":\"ack\",\"seq\":");
    Serial.print(seq);
    Serial.print(",\"message\":\"");
    jsonPrintEscaped(message);
    Serial.println("\"}");
}

static void sendError(int seq, const char *message) {
    Serial.print("{\"type\":\"error\",\"seq\":");
    Serial.print(seq);
    Serial.print(",\"message\":\"");
    jsonPrintEscaped(message);
    Serial.println("\"}");
}

static void sendAnimationProgress(void) {
    float elapsedSec = 0.0f;
    if (g_animation.playing && g_animation.duration > 0.0f) {
        elapsedSec = fmodf((millis() - g_animation.startMs) / 1000.0f, g_animation.duration);
    } else if (g_animation.paused) {
        elapsedSec = g_animation.pausedTimeSec;
    }

    Serial.print("{\"type\":\"animation_progress\",\"name\":\"");
    jsonPrintEscaped(g_activeAnimationName);
    Serial.print("\",\"time\":");
    Serial.print(elapsedSec, 3);
    Serial.println("}");
}

static void sendStateMessage(const char *typeName) {
    Serial.print("{\"type\":\"");
    Serial.print(typeName);
    Serial.print("\",\"payload\":{");
    Serial.print("\"mode\":\"");
    Serial.print(modeToString(g_mode));
    Serial.print("\",\"activeAnimation\":\"");
    jsonPrintEscaped(g_activeAnimationName);
    Serial.print("\",\"servosReleased\":");
    Serial.print(g_servosReleased ? "true" : "false");
    Serial.print(",\"firmwareMs\":");
    Serial.print(millis());
    Serial.print(",\"legs\":{");

    for (int i = 0; i < NUM_LEGS; ++i) {
        if (i > 0) {
            Serial.print(",");
        }

        RobotLegState *leg = &g_legs[i];
        Serial.print("\"");
        Serial.print(LEG_CONFIGS[i].id);
        Serial.print("\":{");
        Serial.print("\"status\":\"");
        Serial.print(modeToString(g_mode));
        Serial.print("\",\"lastError\":\"");
        jsonPrintEscaped(leg->lastError);
        Serial.print("\",\"servoChannelMap\":{\"hipYaw\":");
        Serial.print((int)leg->hipYawChannel);
        Serial.print(",\"thigh\":");
        Serial.print((int)leg->thighChannel);
        Serial.print(",\"calf\":");
        Serial.print((int)leg->calfChannel);
        Serial.print("},\"servoSpeedLimitDegPerSec\":{\"hipYaw\":");
        Serial.print(leg->hipYaw.speedLimitDegPerSec, 3);
        Serial.print(",\"thigh\":");
        Serial.print(leg->thigh.speedLimitDegPerSec, 3);
        Serial.print(",\"calf\":");
        Serial.print(leg->calf.speedLimitDegPerSec, 3);
        Serial.print("},\"jointLimits\":{\"hipYawDeg\":{\"min\":");
        Serial.print(leg->jointLimits.hipYawDeg.min, 3);
        Serial.print(",\"max\":");
        Serial.print(leg->jointLimits.hipYawDeg.max, 3);
        Serial.print("},\"thighDeg\":{\"min\":");
        Serial.print(leg->jointLimits.thighDeg.min, 3);
        Serial.print(",\"max\":");
        Serial.print(leg->jointLimits.thighDeg.max, 3);
        Serial.print("},\"calfDeg\":{\"min\":");
        Serial.print(leg->jointLimits.calfDeg.min, 3);
        Serial.print(",\"max\":");
        Serial.print(leg->jointLimits.calfDeg.max, 3);
        Serial.print("}},\"desired\":{");
        Serial.print("\"servoAnglesDeg\":{\"hipYaw\":");
        Serial.print(leg->desiredServoAngles.hipYaw, 3);
        Serial.print(",\"thigh\":");
        Serial.print(leg->desiredServoAngles.thigh, 3);
        Serial.print(",\"calf\":");
        Serial.print(leg->desiredServoAngles.calf, 3);
        Serial.print("}},\"current\":{");
        Serial.print("\"servoAnglesDeg\":{\"hipYaw\":");
        Serial.print(leg->currentServoAngles.hipYaw, 3);
        Serial.print(",\"thigh\":");
        Serial.print(leg->currentServoAngles.thigh, 3);
        Serial.print(",\"calf\":");
        Serial.print(leg->currentServoAngles.calf, 3);
        Serial.print("}}}");
    }

    Serial.println("}}}");
}

static bool jsonExtractString(const char *json, const char *key, char *out, size_t outSize) {
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *start = strstr(json, pattern);
    if (!start) {
        return false;
    }
    start += strlen(pattern);
    const char *end = strchr(start, '"');
    if (!end) {
        return false;
    }
    size_t len = (size_t)(end - start);
    if (len >= outSize) {
        len = outSize - 1;
    }
    memcpy(out, start, len);
    out[len] = '\0';
    return true;
}

static bool jsonExtractFloat(const char *json, const char *key, float *outValue) {
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *start = strstr(json, pattern);
    if (!start) {
        return false;
    }
    start += strlen(pattern);
    *outValue = strtof(start, NULL);
    return true;
}

static bool jsonExtractInt(const char *json, const char *key, int *outValue) {
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *start = strstr(json, pattern);
    if (!start) {
        return false;
    }
    start += strlen(pattern);
    *outValue = (int)strtol(start, NULL, 10);
    return true;
}

static void refreshCurrentState(void) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        g_legs[i].currentServoAngles = getCurrentServoAngles(&g_legs[i]);
    }
}

static void runBuiltinsAndAnimation(void) {
    if (g_mode == MODE_ANIMATION_PLAYBACK && g_animation.ready && g_animation.playing && !g_animation.paused) {
        float timeSec = fmodf((millis() - g_animation.startMs) / 1000.0f, g_animation.duration);
        for (int i = 0; i < NUM_LEGS; ++i) {
            ServoAnglesDeg servoAnglesDeg;
            if (interpolateTrackServoAngles(&g_animation.tracks[i], timeSec, &servoAnglesDeg)) {
                robotLegCommandServo(i, servoAnglesDeg.hipYaw, servoAnglesDeg.thigh, servoAnglesDeg.calf);
            }
        }
    }
}

static void maybeSendPeriodicEvents(void) {
    uint32_t nowMs = millis();
    if ((nowMs - g_lastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
        g_lastTelemetryMs = nowMs;
        refreshCurrentState();
        sendStateMessage("state");
    }

    if (g_mode == MODE_ANIMATION_PLAYBACK && (nowMs - g_lastAnimationStatusMs) >= ANIMATION_STATUS_INTERVAL_MS) {
        g_lastAnimationStatusMs = nowMs;
        sendAnimationProgress();
    }
}

static void handleModeCommand(const char *modeString) {
    if (strcmp(modeString, "idle") == 0) setMode(MODE_IDLE);
    if (strcmp(modeString, "direct_servo_angles") == 0) setMode(MODE_DIRECT_SERVO_ANGLES);
    if (strcmp(modeString, "animation_playback") == 0) setMode(MODE_ANIMATION_PLAYBACK);
}

static void playAnimationByName(const char *name) {
    if (!g_animation.ready || strcmp(g_animation.name, name) != 0) {
        return;
    }
    strncpy(g_activeAnimationName, name, MAX_NAME_LEN - 1);
    g_activeAnimationName[MAX_NAME_LEN - 1] = '\0';
    g_animation.playing = true;
    g_animation.paused = false;
    g_animation.startMs = millis();
    setMode(MODE_ANIMATION_PLAYBACK);
}

static void processCommandLine(const char *line) {
    char type[32] = "";
    int seq = 0;
    jsonExtractString(line, "type", type, sizeof(type));
    jsonExtractInt(line, "seq", &seq);

    if (strcmp(type, "hello") == 0) {
        refreshCurrentState();
        sendStateMessage("hello_ack");
        sendAck(seq, "hello");
        return;
    }

    if (strcmp(type, "get_state") == 0) {
        refreshCurrentState();
        sendStateMessage("state");
        sendAck(seq, "state");
        return;
    }

    if (strcmp(type, "set_mode") == 0) {
        char modeString[32] = "";
        if (!jsonExtractString(line, "mode", modeString, sizeof(modeString))) {
            sendError(seq, "set_mode missing mode");
            return;
        }
        handleModeCommand(modeString);
        sendAck(seq, "mode updated");
        return;
    }

    if (strcmp(type, "release_servos") == 0) {
        releaseAllServos();
        sendAck(seq, "servos released");
        return;
    }

    if (strcmp(type, "run_builtin") == 0) {
        sendError(seq, "host must send servo angles for builtins");
        return;
    }

    if (strcmp(type, "set_leg_foot_xy") == 0) {
        sendError(seq, "host must convert foot targets to servo angles");
        return;
    }

    if (strcmp(type, "set_leg_joint_angles") == 0) {
        sendError(seq, "host must convert joint targets to servo angles");
        return;
    }

    if (strcmp(type, "set_leg_servo_angles") == 0) {
        char legId[24] = "";
        float hipYawServoDeg = 90.0f;
        float thighServoDeg = 0.0f;
        float calfServoDeg = 0.0f;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "thighServoDeg", &thighServoDeg) || !jsonExtractFloat(line, "calfServoDeg", &calfServoDeg)) {
            sendError(seq, "servo command missing fields");
            return;
        }
        jsonExtractFloat(line, "hipYawServoDeg", &hipYawServoDeg);

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0) {
            sendError(seq, "unknown leg");
            return;
        }

        setMode(MODE_DIRECT_SERVO_ANGLES);
        robotLegCommandServo(legIndex, hipYawServoDeg, thighServoDeg, calfServoDeg);
        sendAck(seq, "servo target accepted");
        return;
    }

    if (strcmp(type, "set_leg_servo_channel_map") == 0) {
        char legId[24] = "";
        int hipYawChannel = 0;
        int thighChannel = 0;
        int calfChannel = 0;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractInt(line, "thighChannel", &thighChannel) || !jsonExtractInt(line, "calfChannel", &calfChannel)) {
            sendError(seq, "servo channel map missing fields");
            return;
        }
        jsonExtractInt(line, "hipYawChannel", &hipYawChannel);

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0) {
            sendError(seq, "unknown leg");
            return;
        }

        if (hipYawChannel < 0 || hipYawChannel > 15 || thighChannel < 0 || thighChannel > 15 || calfChannel < 0 || calfChannel > 15) {
            sendError(seq, "servo channels must be between 0 and 15");
            return;
        }

        robotLegSetServoChannels(legIndex, (uint8_t)hipYawChannel, (uint8_t)thighChannel, (uint8_t)calfChannel);
        sendAck(seq, "servo channel map updated");
        return;
    }

    if (strcmp(type, "set_leg_joint_limits") == 0) {
        char legId[24] = "";
        float hipYawMinDeg = -35.0f;
        float hipYawMaxDeg = 35.0f;
        float thighMinDeg = DEFAULT_THIGH_MIN_DEG;
        float thighMaxDeg = DEFAULT_THIGH_MAX_DEG;
        float calfMinDeg = DEFAULT_CALF_MIN_DEG;
        float calfMaxDeg = DEFAULT_CALF_MAX_DEG;
        if (
            !jsonExtractString(line, "legId", legId, sizeof(legId)) ||
            !jsonExtractFloat(line, "thighMinDeg", &thighMinDeg) ||
            !jsonExtractFloat(line, "thighMaxDeg", &thighMaxDeg) ||
            !jsonExtractFloat(line, "calfMinDeg", &calfMinDeg) ||
            !jsonExtractFloat(line, "calfMaxDeg", &calfMaxDeg)
        ) {
            sendError(seq, "joint limit command missing fields");
            return;
        }
        jsonExtractFloat(line, "hipYawMinDeg", &hipYawMinDeg);
        jsonExtractFloat(line, "hipYawMaxDeg", &hipYawMaxDeg);

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0) {
            sendError(seq, "unknown leg");
            return;
        }

        JointLimitsDeg limits;
        limits.hipYawDeg.min = hipYawMinDeg;
        limits.hipYawDeg.max = hipYawMaxDeg;
        limits.thighDeg.min = thighMinDeg;
        limits.thighDeg.max = thighMaxDeg;
        limits.calfDeg.min = calfMinDeg;
        limits.calfDeg.max = calfMaxDeg;
        robotLegSetJointLimits(legIndex, limits);
        sendAck(seq, "joint limits updated");
        return;
    }

    if (strcmp(type, "set_leg_servo_speed_limit") == 0) {
        char legId[24] = "";
        float hipYawDegPerSec = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
        float thighDegPerSec = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
        float calfDegPerSec = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
        if (
            !jsonExtractString(line, "legId", legId, sizeof(legId)) ||
            !jsonExtractFloat(line, "thighDegPerSec", &thighDegPerSec) ||
            !jsonExtractFloat(line, "calfDegPerSec", &calfDegPerSec)
        ) {
            sendError(seq, "servo speed limit command missing fields");
            return;
        }
        jsonExtractFloat(line, "hipYawDegPerSec", &hipYawDegPerSec);

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0) {
            sendError(seq, "unknown leg");
            return;
        }

        if (hipYawDegPerSec <= 0.0f || thighDegPerSec <= 0.0f || calfDegPerSec <= 0.0f) {
            sendError(seq, "servo speed limits must be positive");
            return;
        }

        robotLegSetServoSpeedLimit(legIndex, hipYawDegPerSec, thighDegPerSec, calfDegPerSec);
        sendAck(seq, "servo speed limit updated");
        return;
    }

    if (strcmp(type, "apply_full_body_pose") == 0) {
        const char *prefixes[NUM_LEGS] = {"FL", "FR", "RL", "RR"};
        float hipYawServoDeg = 90.0f;
        float thighServoDeg = 90.0f;
        float calfServoDeg = 90.0f;

        for (int legIndex = 0; legIndex < NUM_LEGS; ++legIndex) {
            char hipYawKey[32];
            char thighKey[32];
            char calfKey[32];
            snprintf(hipYawKey, sizeof(hipYawKey), "%sHipYawDeg", prefixes[legIndex]);
            snprintf(thighKey, sizeof(thighKey), "%sThighDeg", prefixes[legIndex]);
            snprintf(calfKey, sizeof(calfKey), "%sCalfDeg", prefixes[legIndex]);

            if (
                !jsonExtractFloat(line, hipYawKey, &hipYawServoDeg) ||
                !jsonExtractFloat(line, thighKey, &thighServoDeg) ||
                !jsonExtractFloat(line, calfKey, &calfServoDeg)
            ) {
                sendError(seq, "full body pose missing servo fields");
                return;
            }

            robotLegCommandServo(legIndex, hipYawServoDeg, thighServoDeg, calfServoDeg);
        }

        uint32_t nowMs = millis();
        if ((nowMs - g_lastStreamingAckMs) >= STREAMING_ACK_INTERVAL_MS) {
            g_lastStreamingAckMs = nowMs;
            sendAck(seq, "full body pose accepted");
        }
        return;
    }

    if (strcmp(type, "upload_animation") == 0) {
        char stage[16] = "";
        if (!jsonExtractString(line, "stage", stage, sizeof(stage))) {
            sendError(seq, "upload_animation missing stage");
            return;
        }

        if (strcmp(stage, "begin") == 0) {
            char name[MAX_NAME_LEN] = "";
            float duration = 1.0f;
            jsonExtractString(line, "name", name, sizeof(name));
            jsonExtractFloat(line, "duration", &duration);
            animationBeginUpload(&g_animation, name, duration);
            sendAck(seq, "animation upload begin");
            return;
        }

        if (strcmp(stage, "frame") == 0) {
            char legId[24] = "";
            float time = 0.0f;
            float x = 0.0f;
            float y = 0.0f;
            ServoAnglesDeg servoAnglesDeg = {90.0f, 90.0f, 90.0f};
            if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "time", &time) || !jsonExtractFloat(line, "x", &x) || !jsonExtractFloat(line, "y", &y)) {
                sendError(seq, "animation frame missing fields");
                return;
            }

            bool hasHipYawServo = jsonExtractFloat(line, "hipYawServoDeg", &servoAnglesDeg.hipYaw);
            bool hasThighServo = jsonExtractFloat(line, "thighServoDeg", &servoAnglesDeg.thigh);
            bool hasCalfServo = jsonExtractFloat(line, "calfServoDeg", &servoAnglesDeg.calf);
            if ((hasHipYawServo || hasThighServo || hasCalfServo) && !(hasHipYawServo && hasThighServo && hasCalfServo)) {
                sendError(seq, "animation frame servo fields incomplete");
                return;
            }

            int legIndex = legIndexFromId(legId);
            if (!animationAddFrame(&g_animation, legIndex, time, x, y, hasHipYawServo && hasThighServo && hasCalfServo, servoAnglesDeg)) {
                sendError(seq, "animation frame rejected");
                return;
            }
            sendAck(seq, "animation frame");
            return;
        }

        if (strcmp(stage, "commit") == 0) {
            animationCommit(&g_animation);
            strncpy(g_activeAnimationName, g_animation.name, MAX_NAME_LEN - 1);
            g_activeAnimationName[MAX_NAME_LEN - 1] = '\0';
            sendAck(seq, "animation uploaded");
            return;
        }

        sendError(seq, "unknown upload stage");
        return;
    }

    if (strcmp(type, "play_animation") == 0) {
        char name[MAX_NAME_LEN] = "";
        if (!jsonExtractString(line, "name", name, sizeof(name))) {
            sendError(seq, "play_animation missing name");
            return;
        }

        if (!g_animation.ready || strcmp(g_animation.name, name) != 0) {
            sendError(seq, "animation not ready");
            return;
        }

        playAnimationByName(name);
        sendAck(seq, "animation playing");
        return;
    }

    if (strcmp(type, "pause_animation") == 0) {
        if (g_animation.playing) {
            g_animation.playing = false;
            g_animation.paused = true;
            g_animation.pausedTimeSec = (millis() - g_animation.startMs) / 1000.0f;
        }
        sendAck(seq, "animation paused");
        return;
    }

    if (strcmp(type, "stop_animation") == 0) {
        g_animation.playing = false;
        g_animation.paused = false;
        setMode(MODE_IDLE);
        sendAck(seq, "animation stopped");
        return;
    }

    sendError(seq, "unknown command type");
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    unsigned long serialStartMs = millis();
    while (!Serial && (millis() - serialStartMs) < 5000) {
        delay(10);
    }
    delay(100);

#if defined(NEOPIXEL_I2C_POWER)
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
    delay(10);
#endif
    if (!selectServoI2cBus()) {
        Serial.print("PCA9685 not found on I2C at 0x");
        Serial.println(PCA9685_ADDRESS, HEX);
        Serial.print("Configured SDA=");
        Serial.print(I2C_SDA_PIN);
        Serial.print(", SCL=");
        Serial.println(I2C_SCL_PIN);
        Serial.print("Board default SDA=");
        Serial.print(SDA);
        Serial.print(", SCL=");
        Serial.println(SCL);
        Serial.println("Check SDA/SCL wiring, board power, and address pins.");
        while (true) {
            delay(1000);
        }
    }

    g_pwm.begin();
    g_pwm.setPWMFreq(PCA9685_FREQUENCY_HZ);
    animationReset(&g_animation);

    for (int i = 0; i < NUM_LEGS; ++i) {
        robotLegBegin(&g_legs[i], &LEG_CONFIGS[i]);
        robotLegCommandServo(i, 90.0f, 90.0f, 90.0f);
    }

    Serial.print("Starting up on ");
    Serial.println(MICROCONTROLLER_BOARD_NAME);
    Serial.print("I2C SDA=");
    Serial.print(g_activeI2cSdaPin);
    Serial.print(", SCL=");
    Serial.println(g_activeI2cSclPin);
    Serial.print("PCA9685 OK at 0x");
    Serial.print(PCA9685_ADDRESS, HEX);
    Serial.print(", PWM freq = ");
    Serial.print(PCA9685_FREQUENCY_HZ, 0);
    Serial.println(" Hz");

    g_lastTelemetryMs = millis();
    g_lastAnimationStatusMs = millis();
    sendStateMessage("state");
}

void loop() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            g_serialBuffer[g_serialIndex] = '\0';
            if (g_serialIndex > 0) {
                processCommandLine(g_serialBuffer);
            }
            g_serialIndex = 0;
            continue;
        }

        if (g_serialIndex < (SERIAL_BUFFER_SIZE - 1)) {
            g_serialBuffer[g_serialIndex++] = c;
        }
    }

    runBuiltinsAndAnimation();

    for (int i = 0; i < NUM_LEGS; ++i) {
        robotLegUpdate(&g_legs[i]);
    }

    maybeSendPeriodicEvents();
}
