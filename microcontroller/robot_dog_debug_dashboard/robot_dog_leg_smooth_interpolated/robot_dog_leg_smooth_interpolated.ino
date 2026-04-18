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
#define SERIAL_BAUD 115200
#define SERIAL_BUFFER_SIZE 320
#define MAX_NAME_LEN 32
#define MAX_ERROR_LEN 96
#define MAX_ANIM_KEYFRAMES 32
#define TELEMETRY_INTERVAL_MS 100
#define BUILTIN_STATUS_INTERVAL_MS 400
#define ANIMATION_STATUS_INTERVAL_MS 250

static const float SERVO_SUPPLY_VOLTS = 7.4f;
static const float SERVO_SPEED_SAFETY_FACTOR = 0.80f;
static const float MOTION_TIME_SCALE = 1.15f;
static const float THIGH_CENTER_DEG = 90.0f;
static const float CALF_CENTER_DEG = 90.0f;
static const float THIGH_SIGN = 1.0f;
static const float CALF_SIGN = -1.0f;
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float DEFAULT_THIGH_MIN_DEG = -145.0f;
static const float DEFAULT_THIGH_MAX_DEG = 15.0f;
static const float DEFAULT_CALF_MIN_DEG = -165.0f;
static const float DEFAULT_CALF_MAX_DEG = -25.0f;
static const uint8_t PCA9685_ADDRESS = 0x40;
static const float PCA9685_FREQUENCY_HZ = 50.0f;
static const uint16_t SERVO_PWM_MIN_TICKS = 102;
static const uint16_t SERVO_PWM_MAX_TICKS = 512;

typedef struct {
    float x;
    float y;
} Point2f;

typedef struct {
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

typedef struct {
    float thetaThigh;
    float thetaServo;
    float thighServoDeg;
    float calfServoDeg;
    float footError;
    Point2f resolvedFoot;
    bool hasSolution;
    bool success;
} LegIKResult;

typedef struct {
    float thighDeg;
    float calfDeg;
} JointAnglesDeg;

typedef struct {
    float min;
    float max;
} JointLimitRangeDeg;

typedef struct {
    JointLimitRangeDeg thighDeg;
    JointLimitRangeDeg calfDeg;
} JointLimitsDeg;

typedef struct {
    float thigh;
    float calf;
} ServoAnglesDeg;

typedef struct {
    const char *id;
    uint8_t thighChannel;
    uint8_t calfChannel;
} LegHardwareConfig;

typedef struct {
    uint8_t channel;
    bool attached;
    float minDeg;
    float maxDeg;
    float centerDeg;
    float sign;
    float maxSpeedDegPerSec;
    float startDeg;
    float targetDeg;
    float estimatedDeg;
    uint32_t moveStartUs;
    uint32_t moveDurationUs;
    bool moving;
} SmoothServo;

typedef struct {
    SmoothServo thigh;
    SmoothServo calf;
    Point2f desiredFoot;
    JointAnglesDeg desiredJointAngles;
    ServoAnglesDeg desiredServoAngles;
    Point2f currentFoot;
    JointAnglesDeg currentJointAngles;
    ServoAnglesDeg currentServoAngles;
    uint8_t thighChannel;
    uint8_t calfChannel;
    JointLimitsDeg jointLimits;
    float lastThetaThigh;
    float lastThetaServo;
    char lastError[MAX_ERROR_LEN];
} RobotLegState;

typedef struct {
    float time;
    float x;
    float y;
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
    MODE_DIRECT_FOOT_XY,
    MODE_DIRECT_JOINT_ANGLES,
    MODE_DIRECT_SERVO_ANGLES,
    MODE_BUILTIN_WALK,
    MODE_BUILTIN_CROUCH,
    MODE_ANIMATION_PLAYBACK
} RobotMode;

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
    {"front_left", 0, 1},
    {"front_right", 2, 3},
    {"rear_left", 4, 5},
    {"rear_right", 6, 7}
};

static Adafruit_PWMServoDriver g_pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
static RobotLegState g_legs[NUM_LEGS];
static AnimationClip g_animation;
static RobotMode g_mode = MODE_IDLE;
static char g_builtinName[MAX_NAME_LEN] = "";
static char g_activeAnimationName[MAX_NAME_LEN] = "";
static float g_thighNeutralAngle = 0.0f;
static float g_servoNeutralAngle = 0.0f;
static bool g_neutralInitialized = false;
static char g_serialBuffer[SERIAL_BUFFER_SIZE];
static int g_serialIndex = 0;
static uint32_t g_modeStartMs = 0;
static uint32_t g_lastTelemetryMs = 0;
static uint32_t g_lastBuiltinStatusMs = 0;
static uint32_t g_lastAnimationStatusMs = 0;
static bool g_servosReleased = false;

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float lerpf(float a, float b, float t) {
    return a + (b - a) * t;
}

static float clampAngle(float angle) {
    const float twoPi = 2.0f * (float)M_PI;
    float value = fmodf(angle, twoPi);
    if (value <= -(float)M_PI) value += twoPi;
    if (value > (float)M_PI) value -= twoPi;
    return value;
}

static float radToDeg(float radians) {
    return radians * 180.0f / (float)M_PI;
}

static float degToRad(float degrees) {
    return degrees * (float)M_PI / 180.0f;
}

static JointLimitsDeg normalizeJointLimits(JointLimitsDeg limits) {
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
    limits.thighDeg.min = DEFAULT_THIGH_MIN_DEG;
    limits.thighDeg.max = DEFAULT_THIGH_MAX_DEG;
    limits.calfDeg.min = DEFAULT_CALF_MIN_DEG;
    limits.calfDeg.max = DEFAULT_CALF_MAX_DEG;
    return limits;
}

static bool geometryWithinJointLimits(const LegGeometry *geometry, const JointLimitsDeg *limits) {
    if (!geometry->valid) {
        return false;
    }

    float thighDeg = radToDeg(geometry->thetaThigh);
    float calfDeg = radToDeg(geometry->thetaCalf);
    return (
        thighDeg >= limits->thighDeg.min &&
        thighDeg <= limits->thighDeg.max &&
        calfDeg >= limits->calfDeg.min &&
        calfDeg <= limits->calfDeg.max
    );
}

static JointAnglesDeg clampJointAnglesToLimits(JointAnglesDeg jointAngles, const JointLimitsDeg *limits) {
    jointAngles.thighDeg = clampf(jointAngles.thighDeg, limits->thighDeg.min, limits->thighDeg.max);
    jointAngles.calfDeg = clampf(jointAngles.calfDeg, limits->calfDeg.min, limits->calfDeg.max);
    return jointAngles;
}

static float dist2f(Point2f a, Point2f b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

static Point2f footRelativeToAbsolute(Point2f relativeFoot) {
    Point2f absoluteFoot;
    absoluteFoot.x = relativeFoot.x + FOOT_ORIGIN_OFFSET.x;
    absoluteFoot.y = relativeFoot.y + FOOT_ORIGIN_OFFSET.y;
    return absoluteFoot;
}

static Point2f footAbsoluteToRelative(Point2f absoluteFoot) {
    Point2f relativeFoot;
    relativeFoot.x = absoluteFoot.x - FOOT_ORIGIN_OFFSET.x;
    relativeFoot.y = absoluteFoot.y - FOOT_ORIGIN_OFFSET.y;
    return relativeFoot;
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

static bool circleIntersections(Point2f c0, float r0, Point2f c1, float r1, Point2f *out1, Point2f *out2) {
    float dx = c1.x - c0.x;
    float dy = c1.y - c0.y;
    float d = sqrtf(dx * dx + dy * dy);
    if (d == 0.0f || d > (r0 + r1) || d < fabsf(r0 - r1)) {
        return false;
    }

    float a = (r0 * r0 - r1 * r1 + d * d) / (2.0f * d);
    float hSquared = r0 * r0 - a * a;
    if (hSquared < 0.0f) {
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

static Point2f chooseBellPoint(Point2f p1, Point2f p2) {
    return (p1.y > p2.y) ? p1 : p2;
}

static Point2f chooseCalfAttachPoint(Point2f p1, Point2f p2, Point2f knee) {
    float score1 = (knee.y - p1.y) * 3.0f + fabsf(knee.x - p1.x);
    float score2 = (knee.y - p2.y) * 3.0f + fabsf(knee.x - p2.x);
    return (score1 < score2) ? p1 : p2;
}

static LegGeometry solveGeometry(float thetaThigh, float thetaServo) {
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
    if (!circleIntersections(g.hip, BELL, g.servoHornEnd, LINK_SHORT, &bellI1, &bellI2)) {
        g.valid = false;
        return g;
    }

    g.bellArmA = chooseBellPoint(bellI1, bellI2);
    float bellAngle = atan2f(g.bellArmA.y - g.hip.y, g.bellArmA.x - g.hip.x);
    g.bellArmB.x = g.hip.x + BELL * cosf(bellAngle - (float)M_PI / 2.0f);
    g.bellArmB.y = g.hip.y + BELL * sinf(bellAngle - (float)M_PI / 2.0f);

    Point2f calfI1, calfI2;
    if (!circleIntersections(g.bellArmB, LINK_LONG, g.knee, CALF_ATTACH_OFFSET, &calfI1, &calfI2)) {
        g.valid = false;
        return g;
    }

    g.calfAttach = chooseCalfAttachPoint(calfI1, calfI2, g.knee);
    g.thetaCalf = atan2f(g.knee.y - g.calfAttach.y, g.knee.x - g.calfAttach.x);
    g.foot.x = g.knee.x + L_CALF * cosf(g.thetaCalf);
    g.foot.y = g.knee.y + L_CALF * sinf(g.thetaCalf);
    return g;
}

static void solveAnglesForAbsoluteFoot(Point2f target, float startThetaThigh, float startThetaServo, const JointLimitsDeg *limits, float *outThetaThigh, float *outThetaServo, float *outFootError, bool *outHasSolution, bool *outSuccess) {
    float bestThigh = startThetaThigh;
    float bestServo = startThetaServo;
    float stepThigh = 0.2f;
    float stepServo = 0.2f;
    LegGeometry bestGeometry = solveGeometry(bestThigh, bestServo);
    float bestError = FLT_MAX;

    if (geometryWithinJointLimits(&bestGeometry, limits)) {
        bestError = dist2f(bestGeometry.foot, target);
    }

    for (int i = 0; i < 28; ++i) {
        bool improved = false;
        const float candidates[8][2] = {
            {bestThigh + stepThigh, bestServo},
            {bestThigh - stepThigh, bestServo},
            {bestThigh, bestServo + stepServo},
            {bestThigh, bestServo - stepServo},
            {bestThigh + stepThigh, bestServo + stepServo},
            {bestThigh + stepThigh, bestServo - stepServo},
            {bestThigh - stepThigh, bestServo + stepServo},
            {bestThigh - stepThigh, bestServo - stepServo}
        };

        for (int j = 0; j < 8; ++j) {
            float thigh = clampAngle(candidates[j][0]);
            float servo = clampAngle(candidates[j][1]);
            LegGeometry geometry = solveGeometry(thigh, servo);
            if (!geometryWithinJointLimits(&geometry, limits)) {
                continue;
            }

            float error = dist2f(geometry.foot, target);
            if (error < bestError) {
                bestError = error;
                bestThigh = thigh;
                bestServo = servo;
                bestGeometry = geometry;
                improved = true;
            }
        }

        if (!improved) {
            stepThigh *= 0.5f;
            stepServo *= 0.5f;
        }
    }

    *outThetaThigh = bestThigh;
    *outThetaServo = bestServo;
    *outFootError = bestError;
    *outHasSolution = bestGeometry.valid && isfinite(bestError);
    *outSuccess = bestGeometry.valid && isfinite(bestError) && (bestError < 10.0f);
}

static void solveServoForJointAngles(float thetaThigh, float targetThetaCalf, float startThetaServo, const JointLimitsDeg *limits, float *outThetaServo, float *outJointError, bool *outHasSolution, bool *outSuccess) {
    float bestServo = startThetaServo;
    float stepServo = 0.2f;
    LegGeometry bestGeometry = solveGeometry(thetaThigh, bestServo);
    float bestError = FLT_MAX;

    if (geometryWithinJointLimits(&bestGeometry, limits)) {
        bestError = fabsf(clampAngle(bestGeometry.thetaCalf - targetThetaCalf));
    }

    for (int i = 0; i < 28; ++i) {
        bool improved = false;
        const float candidates[2] = {
            bestServo + stepServo,
            bestServo - stepServo
        };

        for (int j = 0; j < 2; ++j) {
            float servo = clampAngle(candidates[j]);
            LegGeometry geometry = solveGeometry(thetaThigh, servo);
            if (!geometryWithinJointLimits(&geometry, limits)) {
                continue;
            }

            float error = fabsf(clampAngle(geometry.thetaCalf - targetThetaCalf));
            if (error < bestError) {
                bestError = error;
                bestServo = servo;
                bestGeometry = geometry;
                improved = true;
            }
        }

        if (!improved) {
            stepServo *= 0.5f;
        }
    }

    *outThetaServo = bestServo;
    *outJointError = bestError;
    *outHasSolution = bestGeometry.valid && isfinite(bestError);
    *outSuccess = bestGeometry.valid && isfinite(bestError) && (bestError < degToRad(3.0f));
}

static void robotDogLegInitNeutral(void) {
    float thetaThigh;
    float thetaServo;
    float footError;
    bool hasSolution;
    bool success;
    Point2f neutralFootAbs = FOOT_ORIGIN_OFFSET;
    JointLimitsDeg limits = defaultJointLimits();

    solveAnglesForAbsoluteFoot(
        neutralFootAbs,
        -((float)M_PI / 4.0f),
        (float)M_PI / 6.0f,
        &limits,
        &thetaThigh,
        &thetaServo,
        &footError,
        &hasSolution,
        &success
    );

    if (hasSolution) {
        g_thighNeutralAngle = thetaThigh;
        g_servoNeutralAngle = thetaServo;
        g_neutralInitialized = true;
    } else {
        g_thighNeutralAngle = 0.0f;
        g_servoNeutralAngle = 0.0f;
        g_neutralInitialized = false;
    }
}

static float thetaToModelServoDeg(float theta, float neutralTheta) {
    return 90.0f + radToDeg(theta - neutralTheta);
}

static float modelServoDegToTheta(float modelDeg, float neutralTheta) {
    return neutralTheta + degToRad(modelDeg - 90.0f);
}

static LegIKResult robotDogLegIK(float footX, float footY, float startThetaThigh, float startThetaServo, const JointLimitsDeg *limits) {
    if (!g_neutralInitialized) {
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
        &success
    );

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
    float ticks = SERVO_PWM_MIN_TICKS + ratio * (float)(SERVO_PWM_MAX_TICKS - SERVO_PWM_MIN_TICKS);
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

static void smoothServoWriteEstimate(SmoothServo *s) {
    float est = smoothServoReadEstimate(s);
    if (s->attached) {
        g_pwm.setPWM(s->channel, 0, servoDegToPcaTicks(est));
    }
}

static void smoothServoBegin(SmoothServo *s, uint8_t channel, float initialDeg, float minDeg, float maxDeg, float centerDeg, float sign, float supplyVolts) {
    s->channel = channel;
    s->attached = (channel != INVALID_CHANNEL);
    s->minDeg = minDeg;
    s->maxDeg = maxDeg;
    s->centerDeg = centerDeg;
    s->sign = sign;
    s->maxSpeedDegPerSec = servoSpeedDegPerSecFromVoltage(supplyVolts) * SERVO_SPEED_SAFETY_FACTOR;
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
    float eased = easeInOutSmoothStep(t);
    return lerpf(s->startDeg, s->targetDeg, eased);
}

static void smoothServoCommandRaw(SmoothServo *s, float rawTargetDeg) {
    if (s->channel != INVALID_CHANNEL) {
        s->attached = true;
    }
    rawTargetDeg = clampf(rawTargetDeg, s->minDeg, s->maxDeg);
    float currentEstimate = smoothServoReadEstimate(s);
    s->estimatedDeg = currentEstimate;
    s->startDeg = currentEstimate;
    s->targetDeg = rawTargetDeg;
    s->moveStartUs = micros();

    float delta = fabsf(s->targetDeg - s->startDeg);
    float durationSec = 0.0f;
    if (s->maxSpeedDegPerSec > 0.0f) {
        durationSec = delta / s->maxSpeedDegPerSec;
    }
    durationSec *= MOTION_TIME_SCALE;

    if (delta > 0.1f && durationSec < 0.04f) {
        durationSec = 0.04f;
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
}

static void smoothServoCommandModelDeg(SmoothServo *s, float modelDeg) {
    float raw = applyServoCalibration(modelDeg, s->centerDeg, s->sign);
    smoothServoCommandRaw(s, raw);
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
    result.thigh = removeServoCalibration(leg->thigh.estimatedDeg, leg->thigh.centerDeg, leg->thigh.sign);
    result.calf = removeServoCalibration(leg->calf.estimatedDeg, leg->calf.centerDeg, leg->calf.sign);
    return result;
}

static void updateLegDerivedStateFromServo(RobotLegState *leg, ServoAnglesDeg servoAngles, bool desiredState) {
    float thetaThigh = modelServoDegToTheta(servoAngles.thigh, g_thighNeutralAngle);
    float thetaServo = modelServoDegToTheta(servoAngles.calf, g_servoNeutralAngle);
    LegGeometry geometry = solveGeometry(thetaThigh, thetaServo);

    if (!geometry.valid) {
        return;
    }

    if (desiredState) {
        leg->desiredServoAngles = servoAngles;
        leg->desiredJointAngles.thighDeg = radToDeg(geometry.thetaThigh);
        leg->desiredJointAngles.calfDeg = radToDeg(geometry.thetaCalf);
        leg->desiredFoot = footAbsoluteToRelative(geometry.foot);
    } else {
        leg->currentServoAngles = servoAngles;
        leg->currentJointAngles.thighDeg = radToDeg(geometry.thetaThigh);
        leg->currentJointAngles.calfDeg = radToDeg(geometry.thetaCalf);
        leg->currentFoot = footAbsoluteToRelative(geometry.foot);
    }
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
        case MODE_DIRECT_FOOT_XY: return "direct_foot_xy";
        case MODE_DIRECT_JOINT_ANGLES: return "direct_joint_angles";
        case MODE_DIRECT_SERVO_ANGLES: return "direct_servo_angles";
        case MODE_BUILTIN_WALK: return "builtin_walk";
        case MODE_BUILTIN_CROUCH: return "builtin_crouch";
        case MODE_ANIMATION_PLAYBACK: return "animation_playback";
        default: return "idle";
    }
}

static void setMode(RobotMode mode) {
    g_mode = mode;
    g_modeStartMs = millis();

    if (mode != MODE_BUILTIN_WALK && mode != MODE_BUILTIN_CROUCH) {
        g_builtinName[0] = '\0';
    }

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
    smoothServoBegin(&leg->thigh, config->thighChannel, THIGH_CENTER_DEG, SERVO_MIN_DEG, SERVO_MAX_DEG, THIGH_CENTER_DEG, THIGH_SIGN, SERVO_SUPPLY_VOLTS);
    smoothServoBegin(&leg->calf, config->calfChannel, CALF_CENTER_DEG, SERVO_MIN_DEG, SERVO_MAX_DEG, CALF_CENTER_DEG, CALF_SIGN, SERVO_SUPPLY_VOLTS);
    leg->thighChannel = config->thighChannel;
    leg->calfChannel = config->calfChannel;
    leg->jointLimits = defaultJointLimits();
    leg->lastThetaThigh = g_thighNeutralAngle;
    leg->lastThetaServo = g_servoNeutralAngle;
    leg->desiredFoot.x = 0.0f;
    leg->desiredFoot.y = 0.0f;
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

static void robotLegUpdate(RobotLegState *leg) {
    smoothServoUpdate(&leg->thigh);
    smoothServoUpdate(&leg->calf);
    ServoAnglesDeg currentServoAngles = getCurrentServoAngles(leg);
    updateLegDerivedStateFromServo(leg, currentServoAngles, false);
}

static bool robotLegCommandJointAngles(int legIndex, float thighDeg, float calfDeg);

static void robotLegSetServoChannels(int legIndex, uint8_t thighChannel, uint8_t calfChannel) {
    RobotLegState *leg = &g_legs[legIndex];
    leg->thigh.channel = thighChannel;
    leg->calf.channel = calfChannel;
    leg->thigh.attached = (thighChannel != INVALID_CHANNEL);
    leg->calf.attached = (calfChannel != INVALID_CHANNEL);
    leg->thighChannel = thighChannel;
    leg->calfChannel = calfChannel;
    smoothServoWriteEstimate(&leg->thigh);
    smoothServoWriteEstimate(&leg->calf);
}

static void releaseAllServos(void) {
    for (int i = 0; i < NUM_LEGS; ++i) {
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
    robotLegCommandJointAngles(legIndex, leg->desiredJointAngles.thighDeg, leg->desiredJointAngles.calfDeg);
}

static void robotLegCommandServo(int legIndex, float thighServoDeg, float calfServoDeg) {
    RobotLegState *leg = &g_legs[legIndex];
    g_servosReleased = false;
    smoothServoCommandModelDeg(&leg->thigh, thighServoDeg);
    smoothServoCommandModelDeg(&leg->calf, calfServoDeg);
    leg->desiredServoAngles.thigh = clampf(thighServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    leg->desiredServoAngles.calf = clampf(calfServoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    updateLegDerivedStateFromServo(leg, leg->desiredServoAngles, true);
}

static bool robotLegCommandFoot(int legIndex, float footX, float footY) {
    RobotLegState *leg = &g_legs[legIndex];
    LegIKResult ik = robotDogLegIK(footX, footY, leg->lastThetaThigh, leg->lastThetaServo, &leg->jointLimits);
    if (!ik.hasSolution) {
        setLegError(leg, "IK failed for foot target");
        return false;
    }

    robotLegCommandServo(legIndex, ik.thighServoDeg, ik.calfServoDeg);
    leg->desiredFoot = ik.resolvedFoot;
    leg->lastThetaThigh = ik.thetaThigh;
    leg->lastThetaServo = ik.thetaServo;
    clearLegError(leg);
    return true;
}

static bool robotLegCommandJointAngles(int legIndex, float thighDeg, float calfDeg) {
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

    solveServoForJointAngles(thetaThigh, degToRad(limitedAngles.calfDeg), leg->lastThetaServo, &leg->jointLimits, &thetaServo, &jointError, &hasSolution, &success);
    if (!hasSolution) {
        setLegError(leg, "Joint angle solve failed");
        return false;
    }

    float thighServoDeg = thetaToModelServoDeg(thetaThigh, g_thighNeutralAngle);
    float calfServoDeg = thetaToModelServoDeg(thetaServo, g_servoNeutralAngle);
    robotLegCommandServo(legIndex, thighServoDeg, calfServoDeg);
    leg->desiredJointAngles.thighDeg = limitedAngles.thighDeg;
    leg->desiredJointAngles.calfDeg = limitedAngles.calfDeg;
    leg->lastThetaThigh = thetaThigh;
    leg->lastThetaServo = thetaServo;
    clearLegError(leg);
    return true;
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

static bool animationAddFrame(AnimationClip *clip, int legIndex, float time, float x, float y) {
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
    track->count += 1;
    return true;
}

static void animationCommit(AnimationClip *clip) {
    clip->ready = true;
    clip->playing = false;
    clip->paused = false;
}

static Point2f interpolateTrack(const LegTrack *track, float timeSec) {
    Point2f result = {0.0f, 0.0f};
    if (track->count <= 0) {
        return result;
    }

    if (timeSec <= track->frames[0].time) {
        result.x = track->frames[0].x;
        result.y = track->frames[0].y;
        return result;
    }

    if (timeSec >= track->frames[track->count - 1].time) {
        result.x = track->frames[track->count - 1].x;
        result.y = track->frames[track->count - 1].y;
        return result;
    }

    for (int i = 0; i < track->count - 1; ++i) {
        AnimationFrame a = track->frames[i];
        AnimationFrame b = track->frames[i + 1];
        if (timeSec >= a.time && timeSec <= b.time) {
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

static Point2f builtinWalkTarget(int legIndex, float timeSec) {
    static const float phaseOffsets[NUM_LEGS] = {0.0f, 0.5f, 0.5f, 0.0f};
    float cycle = fmodf((timeSec / 1.6f) + phaseOffsets[legIndex], 1.0f);
    float x = lerpf(-18.0f, 18.0f, cycle);
    float y = (cycle < 0.45f) ? (10.0f * sinf((cycle / 0.45f) * (float)M_PI)) : (-6.0f * sinf(((cycle - 0.45f) / 0.55f) * (float)M_PI));
    Point2f foot = {x, y};
    return foot;
}

static Point2f builtinCrouchTarget(float timeSec) {
    float t = clampf(timeSec / 0.9f, 0.0f, 1.0f);
    float eased = easeInOutSmoothStep(t);
    Point2f foot = {0.0f, lerpf(0.0f, -28.0f, eased)};
    return foot;
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

static void sendBuiltinStatus(void) {
    Serial.print("{\"type\":\"builtin_status\",\"name\":\"");
    jsonPrintEscaped(g_builtinName);
    Serial.print("\",\"mode\":\"");
    Serial.print(modeToString(g_mode));
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
        Serial.print("\",\"servoChannelMap\":{\"thigh\":");
        Serial.print((int)leg->thighChannel);
        Serial.print(",\"calf\":");
        Serial.print((int)leg->calfChannel);
        Serial.print("},\"jointLimits\":{\"thighDeg\":{\"min\":");
        Serial.print(leg->jointLimits.thighDeg.min, 3);
        Serial.print(",\"max\":");
        Serial.print(leg->jointLimits.thighDeg.max, 3);
        Serial.print("},\"calfDeg\":{\"min\":");
        Serial.print(leg->jointLimits.calfDeg.min, 3);
        Serial.print(",\"max\":");
        Serial.print(leg->jointLimits.calfDeg.max, 3);
        Serial.print("}},\"desired\":{");
        Serial.print("\"foot\":{\"x\":");
        Serial.print(leg->desiredFoot.x, 3);
        Serial.print(",\"y\":");
        Serial.print(leg->desiredFoot.y, 3);
        Serial.print("},\"jointAnglesDeg\":{\"thigh\":");
        Serial.print(leg->desiredJointAngles.thighDeg, 3);
        Serial.print(",\"calf\":");
        Serial.print(leg->desiredJointAngles.calfDeg, 3);
        Serial.print("},\"servoAnglesDeg\":{\"thigh\":");
        Serial.print(leg->desiredServoAngles.thigh, 3);
        Serial.print(",\"calf\":");
        Serial.print(leg->desiredServoAngles.calf, 3);
        Serial.print("}},\"current\":{");
        Serial.print("\"foot\":{\"x\":");
        Serial.print(leg->currentFoot.x, 3);
        Serial.print(",\"y\":");
        Serial.print(leg->currentFoot.y, 3);
        Serial.print("},\"jointAnglesDeg\":{\"thigh\":");
        Serial.print(leg->currentJointAngles.thighDeg, 3);
        Serial.print(",\"calf\":");
        Serial.print(leg->currentJointAngles.calfDeg, 3);
        Serial.print("},\"servoAnglesDeg\":{\"thigh\":");
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
        updateLegDerivedStateFromServo(&g_legs[i], g_legs[i].currentServoAngles, false);
    }
}

static void runBuiltinsAndAnimation(void) {
    float elapsedSec = (millis() - g_modeStartMs) / 1000.0f;

    if (g_mode == MODE_BUILTIN_WALK) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            Point2f foot = builtinWalkTarget(i, elapsedSec);
            robotLegCommandFoot(i, foot.x, foot.y);
        }
    }

    if (g_mode == MODE_BUILTIN_CROUCH) {
        Point2f foot = builtinCrouchTarget(elapsedSec);
        for (int i = 0; i < NUM_LEGS; ++i) {
            robotLegCommandFoot(i, foot.x, foot.y);
        }
    }

    if (g_mode == MODE_ANIMATION_PLAYBACK && g_animation.ready && g_animation.playing && !g_animation.paused) {
        float timeSec = fmodf((millis() - g_animation.startMs) / 1000.0f, g_animation.duration);
        for (int i = 0; i < NUM_LEGS; ++i) {
            Point2f foot = interpolateTrack(&g_animation.tracks[i], timeSec);
            robotLegCommandFoot(i, foot.x, foot.y);
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

    if ((g_mode == MODE_BUILTIN_WALK || g_mode == MODE_BUILTIN_CROUCH) && (nowMs - g_lastBuiltinStatusMs) >= BUILTIN_STATUS_INTERVAL_MS) {
        g_lastBuiltinStatusMs = nowMs;
        sendBuiltinStatus();
    }

    if (g_mode == MODE_ANIMATION_PLAYBACK && (nowMs - g_lastAnimationStatusMs) >= ANIMATION_STATUS_INTERVAL_MS) {
        g_lastAnimationStatusMs = nowMs;
        sendAnimationProgress();
    }
}

static void handleModeCommand(const char *modeString) {
    if (strcmp(modeString, "idle") == 0) setMode(MODE_IDLE);
    if (strcmp(modeString, "direct_foot_xy") == 0) setMode(MODE_DIRECT_FOOT_XY);
    if (strcmp(modeString, "direct_joint_angles") == 0) setMode(MODE_DIRECT_JOINT_ANGLES);
    if (strcmp(modeString, "direct_servo_angles") == 0) setMode(MODE_DIRECT_SERVO_ANGLES);
    if (strcmp(modeString, "builtin_walk") == 0) setMode(MODE_BUILTIN_WALK);
    if (strcmp(modeString, "builtin_crouch") == 0) setMode(MODE_BUILTIN_CROUCH);
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
        char name[MAX_NAME_LEN] = "";
        if (!jsonExtractString(line, "name", name, sizeof(name))) {
            sendError(seq, "run_builtin missing name");
            return;
        }

        strncpy(g_builtinName, name, MAX_NAME_LEN - 1);
        g_builtinName[MAX_NAME_LEN - 1] = '\0';
        if (strcmp(name, "walk") == 0) {
            setMode(MODE_BUILTIN_WALK);
            sendAck(seq, "builtin walk");
            return;
        }

        if (strcmp(name, "crouch") == 0) {
            setMode(MODE_BUILTIN_CROUCH);
            sendAck(seq, "builtin crouch");
            return;
        }

        sendError(seq, "unknown builtin");
        return;
    }

    if (strcmp(type, "set_leg_foot_xy") == 0) {
        char legId[24] = "";
        float x = 0.0f;
        float y = 0.0f;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "x", &x) || !jsonExtractFloat(line, "y", &y)) {
            sendError(seq, "foot command missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0) {
            sendError(seq, "unknown leg");
            return;
        }

        setMode(MODE_DIRECT_FOOT_XY);
        if (!robotLegCommandFoot(legIndex, x, y)) {
            sendError(seq, g_legs[legIndex].lastError);
            return;
        }
        sendAck(seq, "foot target accepted");
        return;
    }

    if (strcmp(type, "set_leg_joint_angles") == 0) {
        char legId[24] = "";
        float thighDeg = 0.0f;
        float calfDeg = 0.0f;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "thighDeg", &thighDeg) || !jsonExtractFloat(line, "calfDeg", &calfDeg)) {
            sendError(seq, "joint command missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0) {
            sendError(seq, "unknown leg");
            return;
        }

        setMode(MODE_DIRECT_JOINT_ANGLES);
        if (!robotLegCommandJointAngles(legIndex, thighDeg, calfDeg)) {
            sendError(seq, g_legs[legIndex].lastError);
            return;
        }
        sendAck(seq, "joint target accepted");
        return;
    }

    if (strcmp(type, "set_leg_servo_angles") == 0) {
        char legId[24] = "";
        float thighServoDeg = 0.0f;
        float calfServoDeg = 0.0f;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "thighServoDeg", &thighServoDeg) || !jsonExtractFloat(line, "calfServoDeg", &calfServoDeg)) {
            sendError(seq, "servo command missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0) {
            sendError(seq, "unknown leg");
            return;
        }

        setMode(MODE_DIRECT_SERVO_ANGLES);
        robotLegCommandServo(legIndex, thighServoDeg, calfServoDeg);
        sendAck(seq, "servo target accepted");
        return;
    }

    if (strcmp(type, "set_leg_servo_channel_map") == 0) {
        char legId[24] = "";
        int thighChannel = 0;
        int calfChannel = 0;
        if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractInt(line, "thighChannel", &thighChannel) || !jsonExtractInt(line, "calfChannel", &calfChannel)) {
            sendError(seq, "servo channel map missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0) {
            sendError(seq, "unknown leg");
            return;
        }

        if (thighChannel < 0 || thighChannel > 15 || calfChannel < 0 || calfChannel > 15) {
            sendError(seq, "servo channels must be between 0 and 15");
            return;
        }

        robotLegSetServoChannels(legIndex, (uint8_t)thighChannel, (uint8_t)calfChannel);
        sendAck(seq, "servo channel map updated");
        return;
    }

    if (strcmp(type, "set_leg_joint_limits") == 0) {
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
            !jsonExtractFloat(line, "calfMaxDeg", &calfMaxDeg)
        ) {
            sendError(seq, "joint limit command missing fields");
            return;
        }

        int legIndex = legIndexFromId(legId);
        if (legIndex < 0) {
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
            if (!jsonExtractString(line, "legId", legId, sizeof(legId)) || !jsonExtractFloat(line, "time", &time) || !jsonExtractFloat(line, "x", &x) || !jsonExtractFloat(line, "y", &y)) {
                sendError(seq, "animation frame missing fields");
                return;
            }

            int legIndex = legIndexFromId(legId);
            if (!animationAddFrame(&g_animation, legIndex, time, x, y)) {
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
    delay(300);
    Serial.println("Starting up");
    Wire.begin();
    g_pwm.begin();
    g_pwm.setPWMFreq(PCA9685_FREQUENCY_HZ);
    robotDogLegInitNeutral();
    animationReset(&g_animation);

    for (int i = 0; i < NUM_LEGS; ++i) {
        robotLegBegin(&g_legs[i], &LEG_CONFIGS[i]);
        robotLegCommandFoot(i, 0.0f, 0.0f);
    }

    g_lastTelemetryMs = millis();
    g_lastBuiltinStatusMs = millis();
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
