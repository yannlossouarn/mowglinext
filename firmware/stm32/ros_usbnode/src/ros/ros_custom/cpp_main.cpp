/**
 ******************************************************************************
 * @file    cpp_main.cpp
 * @author  Georg Swoboda <cn@warp.at>
 * @date    21/09/2022
 * @version 2.0.0
 * @brief   COBS protocol bridge — replaces rosserial with direct COBS packets
 ******************************************************************************
 *
 * Migration from rosserial to COBS:
 *   - All ROS topic publish/subscribe removed
 *   - All ROS service servers/clients removed
 *   - ros::NodeHandle replaced by mowgli_comms COBS layer
 *   - USB CDC RX feeds mowgli_comms_process_rx() instead of ringbuffer
 *   - Packet send uses mowgli_comms_send_*() convenience wrappers
 *   - cmd_vel timeout uses HAL_GetTick() instead of nh.now()
 *
 ******************************************************************************
 */

#include "board.h"
#include "main.h"
#include "adc.h"

#include <cpp_main.h>
#include "panel.h"
#include "charger.h"
#include "emergency.h"
#include "drivemotor.h"
#include "blademotor.h"
#include "pid.hpp"
#include "ultrasonic_sensor.h"
#include "stm32f_board_hal.h"
#include "nbt.h"

// USB CDC
#include "usbd_cdc_if.h"

// COBS protocol (replaces rosserial)
#include "mowgli_protocol.h"
#include "mowgli_comms.h"

// Math
#include <cmath>

// IMU
#include "imu/imu.h"

#ifdef OPTION_PERIMETER
#include "perimeter.h"
#endif

/* ---------------------------------------------------------------------------
 * Timer intervals
 * ---------------------------------------------------------------------------*/
/*
 * 10 ms (100 Hz) IMU rate. The MEMS sensor (LSM6DS33 / WT901 / LIS3MDL)
 * I2C read takes <1 ms, leaving plenty of CPU headroom in the 10 ms
 * window. At 115200 baud the additional ~50-byte IMU packet costs
 * ~4 ms wire time per second, so total serial usage stays comfortably
 * under the 11.5 KB/s budget alongside the existing 47 Hz odom and
 * 4 Hz status streams.
 */
#define IMU_NBT_TIME_MS    10
#define MOTORS_NBT_TIME_MS 20
#define STATUS_NBT_TIME_MS 250
#define PANEL_NBT_TIME_MS  100
#define LED_NBT_TIME_MS    1000
#define BLADE_NBT_TIME_MS  250

/* ---------------------------------------------------------------------------
 * Drive motor control state
 * ---------------------------------------------------------------------------*/
/* Target wheel velocities written by on_cmd_vel (ISR context) and read by
 * motors_handler() at MOTORS_NBT_TIME_MS cadence. Replaces the previous
 * "ISR writes PWM directly" path so motors_handler can run a wheel-level
 * PI loop using encoder feedback instead of forwarding an open-loop
 * cmd_vel × PWM_PER_MPS mapping. */
static volatile float left_target_mps  = 0.0f;
static volatile float right_target_mps = 0.0f;

/* PWM ultimately sent to the PAC5210. Output of the PI loop in motors_handler;
 * legacy globals kept under the same names so the snapshot+watchdog logic
 * downstream is unchanged. */
static int16_t left_pwm_signed  = 0;
static int16_t right_pwm_signed = 0;

/* ---------------------------------------------------------------------------
 * Wheel-level PI controller
 * ---------------------------------------------------------------------------
 * Brushed-DC motors driven by the PAC5210 have a hard static-friction
 * deadband (~PWM 40). Open-loop cmd_vel × PWM_PER_MPS produces PWM=2 for
 * a 0.05 m/s target — well below deadband, motors buzz, the chassis
 * doesn't move. We can't fix the motor physics but we CAN bridge the
 * deadband with closed-loop feedback: while the target says "move" and
 * the encoder says "stalled", the PI integrator ramps PWM up until the
 * motor breaks free, then settles around whatever PWM keeps the wheel
 * at target speed.
 *
 * Run at MOTORS_NBT_TIME_MS = 20 ms (50 Hz). Encoder feedback comes from
 * left_ticks_signed / right_ticks_signed (signed cumulative ticks
 * maintained by drivemotor.c, already direction-aware).
 *
 * Output = feedforward (target × PWM_PER_MPS, preserves the open-loop
 * behaviour above deadband) + Kp × error + integral_pwm. The integral is
 * stored pre-multiplied by Ki for trivial anti-windup.
 *
 * Set USE_WHEEL_PI to 0 to fall back to open-loop forwarding for
 * debugging / hardware bring-up. */
#define USE_WHEEL_PI         1
#define WHEEL_PI_KP_PWM_PER_MPS    30.0f   /* proportional gain */
#define WHEEL_PI_KI_PWM_PER_MPS_S 5000.0f /* integral gain (50 PWM in ~0.2 s when err=0.05 m/s) */
#define WHEEL_PI_INT_MAX_PWM     100.0f   /* anti-windup clamp on the integral term */
#define WHEEL_PI_DT_S            (MOTORS_NBT_TIME_MS / 1000.0f)
#define WHEEL_PI_TICKS_PER_M    300.0f    /* must match mowgli_robot.yaml: ticks_per_meter */

/* Per-wheel velocity PI — battle-tested PX4 PID core (pid.hpp). Gains/limits
 * set once in init_ROS(). The integrator (kept inside the PID object) is what
 * bridges the static-friction deadband; output is the closed-loop PWM trim
 * added to the open-loop feedforward below. */
static PID left_wheel_pid;
static PID right_wheel_pid;
static int32_t prev_left_ticks_signed_pi  = 0;
static int32_t prev_right_ticks_signed_pi = 0;
static float prev_left_target_mps  = 0.0f;
static float prev_right_target_mps = 0.0f;

/* Open-loop feedforward velocity->PWM scale. Runtime-tunable copy of the
 * board.h PWM_PER_MPS default so the ROS 2 host can retune the drive loop via
 * PKT_ID_SET_DRIVE_PID without a reflash. Seeded with the compile-time default,
 * which therefore remains the power-on fallback (this board has no config
 * persistence; the bridge re-sends the gains on every reconnect). */
static volatile float g_pwm_per_mps = (float)PWM_PER_MPS;

/* Static-friction breakaway feedforward [PWM], runtime-tunable via
 * PKT_ID_SET_DRIVE_PID. When a non-zero wheel target is commanded, the loop
 * adds sign(target) * g_drive_deadband_pwm to the open-loop feedforward so the
 * wheel crosses the PAC5210 breakaway IMMEDIATELY, rather than the PI
 * integrator (or the host gyro-rate loop) having to wind up to reach it. The
 * wind-up was the stick-slip source behind the low-speed autonomous-pivot
 * yaw/position corruption. Default 0 keeps the open-loop mapping unchanged. */
static volatile float g_drive_deadband_pwm = 0.0f;

/* Per-wheel loop selector, runtime-tunable via PKT_ID_SET_DRIVE_PID (replaces
 * the compile-time USE_WHEEL_PI switch so the open-loop vs closed-loop A/B no
 * longer needs a reflash). Seeded with the compile-time default. */
static volatile uint8_t g_use_wheel_pi = (uint8_t)USE_WHEEL_PI;

/* Standstill position hold (runtime-tunable via PKT_ID_SET_DRIVE_PID). While
 * the robot is actively controlled (not IDLE / not emergency) and commanded to
 * ~0 velocity, the loop latches the encoder position and pushes back against
 * creep instead of coasting — fixing the backlash/tire-windup back-creep that
 * walks the heading off after a pivot. Released (coast) only when really idle.
 * g_hold_kp is PWM per tick of position error; the breakaway (g_drive_deadband_
 * pwm) supplies the static-friction floor so a small error can still correct. */
static volatile uint8_t g_hold_enabled = 1u;
static volatile float g_hold_kp = 4.0f;
/* Hold deadzone (ticks) and output clamp (PWM). A tick is ~3.3 mm; tolerate a
 * couple before correcting so quantization noise doesn't buzz the motors, and
 * clamp the corrective PWM well under the motion range. */
#define HOLD_TOL_TICKS  2
#define HOLD_MAX_PWM    120.0f

/* ---------------------------------------------------------------------------
 * IMU-to-odometry discrepancy detector
 * ---------------------------------------------------------------------------
 * Flags stick-slip / wheel-slip / impact from the residual between the chassis
 * yaw rate the WHEEL ENCODERS imply ((v_r - v_l)/WHEEL_BASE) and the one the IMU
 * GYRO measures, plus a stall flag when a real velocity is commanded but the
 * wheels aren't turning (obstruction / collision / deadband stall). Computed in
 * wheelTicks_handler at the ~50 Hz odom cadence; the raw rates + flags ride the
 * drive-telem packet so the host can take its own residual / threshold too.
 *
 * The gyro here is RAW (the host bias-corrects); a ~0.05 rad/s raw bias sits well
 * under the yaw threshold, and an EMA + a few-sample debounce reject transients.
 * Thresholds are deliberately gross — this is a flag, not a precise estimator. */
#define SLIP_EMA_ALPHA       0.3f   /* low-pass on |yaw residual| */
#define SLIP_YAW_THRESH_RPS  0.35f  /* |wheel yaw − gyro yaw| flag threshold [rad/s] */
#define SLIP_YAW_PERSIST     3u     /* consecutive samples over threshold to latch */
#define STALL_CMD_MPS        0.08f  /* commanded |wheel speed| considered "moving" */
#define STALL_MEAS_MPS       0.02f  /* measured |wheel speed| considered "stopped" */
#define STALL_PERSIST        5u     /* consecutive samples to latch a stall */
/* Jam vs deadband: a stall with max wheel load >= this is a real obstruction
 * (motor driving hard), vs a benign deadband stall (~0 load). Field-calibrated
 * 2026-06-17: idle load=0, driving load >=35, jam railed wheel 100-140. */
#define DRIVE_JAM_LOAD_THRESH 30u
/* Soft "bog" (high grass): the wheels turn but well below the commanded speed,
 * with no impact — the chassis is loaded and barely advancing. */
#define BOG_CMD_MPS          0.10f  /* commanded |wheel speed| to test for bog */
#define BOG_RATIO            0.5f   /* flag when measured < BOG_RATIO * commanded */
#define BOG_PERSIST          25u    /* ~0.5 s at the 50 Hz odom cadence */
/* Hard-collision impact: a peak in the dynamic acceleration (|accel| deviation
 * from a slow gravity baseline) at the 100 Hz IMU cadence. Magnitude-based, so
 * it is independent of IMU mounting orientation. */
#define IMPACT_BASELINE_ALPHA 0.01f /* slow EMA on |accel| (~1 s) → tracks gravity */
#define IMPACT_THRESH_MPS2    6.0f  /* |accel − baseline| over this = a hard hit */
#define IMPACT_HOLD_TICKS     30u   /* latch the flag ~300 ms at 100 Hz */
#define IMPACT_PEAK_DECAY     0.95f /* peak-hold decay for the reported magnitude */
/* Blade bog (high grass loading the blade): commanded on but the RPM has
 * collapsed relative to its own free-running maximum this session. Self-
 * calibrating, so no absolute nominal-RPM constant is needed. */
#define BLADE_BOG_RATIO       0.6f  /* flag when rpm < ratio * running-max rpm */
#define BLADE_RPM_FLOOR       800u  /* running-max must exceed this to trust the ratio */
#define BLADE_MAX_DECAY       2u    /* per-tick decay of the running max [rpm] */
#define BLADE_BOG_PERSIST     20u   /* consecutive 100 Hz samples to latch */

/* Latest raw IMU yaw rate [rad/s], published by broadcast_handler, read by the
 * detector in wheelTicks_handler. Single-writer/single-reader, both on the main
 * loop — volatile is belt-and-suspenders. */
static volatile float g_imu_gyro_z = 0.0f;
/* Detector outputs (read by the drive-telem builder in motors_handler). */
static volatile int16_t g_wheel_yaw_mrad_s = 0;
static volatile int16_t g_imu_yaw_mrad_s = 0;
static volatile int16_t g_accel_peak_mg = 0;
static volatile uint8_t g_slip_flags = 0u;
/* Impact + blade-bog detector outputs, set in broadcast_handler (100 Hz, where
 * the accel + blade RPM are read), folded into g_slip_flags by the odom-rate
 * detector. g_impact_hold counts down so the 25 Hz telem catches a brief hit. */
static volatile uint8_t g_impact_hold = 0u;
static volatile uint8_t g_blade_bog = 0u;

/* Runtime-tunable detector magnitude thresholds (PKT_ID_SET_DETECTOR_PARAMS).
 * Seeded from the compile-time defaults above, which therefore remain the
 * power-on fallback. Debounce/persist counts + EMA taus stay compile-time. */
static volatile float g_det_yaw_thresh = SLIP_YAW_THRESH_RPS;
static volatile float g_det_stall_cmd = STALL_CMD_MPS;
static volatile float g_det_stall_meas = STALL_MEAS_MPS;
static volatile float g_det_impact_thresh = IMPACT_THRESH_MPS2;
static volatile float g_det_bog_cmd = BOG_CMD_MPS;
static volatile float g_det_bog_ratio = BOG_RATIO;
static volatile float g_det_jam_load = (float)DRIVE_JAM_LOAD_THRESH;
static volatile float g_det_blade_bog_ratio = BLADE_BOG_RATIO;

/* ---------------------------------------------------------------------------
 * Blade motor control state
 * ---------------------------------------------------------------------------*/
static volatile uint8_t target_blade_on_off = 0;
static uint8_t blade_on_off        = 0;
static uint8_t blade_direction     = 0;

/* ---------------------------------------------------------------------------
 * cmd_vel timeout tracking (replaces ros::Time)
 * ---------------------------------------------------------------------------*/
static volatile uint32_t last_cmd_vel_tick = 0;

/* ---------------------------------------------------------------------------
 * High-level state received from host
 * ---------------------------------------------------------------------------*/
static uint8_t hl_current_mode = 0;
static uint8_t hl_gps_quality  = 0;

/* ---------------------------------------------------------------------------
 * Heartbeat watchdog
 * ---------------------------------------------------------------------------*/
static volatile uint32_t last_heartbeat_tick   = 0;
#define HEARTBEAT_TIMEOUT_MS 2000u

/* ---------------------------------------------------------------------------
 * Reboot flag
 * ---------------------------------------------------------------------------*/
static bool reboot_flag = false;

/* ---------------------------------------------------------------------------
 * Non-blocking timers
 * ---------------------------------------------------------------------------*/
static nbt_t motors_nbt;
static nbt_t panel_nbt;
static nbt_t imu_nbt;
static nbt_t status_nbt;
static nbt_t led_nbt;
static nbt_t blade_nbt;

/* ---------------------------------------------------------------------------
 * Odometry timing
 * ---------------------------------------------------------------------------*/
static uint32_t last_odom_tick = 0;

/* Forward declarations */
static void update_blade_led(void);
static void update_slip_detector(int16_t left_v_mm_s, int16_t right_v_mm_s);
static void update_impact_detector(float ax, float ay, float az);
static void update_blade_bog_detector(void);

/* ---------------------------------------------------------------------------
 * COBS packet handlers (Host -> Firmware)
 * ---------------------------------------------------------------------------*/

static void on_heartbeat(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_heartbeat_t) - 2u) {
        return;
    }

    const pkt_heartbeat_t *pkt = (const pkt_heartbeat_t *)data;

    last_heartbeat_tick = HAL_GetTick();

    if (pkt->emergency_requested) {
        Emergency_SetState(1);
    }
    if (pkt->emergency_release_requested) {
        /* Only clear emergency if no physical sensor is still asserted.
         * Firmware is the sole safety authority — never bypass hardware. */
        if (!Emergency_StopButtonYellow() && !Emergency_StopButtonWhite() &&
            !Emergency_WheelLiftBlue() && !Emergency_WheelLiftRed() &&
            !Emergency_Tilt() && !Emergency_LowZAccelerometer()) {
            Emergency_SetState(0);
        } else {
            debug_printf("emergency release rejected: physical sensor still active\r\n");
        }
    }
}

static void on_cmd_vel(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_cmd_vel_t) - 2u) {
        return;
    }

    const pkt_cmd_vel_t *pkt = (const pkt_cmd_vel_t *)data;

    last_cmd_vel_tick = HAL_GetTick();

    if (main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE) {
        return;
    }

    const float vx = pkt->linear_x;
    const float wz = pkt->angular_z;

    /* Differential-drive inverse kinematics — per-wheel linear speed. */
    float left_mps  = vx - wz * WHEEL_BASE * 0.5f;
    float right_mps = vx + wz * WHEEL_BASE * 0.5f;

    if (left_mps  >  MAX_MPS) left_mps  =  MAX_MPS;
    if (left_mps  < -MAX_MPS) left_mps  = -MAX_MPS;
    if (right_mps >  MAX_MPS) right_mps =  MAX_MPS;
    if (right_mps < -MAX_MPS) right_mps = -MAX_MPS;

    /* Hand the target wheel velocities to the PI loop in motors_handler.
     * The mapping to PWM (feedforward + closed-loop correction) lives
     * there now so the integrator can bridge the static-friction
     * deadband on sub-deadband commands. */
    left_target_mps  = left_mps;
    right_target_mps = right_mps;
}

static void on_set_drive_pid(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_set_drive_pid_t) - 2u) {
        return;
    }

    const pkt_set_drive_pid_t *pkt = (const pkt_set_drive_pid_t *)data;

    /* Drive behaviour is safety-relevant: reject the whole packet if any field
     * is non-finite, then clamp each field to a safe range before applying so a
     * bad host value can never make the wheel loop diverge. Both wheels share
     * the gains; the output limit stays fixed at 255 PWM (motor controller max).
     * pid_constrain() comes from pid.hpp. */
    if (!std::isfinite(pkt->kp) || !std::isfinite(pkt->ki) || !std::isfinite(pkt->kd) ||
        !std::isfinite(pkt->integral_limit) || !std::isfinite(pkt->pwm_per_mps) ||
        !std::isfinite(pkt->deadband_pwm) || !std::isfinite(pkt->hold_kp)) {
        debug_printf("set_drive_pid rejected: non-finite field\r\n");
        return;
    }

    const float kp   = pid_constrain(pkt->kp,             0.0f,   200.0f);
    const float ki   = pid_constrain(pkt->ki,             0.0f, 20000.0f);
    const float kd   = pid_constrain(pkt->kd,             0.0f,   500.0f);
    const float ilim = pid_constrain(pkt->integral_limit, 0.0f,   255.0f);
    const float ff   = pid_constrain(pkt->pwm_per_mps,   50.0f,   600.0f);
    /* Breakaway feedforward: clamp well under the 255 PWM ceiling so it can
     * never on its own saturate the motor command. */
    const float db   = pid_constrain(pkt->deadband_pwm,   0.0f,   150.0f);
    const uint8_t use_pi = pkt->wheel_pi_enabled ? 1u : 0u;
    const uint8_t hold  = pkt->hold_enabled ? 1u : 0u;
    const float hkp     = pid_constrain(pkt->hold_kp,     0.0f,    50.0f);

    /* Apply atomically w.r.t. motors_handler(), which reads these objects in the
     * main loop at 50 Hz: this handler runs in USB RX interrupt context, and a
     * half-applied update (e.g. new gains but the old integral limit) could let
     * the ki integrator wind up unbounded for one cycle. Same __disable_irq
     * guard motors_handler uses for its setpoint snapshot. setOutputLimit is
     * re-asserted here so the ±255 clamp never silently depends on init_ROS
     * having run first (the PX4 PID default-inits _limit_output to 0). */
    __disable_irq();
    left_wheel_pid.setGains(kp, ki, kd);
    left_wheel_pid.setIntegralLimit(ilim);
    left_wheel_pid.setOutputLimit(255.0f);
    right_wheel_pid.setGains(kp, ki, kd);
    right_wheel_pid.setIntegralLimit(ilim);
    right_wheel_pid.setOutputLimit(255.0f);
    g_pwm_per_mps = ff;
    g_drive_deadband_pwm = db;
    g_use_wheel_pi = use_pi;
    g_hold_enabled = hold;
    g_hold_kp = hkp;
    __enable_irq();

    debug_printf(
        "set_drive_pid: kp=%.2f ki=%.2f kd=%.2f ilim=%.1f ff=%.1f db=%.1f pi=%u hold=%u hkp=%.2f\r\n",
        kp, ki, kd, ilim, ff, db, (unsigned)use_pi, (unsigned)hold, hkp);
}

static void on_set_detector_params(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_set_detector_params_t) - 2u) {
        return;
    }

    const pkt_set_detector_params_t *pkt = (const pkt_set_detector_params_t *)data;

    /* Reject the whole packet if any field is non-finite, then clamp each to a
     * sane range before applying. These only gate ADVISORY flags (no actuation),
     * so a bad value can't move the robot — but clamp anyway for tidy telemetry.
     * pid_constrain() comes from pid.hpp. */
    if (!std::isfinite(pkt->yaw_thresh_rps) || !std::isfinite(pkt->stall_cmd_mps) ||
        !std::isfinite(pkt->stall_meas_mps) || !std::isfinite(pkt->impact_thresh_mps2) ||
        !std::isfinite(pkt->bog_cmd_mps) || !std::isfinite(pkt->bog_ratio) ||
        !std::isfinite(pkt->jam_load_pwm) || !std::isfinite(pkt->blade_bog_ratio)) {
        debug_printf("set_detector_params rejected: non-finite field\r\n");
        return;
    }

    const float yaw   = pid_constrain(pkt->yaw_thresh_rps,     0.02f,  3.0f);
    const float scmd  = pid_constrain(pkt->stall_cmd_mps,      0.01f,  1.0f);
    const float smeas = pid_constrain(pkt->stall_meas_mps,     0.0f,   0.5f);
    const float imp   = pid_constrain(pkt->impact_thresh_mps2, 1.0f,  50.0f);
    const float bcmd  = pid_constrain(pkt->bog_cmd_mps,        0.01f,  1.0f);
    const float brat  = pid_constrain(pkt->bog_ratio,          0.05f,  0.95f);
    const float jam   = pid_constrain(pkt->jam_load_pwm,       0.0f, 255.0f);
    const float bbrat = pid_constrain(pkt->blade_bog_ratio,    0.05f,  0.95f);

    /* Detectors run in the main loop (50/100 Hz); this handler is USB RX IRQ
     * context. Apply under the same irq guard the drive loop uses so a torn
     * multi-field read can't briefly mix old/new thresholds. */
    __disable_irq();
    g_det_yaw_thresh = yaw;
    g_det_stall_cmd = scmd;
    g_det_stall_meas = smeas;
    g_det_impact_thresh = imp;
    g_det_bog_cmd = bcmd;
    g_det_bog_ratio = brat;
    g_det_jam_load = jam;
    g_det_blade_bog_ratio = bbrat;
    __enable_irq();

    debug_printf("set_detector_params: yaw=%.2f stall_cmd=%.2f stall_meas=%.2f imp=%.1f "
                 "bog_cmd=%.2f bog_ratio=%.2f jam=%.0f blade_bog=%.2f\r\n",
                 yaw, scmd, smeas, imp, bcmd, brat, jam, bbrat);
}

static void on_hl_state(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_hl_state_t) - 2u) {
        return;
    }

    const pkt_hl_state_t *pkt = (const pkt_hl_state_t *)data;

    hl_current_mode = pkt->current_mode;
    hl_gps_quality  = pkt->gps_quality;

    // Update panel LEDs based on mode
    if (hl_gps_quality < 90) {
        PANEL_Set_LED(PANEL_LED_LOCK, PANEL_LED_OFF);
    } else {
        PANEL_Set_LED(PANEL_LED_LOCK, PANEL_LED_ON);
    }

    // Map host mode to internal status for motor safety.
    // Constants defined in mowgli_protocol.h — keep in sync with HighLevelStatus.msg.
    switch (hl_current_mode) {
    case HL_MODE_AUTONOMOUS:
        PANEL_Set_LED(PANEL_LED_S1, PANEL_LED_ON);
        PANEL_Set_LED(PANEL_LED_S2, PANEL_LED_OFF);
        main_eOpenmowerStatus = OPENMOWER_STATUS_MOWING;
        break;
    case HL_MODE_RECORDING:
        PANEL_Set_LED(PANEL_LED_S1, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_S2, PANEL_LED_ON);
        main_eOpenmowerStatus = OPENMOWER_STATUS_RECORD;
        break;
    case HL_MODE_MANUAL_MOWING:
        PANEL_Set_LED(PANEL_LED_S1, PANEL_LED_ON);
        PANEL_Set_LED(PANEL_LED_S2, PANEL_LED_ON);
        main_eOpenmowerStatus = OPENMOWER_STATUS_MOWING;
        break;
    case HL_MODE_NULL:
    case HL_MODE_IDLE:
    default:
        PANEL_Set_LED(PANEL_LED_S1, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_S2, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_4H, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_6H, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_8H, PANEL_LED_OFF);
        main_eOpenmowerStatus = OPENMOWER_STATUS_IDLE;
        left_target_mps = right_target_mps = 0.0f;
        blade_on_off = target_blade_on_off = 0;
        break;
    }

    update_blade_led();
}

static void on_cmd_blade(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_cmd_blade_t) - 2u) {
        return;
    }

    const pkt_cmd_blade_t *pkt = (const pkt_cmd_blade_t *)data;
    /* Defense-in-depth: never arm the blade target while IDLE/docked. The
     * authoritative gate is in motors_handler (which zeroes blade_on_off in
     * IDLE every tick), but refusing to latch the target here keeps state
     * consistent and avoids an instantaneous spin-up on the IDLE→MOWING edge.
     * blade_dir is still accepted so direction is correct once mowing starts. */
    if (main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE) {
        target_blade_on_off = 0;
    } else {
        target_blade_on_off = pkt->blade_on;
    }
    blade_direction = pkt->blade_dir;
}

/* Host -> Firmware reboot request. Sets reboot_flag so chatter_handler issues
 * NVIC_SystemReset on its next tick (lets the current packet/ISR unwind first).
 * Gated on the magic byte so a corrupt/misframed packet can't reset the board. */
static void on_reboot(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_reboot_t) - 2u) {
        return;
    }
    const pkt_reboot_t *pkt = (const pkt_reboot_t *)data;
    if (pkt->magic == PKT_REBOOT_MAGIC) {
        debug_printf("reboot requested by host\r\n");
        reboot_flag = true;
    }
}

/* on_hl_state blade LED feedback (moved out of on_hl_state for clarity) */
static void update_blade_led(void)
{
    if (target_blade_on_off) {
        #ifdef PANEL_LED_2H
        if (BLADEMOTOR_bActivated) {
            PANEL_Set_LED(PANEL_LED_2H, PANEL_LED_FLASH_SLOW);
        } else {
            PANEL_Set_LED(PANEL_LED_2H, PANEL_LED_ON);
        }
        #endif
    } else {
        #ifdef PANEL_LED_2H
        PANEL_Set_LED(PANEL_LED_2H, PANEL_LED_OFF);
        #endif
    }
}

/* ---------------------------------------------------------------------------
 * USB CDC receive callback — feeds COBS layer
 * ---------------------------------------------------------------------------*/
uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len)
{
    mowgli_comms_process_rx(Buf, (size_t)len);
    return CDC_RX_DATA_HANDLED;
}

/* ---------------------------------------------------------------------------
 * usb_cdc_transmit — required by mowgli_comms.c
 * ---------------------------------------------------------------------------*/
void usb_cdc_transmit(const uint8_t *buf, size_t len)
{
    CDC_Transmit(buf, (uint32_t)len);
}

/* ---------------------------------------------------------------------------
 * LED blink + reboot handler (replaces chatter_handler)
 * ---------------------------------------------------------------------------*/
extern "C" void chatter_handler()
{
    if (NBT_handler(&led_nbt)) {
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);

        if (reboot_flag) {
            NVIC_SystemReset();
        }
    }
}

/* Static-friction breakaway feedforward. When commanding motion, bias an
 * open-loop PWM term by sign(target) * g_drive_deadband_pwm so the wheel
 * crosses the PAC5210 breakaway immediately instead of waiting for a loop to
 * wind up. No bias when the target is ~0 (a stopped wheel stays stopped). */
static inline float apply_deadband_ff(float pwm, float target_mps)
{
    const float db = g_drive_deadband_pwm;
    if (db > 0.0f && fabsf(target_mps) > 1.0e-3f) {
        pwm += (target_mps > 0.0f) ? db : -db;
    }
    return pwm;
}

/* Position-hold corrective PWM for one wheel from its tick error (latched
 * reference minus current). A small deadzone avoids buzzing on encoder
 * quantization; the breakaway supplies the static-friction floor so even a
 * one-tick error can push back, and hold_kp scales the rest. Clamped well
 * under the motion PWM range. */
static inline int16_t hold_pwm(int32_t err_ticks)
{
    if (err_ticks > -HOLD_TOL_TICKS && err_ticks < HOLD_TOL_TICKS) {
        return 0;
    }
    const float floor_pwm = (err_ticks > 0) ? g_drive_deadband_pwm : -g_drive_deadband_pwm;
    float pwm = floor_pwm + g_hold_kp * (float)err_ticks;
    if (pwm > HOLD_MAX_PWM) {
        pwm = HOLD_MAX_PWM;
    }
    if (pwm < -HOLD_MAX_PWM) {
        pwm = -HOLD_MAX_PWM;
    }
    return (int16_t)pwm;
}

/* ---------------------------------------------------------------------------
 * Drive & blade motors handler
 * ---------------------------------------------------------------------------*/
extern "C" void motors_handler()
{
    if (NBT_handler(&motors_nbt)) {
        /* Snapshot ISR-written variables under interrupt lock */
        __disable_irq();
        float    snap_left_target  = left_target_mps;
        float    snap_right_target = right_target_mps;
        uint8_t  snap_target_blade = target_blade_on_off;
        uint32_t snap_heartbeat    = last_heartbeat_tick;
        uint32_t snap_cmd_vel      = last_cmd_vel_tick;
        __enable_irq();

        blade_on_off = snap_target_blade;

        /* --- decide effective target ---
         * Emergency or cmd_vel watchdog timeout overrides to a hard stop.
         * Otherwise the snapshot value drives the PI loop below. */
        bool hard_stop = false;
        if (Emergency_State()) {
            hard_stop = true;
            blade_on_off = 0;
        } else if (main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE) {
            /* Re-assert the IDLE gate HERE — in the one place that actually
             * drives the wheels AND the blade — so the "never move / never
             * spin the blade while idle/docked" guarantee holds regardless of
             * the relative arrival order of CMD_PWM, CMD_BLADE and HL_STATE.
             * on_hl_state zeroes the targets when the IDLE packet arrives, but
             * a CMD_BLADE(on=1) arriving AFTER it would otherwise re-arm the
             * blade with no gate (on_cmd_blade is fire-and-forget). Firmware is
             * the sole blade safety authority. */
            hard_stop = true;
            blade_on_off = 0;
        } else {
            const uint32_t cmd_vel_age_ms = HAL_GetTick() - snap_cmd_vel;
            if (cmd_vel_age_ms > 200u) {
                /* Command-vel watchdog: zero motors if the host hasn't
                 * sent a twist in 200 ms (Pi hang, USB glitch, etc). */
                hard_stop = true;
            }
            if (cmd_vel_age_ms > 25000u) {
                blade_on_off = 0;
            }
        }

        const float l_target = hard_stop ? 0.0f : snap_left_target;
        const float r_target = hard_stop ? 0.0f : snap_right_target;

        /* Standstill position hold. While actively controlled (not hard_stop,
         * i.e. not IDLE / emergency / cmd_vel watchdog) and commanded to ~0 on
         * both wheels, latch the encoder position and push back against creep
         * instead of coasting. Mutually exclusive with the velocity loop so the
         * PI state stays clean across the hold. Released to coast by hard_stop
         * (the "really idle" paths) below. */
        static bool was_holding = false;
        static int32_t hold_l_ref = 0;
        static int32_t hold_r_ref = 0;
        const bool want_hold = (g_hold_enabled != 0u) && !hard_stop &&
                               fabsf(l_target) < 1.0e-3f && fabsf(r_target) < 1.0e-3f;

      if (want_hold) {
        const int32_t cur_l = left_ticks_signed;
        const int32_t cur_r = right_ticks_signed;
        if (!was_holding) {
            hold_l_ref = cur_l;
            hold_r_ref = cur_r;
            // Drop velocity-loop state so it resumes clean when hold releases.
            left_wheel_pid.resetIntegral();
            left_wheel_pid.resetDerivative();
            right_wheel_pid.resetIntegral();
            right_wheel_pid.resetDerivative();
            prev_left_target_mps = 0.0f;
            prev_right_target_mps = 0.0f;
            prev_left_ticks_signed_pi = cur_l;
            prev_right_ticks_signed_pi = cur_r;
            was_holding = true;
        }
        left_pwm_signed = hold_pwm(hold_l_ref - cur_l);
        right_pwm_signed = hold_pwm(hold_r_ref - cur_r);
      } else if (g_use_wheel_pi) {
        was_holding = false;
        /* Wheel-level PI loop.
         *
         * Reads the signed cumulative encoder count maintained by
         * drivemotor.c, derives actual_mps over the 20 ms loop
         * period, computes a feedforward + PI PWM. With the deadband
         * breakaway feedforward (g_drive_deadband_pwm) the integrator no
         * longer has to wind up to cross the static-friction deadband — the
         * breakaway PWM is applied immediately — so the PI only trims the
         * residual velocity error, eliminating the stick-slip lurch. When the
         * breakaway is 0 (default) the integrator still bridges the deadband
         * the old way.
         *
         * Read left_ticks_signed/right_ticks_signed directly (these are
         * 32-bit and updated from the drivemotor rx-decode path —
         * not strictly atomic, but a torn read here costs at most
         * one 20 ms loop of incorrect velocity, then converges). */
        const int32_t cur_left_ticks  = left_ticks_signed;
        const int32_t cur_right_ticks = right_ticks_signed;
        const int32_t dleft_ticks  = cur_left_ticks  - prev_left_ticks_signed_pi;
        const int32_t dright_ticks = cur_right_ticks - prev_right_ticks_signed_pi;
        prev_left_ticks_signed_pi  = cur_left_ticks;
        prev_right_ticks_signed_pi = cur_right_ticks;

        const float l_actual_mps =
            ((float)dleft_ticks)  / WHEEL_PI_TICKS_PER_M / WHEEL_PI_DT_S;
        const float r_actual_mps =
            ((float)dright_ticks) / WHEEL_PI_TICKS_PER_M / WHEEL_PI_DT_S;

        /* Reset the integrator on direction reversal / stop-to-go / hard-stop.
         * Without this the integral built up while decelerating would drive the
         * motor backwards as soon as the chassis stopped (micro-oscillation). */
        const bool l_target_sign_changed =
            (l_target * prev_left_target_mps  < 0.0f) ||
            (l_target == 0.0f && prev_left_target_mps  != 0.0f) ||
            hard_stop;
        const bool r_target_sign_changed =
            (r_target * prev_right_target_mps < 0.0f) ||
            (r_target == 0.0f && prev_right_target_mps != 0.0f) ||
            hard_stop;
        if (l_target_sign_changed) {
            left_wheel_pid.resetIntegral();
            left_wheel_pid.resetDerivative();
        }
        if (r_target_sign_changed) {
            right_wheel_pid.resetIntegral();
            right_wheel_pid.resetDerivative();
        }
        prev_left_target_mps  = l_target;
        prev_right_target_mps = r_target;

        /* Conditional-integration anti-windup, DIRECTION-AWARE: freeze the
         * integrator only in the direction that would worsen an already-
         * saturated output; always allow it to unwind OUT of saturation. Keying
         * on the error sign (not just the saturation bit) avoids a one-
         * directional integral latch — e.g. on overspeed (err < 0) while the
         * output is railed high, the integrator must still be able to wind down
         * to cut PWM. *_pwm_signed is the previous cycle's total (feedforward +
         * trim), saturated by DRIVEMOTOR_SetSpeedSigned at ±255. This sits on
         * top of the PID's own ±100 integral-magnitude clamp. */
        const float l_err = l_target - l_actual_mps;
        const float r_err = r_target - r_actual_mps;
        const bool l_update_integral =
            !((left_pwm_signed >= 255 && l_err > 0.0f) || (left_pwm_signed <= -255 && l_err < 0.0f));
        const bool r_update_integral =
            !((right_pwm_signed >= 255 && r_err > 0.0f) || (right_pwm_signed <= -255 && r_err < 0.0f));

        /* Closed-loop PI trim (Kp·err + integrator; D gain = 0). The PID
         * computes error = setpoint − feedback internally and integrates AFTER
         * forming the output (PX4 form), so a fresh integral increment reaches
         * the actuator one 50 Hz cycle later than the old integrate-before form
         * — steady-state identical, ~20 ms transient shift (immaterial here). */
        left_wheel_pid.setSetpoint(l_target);
        right_wheel_pid.setSetpoint(r_target);
        const float l_trim = left_wheel_pid.update(l_actual_mps, WHEEL_PI_DT_S, l_update_integral);
        const float r_trim = right_wheel_pid.update(r_actual_mps, WHEEL_PI_DT_S, r_update_integral);

        /* Open-loop feedforward (velocity->PWM scale + static-friction
         * breakaway) + closed-loop PI trim. Sign carried through. */
        const float l_pwm_f = apply_deadband_ff(l_target * g_pwm_per_mps, l_target) + l_trim;
        const float r_pwm_f = apply_deadband_ff(r_target * g_pwm_per_mps, r_target) + r_trim;

        /* When the target is exactly zero AND we're not braking from a
         * larger speed, force PWM to zero outright — avoids the residual
         * "hum" from a non-zero integral applied to a stopped wheel. */
        left_pwm_signed  = (l_target == 0.0f && fabsf(l_actual_mps) < 0.02f)
                          ? 0
                          : (int16_t)l_pwm_f;
        right_pwm_signed = (r_target == 0.0f && fabsf(r_actual_mps) < 0.02f)
                          ? 0
                          : (int16_t)r_pwm_f;
      } else {
        was_holding = false;
        /* Open-loop feedforward only (no encoder feedback, no integrator):
         * velocity->PWM scale plus the static-friction breakaway feedforward,
         * which is what lets a sub-deadband target still reach the wheels in
         * this mode instead of buzzing below breakaway. */
        left_pwm_signed  = (int16_t)apply_deadband_ff(l_target * g_pwm_per_mps, l_target);
        right_pwm_signed = (int16_t)apply_deadband_ff(r_target * g_pwm_per_mps, r_target);
      }

        if (hard_stop) {
            DRIVEMOTOR_SetSpeedSigned(0, 0);
        } else {
            DRIVEMOTOR_SetSpeedSigned(left_pwm_signed, right_pwm_signed);
        }

        /* Drive-loop telemetry (~25 Hz = every other 50 Hz cycle): commanded
         * target velocity + the signed PWM actually sent, so the host can
         * watch the velocity->PWM mapping (breakaway, PI trim) while tuning. */
        static uint8_t telem_div = 0u;
        if ((telem_div++ & 1u) == 0u) {
            const int16_t sent_l = hard_stop ? 0 : left_pwm_signed;
            const int16_t sent_r = hard_stop ? 0 : right_pwm_signed;
            pkt_drive_telem_t telem;
            telem.type              = PKT_ID_DRIVE_TELEM;
            telem.left_target_mm_s  = (int16_t)(l_target * 1000.0f);
            telem.right_target_mm_s = (int16_t)(r_target * 1000.0f);
            telem.left_pwm          = sent_l;
            telem.right_pwm         = sent_r;
            telem.wheel_yaw_mrad_s  = g_wheel_yaw_mrad_s;
            telem.imu_yaw_mrad_s    = g_imu_yaw_mrad_s;
            telem.accel_peak_mg     = g_accel_peak_mg;
            telem.left_load         = left_power;   // drive-controller load bytes
            telem.right_load        = right_power;  // (extern from drivemotor.c)
            telem.slip_flags        = g_slip_flags;
            mowgli_comms_send(&telem, sizeof(telem));
        }

        // Heartbeat watchdog: if no heartbeat for HEARTBEAT_TIMEOUT_MS, emergency stop
        if (snap_heartbeat != 0 &&
            (HAL_GetTick() - snap_heartbeat) > HEARTBEAT_TIMEOUT_MS) {
            Emergency_SetState(1);
        }

        BLADEMOTOR_Set(blade_on_off, blade_direction);
    }
}

/* ---------------------------------------------------------------------------
 * Panel handler — button presses generate UI events over COBS
 * ---------------------------------------------------------------------------*/
extern "C" void panel_handler()
{
    if (NBT_handler(&panel_nbt)) {
        PANEL_Tick();

        if (buttonupdated == 1 && buttoncleared == 0) {
            pkt_ui_event_t evt;
            evt.type = PKT_ID_UI_EVENT;
            evt.press_duration = 0;  // short press

            // Map physical buttons to IDs
            if (buttonstate[PANEL_BUTTON_DEF_S1]) {
                evt.button_id = 1;
                mowgli_comms_send(&evt, sizeof(evt));
            }
            if (buttonstate[PANEL_BUTTON_DEF_S2]) {
                evt.button_id = 2;
                mowgli_comms_send(&evt, sizeof(evt));
            }
            if (buttonstate[PANEL_BUTTON_DEF_LOCK]) {
                evt.button_id = 3;
                mowgli_comms_send(&evt, sizeof(evt));
            }
            if (buttonstate[PANEL_BUTTON_DEF_START]) {
                evt.button_id = 4;
                mowgli_comms_send(&evt, sizeof(evt));
            }
            if (buttonstate[PANEL_BUTTON_DEF_HOME]) {
                evt.button_id = 5;
                mowgli_comms_send(&evt, sizeof(evt));
            }

            buttonupdated = 0;
        }
    }
}

#if OPTION_ULTRASONIC == 1
extern "C" void ultrasonic_handler(void)
{
    // USS data is included in the status packet — no separate packet needed.
    // This handler is kept for the main loop call in main.c.
}
#endif

/* ---------------------------------------------------------------------------
 * Wheel ticks handler — called from DRIVEMOTOR_App_Rx() every 20 ms.
 *
 * Builds the odometry packet from signed cumulative ticks. Per-wheel
 * velocity is computed here (not on the Pi) because the firmware has the
 * hardware-timer-accurate dt. All four quantities in the packet are
 * signed; the host doesn't need a direction byte or to re-sign anything.
 * ---------------------------------------------------------------------------*/
extern "C" void wheelTicks_handler(
    int32_t  p_s32LeftTicksSigned,
    int32_t  p_s32RightTicksSigned,
    int16_t  p_s16LeftSpeed,   /* currently unused — reserved for future telemetry */
    int16_t  p_s16RightSpeed)
{
    (void)p_s16LeftSpeed;
    (void)p_s16RightSpeed;

    static int32_t prev_left_ticks  = 0;
    static int32_t prev_right_ticks = 0;

    const uint32_t now_tick = HAL_GetTick();
    const uint16_t dt_ms    = (uint16_t)(now_tick - last_odom_tick);
    last_odom_tick = now_tick;

    const int32_t delta_left  = p_s32LeftTicksSigned  - prev_left_ticks;
    const int32_t delta_right = p_s32RightTicksSigned - prev_right_ticks;
    prev_left_ticks  = p_s32LeftTicksSigned;
    prev_right_ticks = p_s32RightTicksSigned;

    /* Velocity: mm/s = (delta_ticks / TICKS_PER_M) * (1000 / dt_ms) * 1000
     *                = delta_ticks * 1e6 / (TICKS_PER_M * dt_ms).
     * TICKS_PER_M is 300, so the constant numerator (300 * dt_ms) stays
     * comfortably inside int32 for any realistic dt. We still cast to
     * int64 for the mul to be safe on large tick deltas.                  */
    int16_t left_v_mm_s  = 0;
    int16_t right_v_mm_s = 0;
    if (dt_ms > 0)
    {
        const int64_t denom = (int64_t)TICKS_PER_M * (int64_t)dt_ms;
        int64_t v_l = ((int64_t)delta_left  * 1000000LL) / denom;
        int64_t v_r = ((int64_t)delta_right * 1000000LL) / denom;
        if (v_l >  32767) v_l =  32767;
        if (v_l < -32768) v_l = -32768;
        if (v_r >  32767) v_r =  32767;
        if (v_r < -32768) v_r = -32768;
        left_v_mm_s  = (int16_t)v_l;
        right_v_mm_s = (int16_t)v_r;
    }

    pkt_odometry_t odom;
    odom.type                 = PKT_ID_ODOMETRY;
    odom.dt_millis            = dt_ms;
    odom.left_ticks           = p_s32LeftTicksSigned;
    odom.right_ticks          = p_s32RightTicksSigned;
    odom.left_velocity_mm_s   = left_v_mm_s;
    odom.right_velocity_mm_s  = right_v_mm_s;

    mowgli_comms_send_odometry(&odom);

    /* IMU-to-odometry discrepancy detector (see the SLIP_* block above). */
    update_slip_detector(left_v_mm_s, right_v_mm_s);
}

/// @brief Compare the wheel-implied chassis yaw rate to the IMU gyro and flag a
///        discrepancy (stick-slip / wheel-slip / impact), plus a stall flag when
///        a real command produces no wheel motion (obstruction / collision).
///
/// Results are stashed in g_wheel_yaw_mrad_s / g_imu_yaw_mrad_s / g_slip_flags
/// for the drive-telem packet. Pure integer/float math, no I/O.
static void update_slip_detector(int16_t left_v_mm_s, int16_t right_v_mm_s)
{
    const float v_l = (float)left_v_mm_s / 1000.0f;   // m/s
    const float v_r = (float)right_v_mm_s / 1000.0f;
    const float wheel_yaw = (v_r - v_l) / (float)WHEEL_BASE;  // rad/s, CCW+
    const float gyro_yaw = g_imu_gyro_z;                      // rad/s, raw

    // Low-passed |residual| with a short debounce so a single noisy frame can't
    // raise the flag, but a sustained disagreement does within ~60 ms.
    static float resid_ema = 0.0f;
    static uint8_t yaw_count = 0u;
    static uint8_t stall_count = 0u;
    resid_ema += SLIP_EMA_ALPHA * (fabsf(wheel_yaw - gyro_yaw) - resid_ema);
    if (resid_ema > g_det_yaw_thresh) {
        if (yaw_count < SLIP_YAW_PERSIST) {
            yaw_count++;
        }
    } else {
        yaw_count = 0u;
    }

    // Stall: a meaningful forward/turn command but the wheels aren't moving.
    const float cmd = (fabsf(left_target_mps) + fabsf(right_target_mps)) * 0.5f;
    const float meas = (fabsf(v_l) + fabsf(v_r)) * 0.5f;
    static uint8_t bog_count = 0u;
    if (cmd > g_det_stall_cmd && meas < g_det_stall_meas) {
        if (stall_count < STALL_PERSIST) {
            stall_count++;
        }
    } else {
        stall_count = 0u;
    }

    // Bog (soft resistance / high grass): the wheels are turning but well below
    // the commanded speed, and it isn't a hard impact — the chassis is loaded
    // and barely advancing. Excludes the stall case (meas above the stall floor).
    const bool impact_active = (g_impact_hold > 0u);
    if (cmd > g_det_bog_cmd && meas >= g_det_stall_meas && meas < g_det_bog_ratio * cmd &&
        !impact_active) {
        if (bog_count < BOG_PERSIST) {
            bog_count++;
        }
    } else {
        bog_count = 0u;
    }

    uint8_t flags = 0u;
    if (yaw_count >= SLIP_YAW_PERSIST) {
        flags |= DRIVE_SLIP_FLAG_YAW;
    }
    if (stall_count >= STALL_PERSIST) {
        flags |= DRIVE_SLIP_FLAG_STALL;
        // Jam vs deadband: the drive-controller load byte is a commanded-effort
        // proxy (≈0 only when uncommanded), so a stall with the motor still
        // driving hard (max wheel load ≥ threshold) is a real obstruction,
        // whereas ≈0 load is a benign deadband stall. left_power/right_power are
        // extern from drivemotor.c (the PAC5210 per-wheel load bytes).
        const uint8_t load_max = (left_power > right_power) ? left_power : right_power;
        if ((float)load_max >= g_det_jam_load) {
            flags |= DRIVE_SLIP_FLAG_JAM;
        }
    }
    if (bog_count >= BOG_PERSIST) {
        flags |= DRIVE_SLIP_FLAG_BOG;
    }
    if (impact_active) {
        flags |= DRIVE_SLIP_FLAG_IMPACT;
    }
    if (g_blade_bog) {
        flags |= DRIVE_SLIP_FLAG_BLADE_BOG;
    }

    g_wheel_yaw_mrad_s = (int16_t)pid_constrain(wheel_yaw * 1000.0f, -32767.0f, 32767.0f);
    g_imu_yaw_mrad_s = (int16_t)pid_constrain(gyro_yaw * 1000.0f, -32767.0f, 32767.0f);
    g_slip_flags = flags;
}

/// @brief Hard-collision detector: a peak in the dynamic acceleration (|accel|
///        deviation from a slow gravity baseline). Magnitude-based, so it is
///        independent of IMU mounting. Runs at the 100 Hz IMU cadence; latches
///        g_impact_hold for ~300 ms and reports the peak magnitude in milli-g.
static void update_impact_detector(float ax, float ay, float az)
{
    const float amag = sqrtf(ax * ax + ay * ay + az * az);
    static float baseline = 9.81f;  // slow EMA tracks gravity through any tilt
    baseline += IMPACT_BASELINE_ALPHA * (amag - baseline);
    const float dev = fabsf(amag - baseline);

    static float peak = 0.0f;
    peak = (dev > peak) ? dev : peak * IMPACT_PEAK_DECAY;
    g_accel_peak_mg = (int16_t)pid_constrain(peak / 9.81f * 1000.0f, 0.0f, 32767.0f);

    if (dev > g_det_impact_thresh) {
        g_impact_hold = IMPACT_HOLD_TICKS;
    } else if (g_impact_hold > 0u) {
        g_impact_hold--;
    }
}

/// @brief Blade-bog detector: blade commanded on but its RPM has collapsed
///        relative to its own free-running maximum this session (high grass
///        loading the blade → the operator should slow the advance rate).
///        Self-calibrating, so no absolute nominal-RPM constant is needed.
static void update_blade_bog_detector(void)
{
    static uint16_t rpm_max = 0u;
    static uint8_t bog_count = 0u;
    if (!blade_on_off) {
        rpm_max = 0u;
        bog_count = 0u;
        g_blade_bog = 0u;
        return;
    }
    const uint16_t rpm = BLADEMOTOR_u16RPM;
    if (rpm > rpm_max) {
        rpm_max = rpm;  // track the free-running peak
    } else if (rpm_max > BLADE_MAX_DECAY) {
        rpm_max -= BLADE_MAX_DECAY;  // slow decay so it adapts to a lower no-load speed
    }
    const bool spun_up = rpm_max > BLADE_RPM_FLOOR;
    if (spun_up && (float)rpm < g_det_blade_bog_ratio * (float)rpm_max) {
        if (bog_count < BLADE_BOG_PERSIST) {
            bog_count++;
        }
    } else {
        bog_count = 0u;
    }
    g_blade_bog = (bog_count >= BLADE_BOG_PERSIST) ? 1u : 0u;
}

/* ---------------------------------------------------------------------------
 * IMU + status broadcast handler
 * ---------------------------------------------------------------------------*/
extern "C" void broadcast_handler()
{
    if (NBT_handler(&imu_nbt)) {
        pkt_imu_t imu_pkt;
        imu_pkt.type = PKT_ID_IMU;

        static uint32_t last_imu_tick = 0;
        uint32_t now_tick = HAL_GetTick();
        imu_pkt.dt_millis = (uint16_t)(now_tick - last_imu_tick);
        last_imu_tick = now_tick;

#ifdef EXTERNAL_IMU_ACCELERATION
        float ax, ay, az;
        IMU_ReadAccelerometer(&ax, &ay, &az);
        imu_pkt.acceleration_mss[0] = ax;
        imu_pkt.acceleration_mss[1] = ay;
        imu_pkt.acceleration_mss[2] = az;
        update_impact_detector(ax, ay, az);  // hard-collision peak detector
#else
        imu_pkt.acceleration_mss[0] = 0.0f;
        imu_pkt.acceleration_mss[1] = 0.0f;
        imu_pkt.acceleration_mss[2] = 0.0f;
#endif

#ifdef EXTERNAL_IMU_ANGULAR
        float gx, gy, gz;
        IMU_ReadGyro(&gx, &gy, &gz);
        imu_pkt.gyro_rads[0] = gx;
        imu_pkt.gyro_rads[1] = gy;
        imu_pkt.gyro_rads[2] = gz;
        g_imu_gyro_z = gz;  // feed the IMU-to-odometry discrepancy detector
#else
        imu_pkt.gyro_rads[0] = 0.0f;
        imu_pkt.gyro_rads[1] = 0.0f;
        imu_pkt.gyro_rads[2] = 0.0f;
#endif

        // Magnetometer — uses generic IMU_ReadMag (works with any IMU that has mag)
        IMU_ReadMag(&imu_pkt.mag_uT[0], &imu_pkt.mag_uT[1], &imu_pkt.mag_uT[2]);

        update_blade_bog_detector();  // blade RPM collapse under high-grass load

        mowgli_comms_send_imu(&imu_pkt);
    }

    if (NBT_handler(&status_nbt)) {
        pkt_status_t status_pkt;
        status_pkt.type = PKT_ID_STATUS;

        // Build status bitmask
        uint8_t status_bits = STATUS_BIT_INITIALIZED | STATUS_BIT_RASPI_POWER;
        if (chargecontrol_is_charging) {
            status_bits |= STATUS_BIT_CHARGING;
        }
        if (RAIN_Sense()) {
            status_bits |= STATUS_BIT_RAIN;
        }
        // Sound and UI availability from panel
        status_bits |= STATUS_BIT_UI_AVAIL;
        status_pkt.status_bitmask = status_bits;

        // USS ranges — fill from ultrasonic sensors
        for (unsigned int i = 0; i < MOWGLI_USS_COUNT; i++) {
            status_pkt.uss_ranges_m[i] = 0.0f;
        }
#if OPTION_ULTRASONIC == 1
        status_pkt.uss_ranges_m[0] = (float)(ULTRASONICSENSOR_u32GetLeftDistance()) / 10000.0f;
        status_pkt.uss_ranges_m[1] = (float)(ULTRASONICSENSOR_u32GetRightDistance()) / 10000.0f;
#endif

        // Emergency bitmask
        uint8_t emergency_bits = 0u;
        if (Emergency_State()) {
            emergency_bits |= EMERGENCY_BIT_LATCH;
            if (Emergency_StopButtonYellow() || Emergency_StopButtonWhite()) {
                emergency_bits |= EMERGENCY_BIT_STOP;
            }
            if (Emergency_WheelLiftBlue() || Emergency_WheelLiftRed()) {
                emergency_bits |= EMERGENCY_BIT_LIFT;
            }
        }
        status_pkt.emergency_bitmask = emergency_bits;

        // Power
        status_pkt.v_charge         = charge_voltage;
        status_pkt.v_system         = battery_voltage;
        status_pkt.charging_current = current;
        status_pkt.batt_percentage  = 0;  // TODO: compute from voltage curve

        mowgli_comms_send_status(&status_pkt);
    }

    // Blade motor status (4 Hz) — only after system has initialized
    if (NBT_handler(&blade_nbt) && last_heartbeat_tick != 0u) {
        pkt_blade_status_t blade_pkt;
        memset(&blade_pkt, 0, sizeof(blade_pkt));
        blade_pkt.type        = PKT_ID_BLADE_STATUS;
        blade_pkt.is_active   = BLADEMOTOR_bActivated ? 1u : 0u;
        blade_pkt.rpm         = BLADEMOTOR_u16RPM;
        blade_pkt.power_watts = BLADEMOTOR_u16Power;
        blade_pkt.temperature = blade_temperature;
        blade_pkt.error_count = BLADEMOTOR_u32Error;
        mowgli_comms_send(&blade_pkt, sizeof(blade_pkt));
    }
}

/* ---------------------------------------------------------------------------
 * spinOnce — no-op (rosserial spin removed)
 * ---------------------------------------------------------------------------*/
extern "C" void spinOnce()
{
    // Nothing to do — COBS RX is handled in CDC_DataReceivedHandler().
    // This function is kept so main.c doesn't need modification.
}

/* ---------------------------------------------------------------------------
 * Initialisation (replaces init_ROS)
 * ---------------------------------------------------------------------------*/
extern "C" void init_ROS()
{
    // Initialise COBS comms layer
    mowgli_comms_init();

    // Register handlers for Host -> Firmware packets
    mowgli_comms_register_handler(PKT_ID_HEARTBEAT, on_heartbeat);
    mowgli_comms_register_handler(PKT_ID_CMD_VEL,   on_cmd_vel);
    mowgli_comms_register_handler(PKT_ID_HL_STATE,  on_hl_state);
    mowgli_comms_register_handler(PKT_ID_CMD_BLADE, on_cmd_blade);
    mowgli_comms_register_handler(PKT_ID_REBOOT,    on_reboot);
    mowgli_comms_register_handler(PKT_ID_SET_DRIVE_PID, on_set_drive_pid);
    mowgli_comms_register_handler(PKT_ID_SET_DETECTOR_PARAMS, on_set_detector_params);

    // Initialise timers
    NBT_init(&led_nbt,     LED_NBT_TIME_MS);
    NBT_init(&panel_nbt,   PANEL_NBT_TIME_MS);
    NBT_init(&status_nbt,  STATUS_NBT_TIME_MS);
    NBT_init(&imu_nbt,     IMU_NBT_TIME_MS);
    NBT_init(&motors_nbt,  MOTORS_NBT_TIME_MS);
    NBT_init(&blade_nbt,   BLADE_NBT_TIME_MS);

    // Per-wheel velocity PI gains/limits (vendored PX4 PID, pid.hpp). D=0 — no
    // derivative on a velocity loop. Gains/limits are in PWM units, matching the
    // hand-rolled loop they replace (Kp·err + integrator, integral clamp ±100,
    // output clamp ±255). The PID adds derivative-on-measurement (unused at D=0)
    // and conditional-integration anti-windup. Always initialised (not gated on
    // the compile-time default) because g_use_wheel_pi can enable the loop at
    // runtime via PKT_ID_SET_DRIVE_PID — uninitialised gains would be zero.
    left_wheel_pid.setGains(WHEEL_PI_KP_PWM_PER_MPS, WHEEL_PI_KI_PWM_PER_MPS_S, 0.0f);
    left_wheel_pid.setIntegralLimit(WHEEL_PI_INT_MAX_PWM);
    left_wheel_pid.setOutputLimit(255.0f);
    right_wheel_pid.setGains(WHEEL_PI_KP_PWM_PER_MPS, WHEEL_PI_KI_PWM_PER_MPS_S, 0.0f);
    right_wheel_pid.setIntegralLimit(WHEEL_PI_INT_MAX_PWM);
    right_wheel_pid.setOutputLimit(255.0f);

    last_odom_tick      = HAL_GetTick();
    last_heartbeat_tick = 0;
    last_cmd_vel_tick   = 0;
}

float clamp(float d, float min, float max)
{
    const float t = d < min ? min : d;
    return t > max ? max : t;
}
