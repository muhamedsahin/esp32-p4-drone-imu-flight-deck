#include "fake_esp.hpp"
#include "imu_text_store.hpp"
#include "mpu6050.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace
{
int checks = 0;
void check(bool ok, const char *label)
{
    ++checks;
    if (!ok)
    {
        std::printf("FAIL: %s\n", label);
        std::exit(1);
    }
}
imu_calibration::Config fast()
{
    imu_calibration::Config c{};
    c.warmup_ms = 0;
    c.settle_samples = c.gyro_samples = c.face_samples = c.validation_samples = 20;
    c.pose_timeout_ms = 1500;
    return c;
}
} // namespace
int main()
{
    using namespace imu_calibration;
    TextCalibration state{};
    std::string error;
    const std::string empty = "format = 1\nsensor = 112\nx = null\n";
    check(parseCalibrationText(empty, state, error) && !state.axes[0].present, "null means missing");
    check(parseCalibrationText("# comment\r\nformat=1\r\nsensor=112\r\ngyro_x=0 # zero is valid\r\n", state,
                               error) &&
              state.gyro_axes[0].present && state.gyro_axes[0].value[0] == 0,
          "zero differs from null and CRLF accepted");
    check(!parseCalibrationText(empty + "x=1,2\n", state, error), "duplicate key rejected");
    check(!parseCalibrationText(empty + "oops=1\n", state, error), "unknown key rejected");
    check(!parseCalibrationText(empty + "y=nan,16384\n", state, error), "nonfinite number rejected");
    check(!parseCalibrationText(empty + "y=1,16384,2\n", state, error), "extra vector member rejected");
    check(!parseCalibrationText(empty + "y=1,16384oops\n", state, error), "trailing garbage rejected");
    check(!parseCalibrationText("format=1\nx=null\n", state, error), "sensor identity mandatory");
    check(!parseCalibrationText(std::string(9000, ' '), state, error), "oversized source rejected");
    fake::reset();
    fake::state.identity = 0x70;
    TextStore store;
    check(store.load(empty, state, error), "first source imported");
    {
        MPU6050 sensor;
        check(sensor.init() && sensor.calibrateFromText(state, TextStore::checkpoint, &store, fast()),
              "all null stages calibrated");
        check(fake::state.face_requests == 6 && fake::state.tilt_requests == 3,
              "new file captures six faces and three holdouts");
        check(state.accel_quality.present && state.gyro_axes[0].present, "all completed values filled");
    }
    const auto full = state;
    TextCalibration decoded{};
    check(parseCalibrationText(calibrationText(full), decoded, error) &&
              decoded.axes[0].value == full.axes[0].value,
          "text round trip keeps precision");
    {
        TextStore reboot;
        check(reboot.load(empty, state, error) && state.accel_quality.present,
              "unchanged seed preserves newer NVS result");
        MPU6050 sensor;
        fake::state.face_requests = fake::state.tilt_requests = 0;
        check(sensor.init(), "reboot sensor initialized");
        const auto before = fake::state.bursts;
        check(sensor.calibrateFromText(state, TextStore::checkpoint, &reboot, fast()),
              "complete file skips calibration");
        check(fake::state.face_requests == 0 && fake::state.tilt_requests == 0 &&
                  fake::state.bursts - before == 1,
              "complete record needs only temperature read, no calibration window");
    }
    state = full;
    state.axes[0].present = false;
    const auto edited = calibrationText(state);
    {
        TextStore changed;
        check(changed.load(edited, state, error), "edited x=null source imported once");
        check(!state.faces[0].present && !state.faces[1].present &&
                  state.faces[2].value == full.faces[2].value && state.faces[5].value == full.faces[5].value,
              "only X faces invalidated, Y/Z preserved");
        MPU6050 sensor;
        fake::state.face_requests = fake::state.tilt_requests = 0;
        fake::state.blocked_tilt_pose = 1;
        check(sensor.init() && !sensor.calibrateFromText(state, TextStore::checkpoint, &changed, fast()),
              "partial session pauses at XZ");
        check(fake::state.face_requests == 2 && state.tilts[0].present && !state.tilts[1].present,
              "x=null captures only two faces and checkpoints completed XY");
        check(!sensor.accelerometerCalibration().calibrated, "unvalidated candidate not active");
    }
    {
        TextStore resumed;
        check(resumed.load(edited, state, error) && state.tilts[0].present,
              "partial checkpoint survives restart");
        MPU6050 sensor;
        fake::state.face_requests = fake::state.tilt_requests = 0;
        fake::state.blocked_tilt_pose = -1;
        check(sensor.init() && sensor.calibrateFromText(state, TextStore::checkpoint, &resumed, fast()),
              "resume finishes remaining holdouts");
        check(fake::state.face_requests == 0 && fake::state.tilt_requests == 2,
              "resume repeats neither faces nor XY");
        fake::state.temperature += 2004;
        MPU6050 warm_sensor;
        check(warm_sensor.init() &&
                  warm_sensor.calibrateFromText(state, TextStore::checkpoint, &resumed, fast()),
              "gyro refreshes when reference temperature differs by more than 5 C");
        check(state.gyro_temperature.value[0] > full.gyro_temperature.value[0] + 5,
              "updated gyro temperature saved");
        fake::state.temperature -= 2004;
    }
    {
        TextStore storage;
        fake::state.commit_failure = true;
        check(!storage.save(state), "checkpoint write failure reported");
        fake::state.commit_failure = false;
        fake::state.other_stored["text_v1"][5] ^= 1;
        check(!storage.load(edited, state, error), "CRC covers both source hash and text");
    }
    fake::reset();
    fake::state.identity = 0x70;
    {
        TextStore single_face;
        check(single_face.load(edited, state, error), "single-face resume source loaded");
        MPU6050 sensor;
        fake::state.blocked_face = 1;
        check(sensor.init() && !sensor.calibrateFromText(state, TextStore::checkpoint, &single_face, fast()),
              "session pauses before negative X");
        check(state.faces[0].present && !state.faces[1].present, "positive X alone is checkpointed");
    }
    {
        TextStore restarted;
        check(restarted.load(edited, state, error) && state.faces[0].present && !state.faces[1].present,
              "single-face checkpoint survives restart");
        MPU6050 sensor;
        fake::state.blocked_face = -1;
        fake::state.face_requests = fake::state.tilt_requests = 0;
        check(sensor.init() && sensor.calibrateFromText(state, TextStore::checkpoint, &restarted, fast()),
              "single-face resume completes");
        check(fake::state.face_requests == 1 && fake::state.tilt_requests == 3,
              "single-face resume repeats only remaining negative X, then fresh holdouts");
        state.gyro_axes[0].present = false;
        const auto before = fake::state.bursts;
        check(sensor.calibrateFromText(state, TextStore::checkpoint, &restarted, fast()) &&
                  fake::state.bursts > before + 1,
              "gyro null forces new measurement even on an already calibrated object");
    }
    std::printf("%d text/file/workflow checks passed\n", checks);
}
