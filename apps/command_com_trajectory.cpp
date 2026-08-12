// Commands the robot's COM velocity (x_dot, y_dot) directly for a fixed sim
// duration -- no target position/distance, no closed-loop position tracking.
//
// There is no IMU/world-position feedback wired into this control stack, so
// treating (dx, dy) as a "goal to reach" was never actually closed-loop; it
// only worked by dead-reckoning a velocity profile and hoping the achieved
// distance matched. This tool drops that pretense: you give it a target
// velocity and an acceleration to ramp up to it, it holds that velocity for
// the rest of the run, and the sim just ends after --duration seconds.
//
// Ramps are built as densely-sampled (time, value) points and fed to the sim
// through the CONVEXMPC_HEADLESS_*_PROFILE environment-variable hooks in
// SimulationRunner (sim/src/SimulationRunner.cpp:
// applyHeadlessUserCommandSchedule) -- densely sampled because that sampler
// is step-hold, not interpolated, between points. The MPC tracks the
// resulting x_dot/y_dot reference in closed loop the same way it tracks a
// keyboard command.
//
// Usage:
//   ./build/apps/command_com_trajectory --vx 0.3 --duration 15
//   ./build/apps/command_com_trajectory --vx 0.3 --vy 0.15 --ax 0.2 --ay 0.15 --duration 15
//   ./build/apps/command_com_trajectory --vx 0.3 --viewer n --plot
//   ./build/apps/command_com_trajectory --vx 0.3 --dry-run

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "My_Controller.h"
#include "main_helper.h"

namespace {

struct Args {
    double vx{0.3};
    double vy{0.0};
    double ax{0.3};
    double ay{0.3};
    double duration{15.0};
    double settleTime{2.2};
    double sampleDt{0.05};
    double telemetryDt{0.02};
    std::string viewer{"y"};
    std::string robot{"t"};
    std::string csvPath{"logs/debug/state_telemetry/state_telemetry.csv"};
    bool plot{false};
    bool dryRun{false};
};

struct Profile {
    std::vector<std::pair<double, double>> points;
};

void printUsage() {
    std::cout <<
        "Usage: command_com_trajectory [options]\n"
        "  --vx VALUE            target (cruise) x_dot velocity [m/s] (default 0.3)\n"
        "  --vy VALUE            target (cruise) y_dot velocity [m/s] (default 0.0)\n"
        "  --ax VALUE            acceleration used to ramp up to vx [m/s^2] (default 0.3)\n"
        "  --ay VALUE            acceleration used to ramp up to vy [m/s^2] (default 0.3)\n"
        "  --duration VALUE      total sim run time [s] (default 15.0)\n"
        "  --settle-time VALUE   wait before commanding velocity, for leg PD init + the\n"
        "                        standing->walking FSM handshake to finish [s] (default 2.2)\n"
        "  --sample-dt VALUE     profile sample interval [s] (default 0.05)\n"
        "  --telemetry-dt VALUE  CONVEXMPC_STATE_TELEMETRY_DT [s] (default 0.02)\n"
        "  --viewer y|n          y = GUI viewer, n = headless (default y)\n"
        "  --robot ID            robot-id for the sim (default t = TRON1 biped)\n"
        "  --csv PATH            telemetry CSV path (default logs/debug/state_telemetry/state_telemetry.csv)\n"
        "  --plot                run plots/plot_com_state.py on the resulting CSV\n"
        "  --dry-run             print the computed profiles/env and exit\n";
}

bool parseArgs(int argc, char** argv, Args& args) {
    auto nextValue = [&](int& i) -> std::string {
        ++i;
        return i < argc ? std::string(argv[i]) : std::string();
    };

    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--help" || flag == "-h") {
            printUsage();
            std::exit(0);
        } else if (flag == "--vx") {
            args.vx = std::stod(nextValue(i));
        } else if (flag == "--vy") {
            args.vy = std::stod(nextValue(i));
        } else if (flag == "--ax") {
            args.ax = std::stod(nextValue(i));
        } else if (flag == "--ay") {
            args.ay = std::stod(nextValue(i));
        } else if (flag == "--duration") {
            args.duration = std::stod(nextValue(i));
        } else if (flag == "--settle-time") {
            args.settleTime = std::stod(nextValue(i));
        } else if (flag == "--sample-dt") {
            args.sampleDt = std::stod(nextValue(i));
        } else if (flag == "--telemetry-dt") {
            args.telemetryDt = std::stod(nextValue(i));
        } else if (flag == "--viewer") {
            args.viewer = nextValue(i);
        } else if (flag == "--robot") {
            args.robot = nextValue(i);
        } else if (flag == "--csv") {
            args.csvPath = nextValue(i);
        } else if (flag == "--plot") {
            args.plot = true;
        } else if (flag == "--dry-run") {
            args.dryRun = true;
        } else {
            std::cerr << "Unknown argument: " << flag << '\n';
            printUsage();
            return false;
        }
    }

    if (args.viewer != "y" && args.viewer != "n") {
        std::cerr << "--viewer must be 'y' or 'n'\n";
        return false;
    }
    if (args.duration <= args.settleTime) {
        std::cerr << "--duration must be greater than --settle-time\n";
        return false;
    }
    return true;
}

// Dense (sampled) ramp from 0 up to targetVelocity at the given acceleration,
// then held at targetVelocity. Densely sampled because SimulationRunner's
// profile sampler is step-hold (not interpolated) between the given points.
// If accel <= 0, commands targetVelocity as an immediate step (the
// controller's own x_dot_tau/y_dot_tau low-pass filter will still smooth it).
Profile rampToVelocityProfile(double targetVelocity, double accel, double tStart,
                               double runDuration, double sampleDt) {
    Profile profile;
    if (std::abs(targetVelocity) < 1e-9) {
        return profile;
    }

    if (accel <= 0.0) {
        profile.points.emplace_back(tStart, targetVelocity);
        return profile;
    }

    const double direction = std::copysign(1.0, targetVelocity);
    const double rampDuration = std::abs(targetVelocity) / accel;
    const double cappedRampDuration = std::min(rampDuration, runDuration);

    for (double t = 0.0; t < cappedRampDuration; t += sampleDt) {
        profile.points.emplace_back(tStart + t, direction * accel * t);
    }
    profile.points.emplace_back(tStart + cappedRampDuration,
                                 direction * accel * cappedRampDuration);
    return profile;
}

std::string formatProfile(const std::vector<std::pair<double, double>>& points) {
    std::ostringstream out;
    out << std::fixed;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i > 0) {
            out << ',';
        }
        out << std::setprecision(3) << points[i].first << ':'
            << std::setprecision(4) << points[i].second;
    }
    return out.str();
}

std::string truncatedForPrint(const std::string& value, std::size_t maxLen = 120) {
    if (value.size() <= maxLen) {
        return value;
    }
    return value.substr(0, maxLen) + "...";
}

// Minimal CSV reader: returns column name -> value for the first and last data rows.
struct CsvEndpoints {
    bool valid{false};
    std::unordered_map<std::string, double> first;
    std::unordered_map<std::string, double> last;
};

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

CsvEndpoints readCsvEndpoints(const std::string& path) {
    CsvEndpoints result;
    std::ifstream file(path);
    if (!file.is_open()) {
        return result;
    }

    std::string headerLine;
    if (!std::getline(file, headerLine)) {
        return result;
    }
    const std::vector<std::string> header = splitCsvLine(headerLine);

    auto rowToMap = [&](const std::string& line) -> std::unordered_map<std::string, double> {
        std::unordered_map<std::string, double> row;
        const std::vector<std::string> fields = splitCsvLine(line);
        for (std::size_t i = 0; i < header.size() && i < fields.size(); ++i) {
            try {
                row[header[i]] = std::stod(fields[i]);
            } catch (const std::exception&) {
                // skip unparseable field
            }
        }
        return row;
    };

    std::string line;
    std::string lastLine;
    bool haveFirst = false;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        if (!haveFirst) {
            result.first = rowToMap(line);
            haveFirst = true;
        }
        lastLine = line;
    }
    if (!haveFirst) {
        return result;
    }
    result.last = rowToMap(lastLine);
    result.valid = true;
    return result;
}

void reportResult(const std::string& csvPath, const Args& args) {
    const CsvEndpoints endpoints = readCsvEndpoints(csvPath);
    if (!endpoints.valid) {
        std::cout << "\n[warning] no telemetry found at " << csvPath << "; nothing to report.\n";
        return;
    }

    auto get = [](const std::unordered_map<std::string, double>& row, const char* key) {
        const auto it = row.find(key);
        return it != row.end() ? it->second : 0.0;
    };

    const double distanceX = get(endpoints.last, "com_x") - get(endpoints.first, "com_x");
    const double distanceY = get(endpoints.last, "com_y") - get(endpoints.first, "com_y");
    const double finalVx = get(endpoints.last, "com_vx");
    const double finalVy = get(endpoints.last, "com_vy");

    std::cout << "\n=== Run summary (t=0 to t=" << std::fixed << std::setprecision(1)
              << args.duration << "s) ===\n";
    std::cout << std::left << std::setw(6) << "axis" << std::right << std::setw(16)
              << "commanded_v" << std::setw(18) << "final_actual_v" << std::setw(20)
              << "net_distance" << '\n';
    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::left << std::setw(6) << "x" << std::right << std::setw(16) << args.vx
              << std::setw(18) << finalVx << std::setw(20) << distanceX << '\n';
    std::cout << std::left << std::setw(6) << "y" << std::right << std::setw(16) << args.vy
              << std::setw(18) << finalVy << std::setw(20) << distanceY << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, args)) {
        return EXIT_FAILURE;
    }

    const double runDuration = args.duration - args.settleTime;
    const Profile xProfile =
        rampToVelocityProfile(args.vx, args.ax, args.settleTime, runDuration, args.sampleDt);
    const Profile yProfile =
        rampToVelocityProfile(args.vy, args.ay, args.settleTime, runDuration, args.sampleDt);

    std::cout << "Total sim duration: " << std::fixed << std::setprecision(2) << args.duration
              << " s (settle=" << args.settleTime << "s, then hold vx=" << args.vx
              << " m/s, vy=" << args.vy << " m/s)\n";

    setenv("CONVEXMPC_HEADLESS_AUTO_WALK", "1", 1);
    // This tool commands x_dot/y_dot programmatically -- never start the keyboard
    // reader thread, so there's no chance of stray keypresses (or terminal focus)
    // affecting the run.
    setenv("CONVEXMPC_DISABLE_KEYBOARD", "1", 1);
    setenv("CONVEXMPC_STATE_TELEMETRY", args.csvPath.c_str(), 1);
    setenv("CONVEXMPC_STATE_TELEMETRY_DT", std::to_string(args.telemetryDt).c_str(), 1);

    if (!xProfile.points.empty()) {
        const std::string value = formatProfile(xProfile.points);
        setenv("CONVEXMPC_HEADLESS_X_DOT_PROFILE", value.c_str(), 1);
        std::cout << "  CONVEXMPC_HEADLESS_X_DOT_PROFILE=" << truncatedForPrint(value) << '\n';
    }
    if (!yProfile.points.empty()) {
        const std::string value = formatProfile(yProfile.points);
        setenv("CONVEXMPC_HEADLESS_Y_DOT_PROFILE", value.c_str(), 1);
        std::cout << "  CONVEXMPC_HEADLESS_Y_DOT_PROFILE=" << truncatedForPrint(value) << '\n';
    }

    // Auto-terminates the sim (and closes the viewer window, if any) after
    // --duration seconds of sim time -- works in both GUI and headless mode.
    const std::string stopTime = std::to_string(args.duration);
    setenv("CONVEXMPC_HEADLESS_STOP_TIME", stopTime.c_str(), 1);
    std::cout << "  CONVEXMPC_HEADLESS_STOP_TIME=" << stopTime << '\n';

    if (args.dryRun) {
        std::cout << "\n--dry-run: not launching the sim.\n";
        return EXIT_SUCCESS;
    }

    std::cout << "\nRunning sim: robot=" << args.robot << " viewer=" << args.viewer << '\n';
    if (args.viewer == "y") {
        std::cout << "(viewer will close itself automatically after --duration seconds)\n";
    }

    std::vector<std::string> simArgsStorage = {"command_com_trajectory", args.robot, args.viewer};
    std::vector<char*> simArgv;
    simArgv.reserve(simArgsStorage.size());
    for (auto& value : simArgsStorage) {
        simArgv.push_back(value.data());
    }

    auto controller = std::make_unique<MyController>();
    main_helper(static_cast<int>(simArgv.size()), simArgv.data(), controller.get());

    reportResult(args.csvPath, args);

    if (args.plot) {
        const std::string command =
            "python3 plots/plot_com_state.py --csv \"" + args.csvPath + "\"";
        if (std::system(command.c_str()) != 0) {
            std::cerr << "[warning] plotting command failed: " << command << '\n';
        }
    }

    return EXIT_SUCCESS;
}
