#include "models/RobotMujocoSpec.h"

const RobotMujocoSpec& getTronBipedMujocoSpec() {
    static const RobotMujocoSpec spec{
        RobotType::TRON_BIPED,
        "base_Link",
        {
            {Side::Left,
             "foot_L_Link",
             "foot_L",
             {
                 {"abad_L_Joint", "abad_L_motor"},
                 {"hip_L_Joint", "hip_L_motor"},
                 {"knee_L_Joint", "knee_L_motor"},
                 {"ankle_L_Joint", "ankle_L_motor"},
             }},
            {Side::Right,
             "foot_R_Link",
             "foot_R",
             {
                 {"abad_R_Joint", "abad_R_motor"},
                 {"hip_R_Joint", "hip_R_motor"},
                 {"knee_R_Joint", "knee_R_motor"},
                 {"ankle_R_Joint", "ankle_R_motor"},
             }},
        },
        {},
        {}};

    return spec;
}
