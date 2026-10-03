/*! @file Biped10DoF.h
 *  @brief Data structure containing parameters for a 10-DoF bipedal robot
 *
 *  This file contains the Biped10DoF class. This stores all the parameters for
 * a 10-DoF (5-DoF/leg) bipedal robot, following the Li & Nguyen (CDC 2021)
 * force-and-moment MPC robot topology. There is a buildModel() method
 * which can be used to create a floating-base dynamics model of the biped.
 */

#ifndef LIBBIOMIMETICS_BIPED10DOF_H
#define LIBBIOMIMETICS_BIPED10DOF_H

#include <vector>
#include "Dynamics/ActuatorModel.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/SpatialInertia.h"

#include <eigen3/Eigen/StdVector>



/*!
 * Basic parameters for a 10-DoF bipedal robot
 */
namespace biped {
constexpr size_t num_act_joint = 10;
constexpr size_t num_q = 17;       // 7 (floating base: quat+pos) + 10 actuated
constexpr size_t dim_config = 16;  // 6 (floating base) + 10 actuated
constexpr size_t num_leg = 2;
constexpr size_t num_leg_joint = 5;
}  // namespace biped

/*!
 * Link indices for the biped.
 * Body ID assignment follows the serial chain per leg:
 * HipYaw -> HipRoll(Abad) -> HipPitch -> Knee -> Ankle
 * starting at baseID+1 for the right leg, continuing for the left leg.
 */
namespace linkID {
constexpr size_t R_foot = 9;    // Right Foot (Ankle link)
constexpr size_t L_foot = 14;   // Left Foot (Ankle link)

constexpr size_t R_hipYaw = 5;   // Right Hip Yaw
constexpr size_t L_hipYaw = 10;  // Left Hip Yaw

constexpr size_t R_abd = 6;      // Right Hip Roll / Abduction
constexpr size_t L_abd = 11;     // Left Hip Roll / Abduction
}  // namespace linkID

using std::vector;

/*!
 * Representation of a 10-DoF bipedal robot's physical properties.
 *
 * When viewed from the top, the biped's legs are:
 *
 * FRONT
 * 2 1   RIGHT
 * BACK
 *
 * (leg 0 = right, leg 1 = left; no front/back pair, unlike the quadruped)
 */
template <typename T>
class Biped10DoF {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotType _robotType;
  T _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;
  T _hipYawGearRatio, _abadGearRatio, _hipPitchGearRatio, _kneeGearRatio, _ankleGearRatio;
  T _hipYawLinkLength, _abadLinkLength, _hipPitchLinkLength, _kneeLinkLength,
      _ankleLinkLength, _kneeLinkY_offset, _maxLegLength;
  T _footToeLength, _footHeelLength, _footHeight;  // ltoe, lheel, foot sole offset (Table I)
  T _motorKT, _motorR, _batteryV;
  T _motorTauMax;
  T _jointDamping, _jointDryFriction;
  SpatialInertia<T> _hipYawInertia, _abadInertia, _hipPitchInertia, _kneeInertia,
      _ankleInertia, _hipYawRotorInertia, _abadRotorInertia, _hipPitchRotorInertia,
      _kneeRotorInertia, _ankleRotorInertia, _bodyInertia;
  Vec3<T> _hipYawLocation, _hipYawRotorLocation, _abadLocation, _abadRotorLocation,
      _hipPitchLocation, _hipPitchRotorLocation, _kneeLocation, _kneeRotorLocation,
      _ankleLocation, _ankleRotorLocation;
  FloatingBaseModel<T> buildModel();
  bool buildModel(FloatingBaseModel<T>& model);
  std::vector<ActuatorModel<T>> buildActuatorModels();

  /*!
   * Get if the i-th leg is on the left (+) or right (-) of the robot.
   * @param leg : the leg index (0 = right, 1 = left)
   * @return The side sign (-1 for right leg, +1 for left leg)
   */
  static T getSideSign(int leg) {
    const T sideSigns[2] = {-1, 1};
    assert(leg >= 0 && leg < 2);
    return sideSigns[leg];
  }

  /*!
   * Get location of the hip (hip yaw joint origin) for the given leg in robot frame
   * @param leg : the leg index (0 = right, 1 = left)
   */
  Vec3<T> getHipLocation(int leg) {
    assert(leg >= 0 && leg < 2);
    Vec3<T> pHip(_hipYawLocation(0),
                 (leg == 1) ? _hipYawLocation(1) : -_hipYawLocation(1),
                 _hipYawLocation(2));
    return pHip;
  }
};

template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID);

#endif  // LIBBIOMIMETICS_BIPED10DOF_H