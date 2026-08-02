/*! @file Biped10DoF.cpp
 *  @brief FloatingBaseModel builder for a 10-DoF (5-DoF/leg) bipedal robot,
 *         matching Li & Nguyen (2021) robot topology and Table I params.
 */

#include "Dynamics/Biped10DoF.h"
#include "Dynamics/spatial.h"
#include "Math/orientation_tools.h"

using namespace ori;
using namespace spatial;

template <typename T>
bool Biped10DoF<T>::buildModel(FloatingBaseModel<T>& model) {
  Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);
  model.addBase(_bodyInertia);
  model.addGroundContactBoxPoints(5, bodyDims);

  const int baseID = 5;      // bodies 0-4 = virtual 6-DoF floating chain
  int bodyID = baseID;
  T sideSign = -1;            // -1 = right leg, +1 = left leg

  Mat3<T> I3 = Mat3<T>::Identity();

  for (int legID = 0; legID < 2; legID++) {
    // ---- Hip Yaw (Z) ----
    bodyID++;
    Mat6<T> xtreeHipYaw =
        createSXform(I3, withLegSigns<T>(_hipYawLocation, legID));
    Mat6<T> xtreeHipYawRotor =
        createSXform(I3, withLegSigns<T>(_hipYawRotorLocation, legID));
    if (sideSign < 0) {
      model.addBody(_hipYawInertia.flipAlongAxis(CoordinateAxis::Y),
                     _hipYawRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                     _hipYawGearRatio, baseID, JointType::Revolute,
                     CoordinateAxis::Z, xtreeHipYaw, xtreeHipYawRotor);
    } else {
      model.addBody(_hipYawInertia, _hipYawRotorInertia, _hipYawGearRatio,
                     baseID, JointType::Revolute, CoordinateAxis::Z,
                     xtreeHipYaw, xtreeHipYawRotor);
    }

    // ---- Hip Roll / Abduction (X) ----
    bodyID++;
    Mat6<T> xtreeAbad =
        createSXform(I3, withLegSigns<T>(_abadLocation, legID));
    Mat6<T> xtreeAbadRotor =
        createSXform(I3, withLegSigns<T>(_abadRotorLocation, legID));
    if (sideSign < 0) {
      model.addBody(_abadInertia.flipAlongAxis(CoordinateAxis::Y),
                     _abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                     _abadGearRatio, bodyID - 1, JointType::Revolute,
                     CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
    } else {
      model.addBody(_abadInertia, _abadRotorInertia, _abadGearRatio,
                     bodyID - 1, JointType::Revolute, CoordinateAxis::X,
                     xtreeAbad, xtreeAbadRotor);
    }

    // ---- Hip Pitch (Y) ----
    bodyID++;
    Mat6<T> xtreeHipPitch =
        createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                     withLegSigns<T>(_hipPitchLocation, legID));
    Mat6<T> xtreeHipPitchRotor =
        createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                     withLegSigns<T>(_hipPitchRotorLocation, legID));
    if (sideSign < 0) {
      model.addBody(_hipPitchInertia.flipAlongAxis(CoordinateAxis::Y),
                     _hipPitchRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                     _hipPitchGearRatio, bodyID - 1, JointType::Revolute,
                     CoordinateAxis::Y, xtreeHipPitch, xtreeHipPitchRotor);
    } else {
      model.addBody(_hipPitchInertia, _hipPitchRotorInertia,
                     _hipPitchGearRatio, bodyID - 1, JointType::Revolute,
                     CoordinateAxis::Y, xtreeHipPitch, xtreeHipPitchRotor);
    }

    // ---- Knee (Y) ----
    bodyID++;
    Mat6<T> xtreeKnee = createSXform(I3, _kneeLocation);
    Mat6<T> xtreeKneeRotor = createSXform(I3, _kneeRotorLocation);
    if (sideSign < 0) {
      model.addBody(_kneeInertia.flipAlongAxis(CoordinateAxis::Y),
                     _kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                     _kneeGearRatio, bodyID - 1, JointType::Revolute,
                     CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);
    } else {
      model.addBody(_kneeInertia, _kneeRotorInertia, _kneeGearRatio,
                     bodyID - 1, JointType::Revolute, CoordinateAxis::Y,
                     xtreeKnee, xtreeKneeRotor);
    }

    // ---- Ankle Pitch (Y) — terminal link, carries the foot ----
    bodyID++;
    Mat6<T> xtreeAnkle = createSXform(I3, _ankleLocation);
    Mat6<T> xtreeAnkleRotor = createSXform(I3, _ankleRotorLocation);
    if (sideSign < 0) {
      model.addBody(_ankleInertia.flipAlongAxis(CoordinateAxis::Y),
                     _ankleRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                     _ankleGearRatio, bodyID - 1, JointType::Revolute,
                     CoordinateAxis::Y, xtreeAnkle, xtreeAnkleRotor);
    } else {
      model.addBody(_ankleInertia, _ankleRotorInertia, _ankleGearRatio,
                     bodyID - 1, JointType::Revolute, CoordinateAxis::Y,
                     xtreeAnkle, xtreeAnkleRotor);
    }

    // ---- Single foot contact point (Model 3: 1 pt/foot, 3D force + 2D moment) ----
    // Vertical offset places the point at the sole; toe/heel length (ltoe,
    // lheel from Table I) is NOT modeled as separate contact points here —
    // that distinction belongs to Model 1, not the chosen Model 3.
    model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_footHeight), true);

    sideSign *= -1;
  }

  Vec3<T> g(0, 0, -9.81);
  model.setGravity(g);

  return true;
}

/*!
 * Flip signs of hip-offset vectors between right (legID=0) and left (legID=1) leg.
 * Biped only needs a single left/right mirror — no front/back case, unlike
 * the quadruped's 4-way withLegSigns.
 */
template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID) {
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  switch (legID) {
    case 0: return Vec3<T>(v[0], -v[1], v[2]);  // right leg
    case 1: return Vec3<T>(v[0],  v[1], v[2]);  // left leg
    default: throw std::runtime_error("Invalid leg id!");
  }
}

template <typename T>
std::vector<ActuatorModel<T>> Biped10DoF<T>::buildActuatorModels() {
  std::vector<ActuatorModel<T>> models;
  models.emplace_back(_hipYawGearRatio,   _motorKT, _motorR, _batteryV, _jointDamping, _jointDryFriction, _motorTauMax);
  models.emplace_back(_abadGearRatio,     _motorKT, _motorR, _batteryV, _jointDamping, _jointDryFriction, _motorTauMax);
  models.emplace_back(_hipPitchGearRatio, _motorKT, _motorR, _batteryV, _jointDamping, _jointDryFriction, _motorTauMax);
  models.emplace_back(_kneeGearRatio,     _motorKT, _motorR, _batteryV, _jointDamping, _jointDryFriction, _motorTauMax);
  models.emplace_back(_ankleGearRatio,    _motorKT, _motorR, _batteryV, _jointDamping, _jointDryFriction, _motorTauMax);
  return models;
}

template class Biped10DoF<double>;
template class Biped10DoF<float>;