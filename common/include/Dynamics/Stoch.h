/*! @file Stoch.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_STOCH_H
#define PROJECT_STOCH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Stoch
 */
template <typename T>
Quadruped<T> buildStoch() {
  Quadruped<T> stoch;
  stoch._robotType = RobotType::STOCH;

  stoch._bodyMass = 3.3;
  stoch._bodyLength = 0.19 * 2;
  stoch._bodyWidth = 0.049 * 2;
  stoch._bodyHeight = 0.05 * 2;
  stoch._abadGearRatio = 6;
  stoch._hipGearRatio = 6;
  stoch._kneeGearRatio = 9.33;
  stoch._abadLinkLength = 0.062;
  stoch._hipLinkLength = 0.209;
  stoch._kneeLinkY_offset = 0.004;
  stoch._kneeLinkLength = 0.195;
  stoch._maxLegLength = 0.409;


  stoch._motorTauMax = 3.f;
  stoch._batteryV = 24;
  stoch._motorKT = .05;  // this is flux linkage * pole pairs
  stoch._motorR = 0.173;
  stoch._jointDamping = .01;
  stoch._jointDryFriction = .2;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0, 0.016, -0.02);
  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.064, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(stoch._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  stoch._abadInertia = abadInertia;
  stoch._hipInertia = hipInertia;
  stoch._kneeInertia = kneeInertia;
  stoch._abadRotorInertia = rotorInertiaX;
  stoch._hipRotorInertia = rotorInertiaY;
  stoch._kneeRotorInertia = rotorInertiaY;
  stoch._bodyInertia = bodyInertia;

  // locations
  stoch._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  stoch._abadLocation =
      Vec3<T>(stoch._bodyLength, stoch._bodyWidth, 0) * 0.5;
  stoch._hipLocation = Vec3<T>(0, stoch._abadLinkLength, 0);
  stoch._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  stoch._kneeLocation = Vec3<T>(0, 0, -stoch._hipLinkLength);
  stoch._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return stoch;
}

#endif  // PROJECT_STOCH_H
