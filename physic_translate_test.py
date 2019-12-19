# -*- coding: utf-8 -*-
import math


def PxVehicleComputeTireForceDefault():

  # 填写参数位置
  tireData = {
    "mLatStiffX": 1,
    "mLatStiffY": 1,
    "mLongitudinalStiffnessPerUnitGravity": 1,
    "mCamberStiffnessPerUnitGravity": 1,
  }
  tireFriction = 1
  longSlipUnClamped = 1
  latSlipUnClamped = 1
  camberUnclamped = 1
  wheelRadius = 1
  restTireLoad = 1
  normalisedTireLoad = 1
  tireLoad = 1
  gravity = 1



  assert tireFriction > 0
  assert  tireLoad > 0
  wheelTorque = 0.0
  tireLongForceMag = 0.0
  tireLatForceMag = 0.0
  tireAlignMoment = 0.0

  gMinimumSlipThreshold = 0.0  # 最小滑动阈值，这个值自己设置

  latSlip = latSlipUnClamped if abs(longSlipUnClamped) >= gMinimumSlipThreshold else 0.0
  longSlip = longSlipUnClamped if abs(longSlipUnClamped) >= gMinimumSlipThreshold else 0.0
  camber = camberUnclamped if abs(camberUnclamped) >= gMinimumSlipThreshold else 0.0

  if not latSlip and not longSlip and not camber:
    return

  latStiff = restTireLoad * tireData["mLatStiffY"] * smoothingFunction1(normalisedTireLoad * 3.0 /tireData["mLatStiffX"])
  longStiff = tireData["mLongitudinalStiffnessPerUnitGravity"] * gravity
  recipLongStiff = tireData["mCamberStiffnessPerUnitGravity"] * gravity
  camberStiff = tireData["mCamberStiffnessPerUnitGravity"] * gravity

  TEff = math.tan(latSlip - camberStiff/latStiff)
  K = math.sqrt(latStiff*TEff*latStiff*TEff + longStiff*longSlip*longStiff*longSlip) / (tireFriction*tireLoad)
  FBar = smoothingFunction1(K)
  MBar = smoothingFunction2(K)

  nu = 1
  if K < 2.0 * math.pi:
    latOverlLong = latStiff * recipLongStiff
    nu = 0.5 * (1.0 + latOverlLong - (1.0 - latOverlLong) * math.cos(K * 0.5))

  FZero = tireFriction * tireLoad / (math.sqrt(longSlip * longSlip + nu * TEff * nu * TEff))
  fz = longSlip * FBar * FZero
  fx = -nu*TEff*FBar*FZero
  pneumaticTrail = 1.0
  fMy = nu * pneumaticTrail * TEff * MBar * FZero

  wheelTorque = -fz * wheelRadius
  tireLongForceMag = fz
  tireLatForceMag = fx
  tireAlignMoment = fMy
  print ("wheelTorque =", wheelTorque, " tireLongForceMag = ", tireLongForceMag, " tireLatForceMag = ", tireLatForceMag,
         " tireAlignMoment = ", tireAlignMoment)


ONE_TWENTYSEVENTH = 0.037037
ONE_THIRD = 0.33333

def smoothingFunction1(k):
  assert k >= 0.0
  return min(1.0, k-ONE_THIRD*k*k + ONE_TWENTYSEVENTH *k*k*k)

def smoothingFunction2(k):
  assert k >= 0.0
  return  k - k * k + ONE_THIRD* k * k * k - ONE_TWENTYSEVENTH * k * k * k * k

PxVehicleComputeTireForceDefault( ) # 在这里填写参数