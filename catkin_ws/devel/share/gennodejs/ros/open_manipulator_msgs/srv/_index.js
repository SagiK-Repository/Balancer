
"use strict";

let GetJointPosition = require('./GetJointPosition.js')
let SetDrawingTrajectory = require('./SetDrawingTrajectory.js')
let GetKinematicsPose = require('./GetKinematicsPose.js')
let SetActuatorState = require('./SetActuatorState.js')
let SetKinematicsPose = require('./SetKinematicsPose.js')
let SetJointPosition = require('./SetJointPosition.js')

module.exports = {
  GetJointPosition: GetJointPosition,
  SetDrawingTrajectory: SetDrawingTrajectory,
  GetKinematicsPose: GetKinematicsPose,
  SetActuatorState: SetActuatorState,
  SetKinematicsPose: SetKinematicsPose,
  SetJointPosition: SetJointPosition,
};
