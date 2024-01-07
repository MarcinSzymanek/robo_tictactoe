
"use strict";

let SetComplianceSlope = require('./SetComplianceSlope.js')
let StartController = require('./StartController.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let StopController = require('./StopController.js')
let TorqueEnable = require('./TorqueEnable.js')
let SetSpeed = require('./SetSpeed.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let RestartController = require('./RestartController.js')

module.exports = {
  SetComplianceSlope: SetComplianceSlope,
  StartController: StartController,
  SetComplianceMargin: SetComplianceMargin,
  SetCompliancePunch: SetCompliancePunch,
  StopController: StopController,
  TorqueEnable: TorqueEnable,
  SetSpeed: SetSpeed,
  SetTorqueLimit: SetTorqueLimit,
  RestartController: RestartController,
};
