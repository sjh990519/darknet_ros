
"use strict";

let BoundingBox = require('./BoundingBox.js');
let BoundingBoxes = require('./BoundingBoxes.js');
let ObjectCount = require('./ObjectCount.js');
let CheckForObjectsActionFeedback = require('./CheckForObjectsActionFeedback.js');
let CheckForObjectsResult = require('./CheckForObjectsResult.js');
let CheckForObjectsFeedback = require('./CheckForObjectsFeedback.js');
let CheckForObjectsActionResult = require('./CheckForObjectsActionResult.js');
let CheckForObjectsGoal = require('./CheckForObjectsGoal.js');
let CheckForObjectsActionGoal = require('./CheckForObjectsActionGoal.js');
let CheckForObjectsAction = require('./CheckForObjectsAction.js');

module.exports = {
  BoundingBox: BoundingBox,
  BoundingBoxes: BoundingBoxes,
  ObjectCount: ObjectCount,
  CheckForObjectsActionFeedback: CheckForObjectsActionFeedback,
  CheckForObjectsResult: CheckForObjectsResult,
  CheckForObjectsFeedback: CheckForObjectsFeedback,
  CheckForObjectsActionResult: CheckForObjectsActionResult,
  CheckForObjectsGoal: CheckForObjectsGoal,
  CheckForObjectsActionGoal: CheckForObjectsActionGoal,
  CheckForObjectsAction: CheckForObjectsAction,
};
