
"use strict";

let BoundingBox3DArray = require('./BoundingBox3DArray.js');
let Classification2D = require('./Classification2D.js');
let BoundingBox3D = require('./BoundingBox3D.js');
let Detection2DArray = require('./Detection2DArray.js');
let Classification3D = require('./Classification3D.js');
let Detection2D = require('./Detection2D.js');
let BoundingBox2DArray = require('./BoundingBox2DArray.js');
let BoundingBox2D = require('./BoundingBox2D.js');
let Detection3D = require('./Detection3D.js');
let Detection3DArray = require('./Detection3DArray.js');
let VisionInfo = require('./VisionInfo.js');
let ObjectHypothesisWithPose = require('./ObjectHypothesisWithPose.js');
let ObjectHypothesis = require('./ObjectHypothesis.js');

module.exports = {
  BoundingBox3DArray: BoundingBox3DArray,
  Classification2D: Classification2D,
  BoundingBox3D: BoundingBox3D,
  Detection2DArray: Detection2DArray,
  Classification3D: Classification3D,
  Detection2D: Detection2D,
  BoundingBox2DArray: BoundingBox2DArray,
  BoundingBox2D: BoundingBox2D,
  Detection3D: Detection3D,
  Detection3DArray: Detection3DArray,
  VisionInfo: VisionInfo,
  ObjectHypothesisWithPose: ObjectHypothesisWithPose,
  ObjectHypothesis: ObjectHypothesis,
};
