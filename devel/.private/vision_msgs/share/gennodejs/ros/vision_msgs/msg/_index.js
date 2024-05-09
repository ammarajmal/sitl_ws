
"use strict";

let Classification2D = require('./Classification2D.js');
let BoundingBox3D = require('./BoundingBox3D.js');
let BoundingBox2D = require('./BoundingBox2D.js');
let BoundingBox3DArray = require('./BoundingBox3DArray.js');
let Detection3DArray = require('./Detection3DArray.js');
let ObjectHypothesis = require('./ObjectHypothesis.js');
let Detection3D = require('./Detection3D.js');
let Detection2DArray = require('./Detection2DArray.js');
let ObjectHypothesisWithPose = require('./ObjectHypothesisWithPose.js');
let BoundingBox2DArray = require('./BoundingBox2DArray.js');
let VisionInfo = require('./VisionInfo.js');
let Classification3D = require('./Classification3D.js');
let Detection2D = require('./Detection2D.js');

module.exports = {
  Classification2D: Classification2D,
  BoundingBox3D: BoundingBox3D,
  BoundingBox2D: BoundingBox2D,
  BoundingBox3DArray: BoundingBox3DArray,
  Detection3DArray: Detection3DArray,
  ObjectHypothesis: ObjectHypothesis,
  Detection3D: Detection3D,
  Detection2DArray: Detection2DArray,
  ObjectHypothesisWithPose: ObjectHypothesisWithPose,
  BoundingBox2DArray: BoundingBox2DArray,
  VisionInfo: VisionInfo,
  Classification3D: Classification3D,
  Detection2D: Detection2D,
};
