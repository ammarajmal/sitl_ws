
"use strict";

let BoundingBox2D = require('./BoundingBox2D.js');
let ObjectHypothesisWithPose = require('./ObjectHypothesisWithPose.js');
let Classification3D = require('./Classification3D.js');
let BoundingBox2DArray = require('./BoundingBox2DArray.js');
let Detection3D = require('./Detection3D.js');
let VisionInfo = require('./VisionInfo.js');
let ObjectHypothesis = require('./ObjectHypothesis.js');
let Detection2D = require('./Detection2D.js');
let Detection2DArray = require('./Detection2DArray.js');
let BoundingBox3DArray = require('./BoundingBox3DArray.js');
let Classification2D = require('./Classification2D.js');
let Detection3DArray = require('./Detection3DArray.js');
let BoundingBox3D = require('./BoundingBox3D.js');

module.exports = {
  BoundingBox2D: BoundingBox2D,
  ObjectHypothesisWithPose: ObjectHypothesisWithPose,
  Classification3D: Classification3D,
  BoundingBox2DArray: BoundingBox2DArray,
  Detection3D: Detection3D,
  VisionInfo: VisionInfo,
  ObjectHypothesis: ObjectHypothesis,
  Detection2D: Detection2D,
  Detection2DArray: Detection2DArray,
  BoundingBox3DArray: BoundingBox3DArray,
  Classification2D: Classification2D,
  Detection3DArray: Detection3DArray,
  BoundingBox3D: BoundingBox3D,
};
