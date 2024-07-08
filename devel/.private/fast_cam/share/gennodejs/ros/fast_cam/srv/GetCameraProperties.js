// Auto-generated. Do not edit!

// (in-package fast_cam.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetCameraPropertiesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCameraPropertiesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCameraPropertiesRequest
    let len;
    let data = new GetCameraPropertiesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'fast_cam/GetCameraPropertiesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # string set_gain
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCameraPropertiesRequest(null);
    return resolved;
    }
};

class GetCameraPropertiesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.model = null;
      this.serial_number = null;
      this.ip_address = null;
      this.resolution = null;
      this.target_fps = null;
      this.exposure_time = null;
      this.gain = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('model')) {
        this.model = initObj.model
      }
      else {
        this.model = '';
      }
      if (initObj.hasOwnProperty('serial_number')) {
        this.serial_number = initObj.serial_number
      }
      else {
        this.serial_number = '';
      }
      if (initObj.hasOwnProperty('ip_address')) {
        this.ip_address = initObj.ip_address
      }
      else {
        this.ip_address = '';
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = '';
      }
      if (initObj.hasOwnProperty('target_fps')) {
        this.target_fps = initObj.target_fps
      }
      else {
        this.target_fps = 0;
      }
      if (initObj.hasOwnProperty('exposure_time')) {
        this.exposure_time = initObj.exposure_time
      }
      else {
        this.exposure_time = 0.0;
      }
      if (initObj.hasOwnProperty('gain')) {
        this.gain = initObj.gain
      }
      else {
        this.gain = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCameraPropertiesResponse
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [model]
    bufferOffset = _serializer.string(obj.model, buffer, bufferOffset);
    // Serialize message field [serial_number]
    bufferOffset = _serializer.string(obj.serial_number, buffer, bufferOffset);
    // Serialize message field [ip_address]
    bufferOffset = _serializer.string(obj.ip_address, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = _serializer.string(obj.resolution, buffer, bufferOffset);
    // Serialize message field [target_fps]
    bufferOffset = _serializer.uint8(obj.target_fps, buffer, bufferOffset);
    // Serialize message field [exposure_time]
    bufferOffset = _serializer.float32(obj.exposure_time, buffer, bufferOffset);
    // Serialize message field [gain]
    bufferOffset = _serializer.string(obj.gain, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCameraPropertiesResponse
    let len;
    let data = new GetCameraPropertiesResponse(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [model]
    data.model = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [serial_number]
    data.serial_number = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ip_address]
    data.ip_address = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_fps]
    data.target_fps = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [exposure_time]
    data.exposure_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gain]
    data.gain = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += _getByteLength(object.model);
    length += _getByteLength(object.serial_number);
    length += _getByteLength(object.ip_address);
    length += _getByteLength(object.resolution);
    length += _getByteLength(object.gain);
    return length + 29;
  }

  static datatype() {
    // Returns string type for a service object
    return 'fast_cam/GetCameraPropertiesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca77d29133bf5a9bfbfacc01b41e087c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    string model
    string serial_number
    string ip_address
    string resolution
    uint8 target_fps
    float32 exposure_time
    string gain
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCameraPropertiesResponse(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.model !== undefined) {
      resolved.model = msg.model;
    }
    else {
      resolved.model = ''
    }

    if (msg.serial_number !== undefined) {
      resolved.serial_number = msg.serial_number;
    }
    else {
      resolved.serial_number = ''
    }

    if (msg.ip_address !== undefined) {
      resolved.ip_address = msg.ip_address;
    }
    else {
      resolved.ip_address = ''
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = ''
    }

    if (msg.target_fps !== undefined) {
      resolved.target_fps = msg.target_fps;
    }
    else {
      resolved.target_fps = 0
    }

    if (msg.exposure_time !== undefined) {
      resolved.exposure_time = msg.exposure_time;
    }
    else {
      resolved.exposure_time = 0.0
    }

    if (msg.gain !== undefined) {
      resolved.gain = msg.gain;
    }
    else {
      resolved.gain = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: GetCameraPropertiesRequest,
  Response: GetCameraPropertiesResponse,
  md5sum() { return 'ca77d29133bf5a9bfbfacc01b41e087c'; },
  datatype() { return 'fast_cam/GetCameraProperties'; }
};
