// Auto-generated. Do not edit!

// (in-package fast_cam.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CameraSpecs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.model = null;
      this.serial_number = null;
      this.ip_address = null;
      this.resolution = null;
      this.frame_rate = null;
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
      if (initObj.hasOwnProperty('frame_rate')) {
        this.frame_rate = initObj.frame_rate
      }
      else {
        this.frame_rate = 0.0;
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
    // Serializes a message object of type CameraSpecs
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
    // Serialize message field [frame_rate]
    bufferOffset = _serializer.float64(obj.frame_rate, buffer, bufferOffset);
    // Serialize message field [exposure_time]
    bufferOffset = _serializer.float64(obj.exposure_time, buffer, bufferOffset);
    // Serialize message field [gain]
    bufferOffset = _serializer.string(obj.gain, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CameraSpecs
    let len;
    let data = new CameraSpecs(null);
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
    // Deserialize message field [frame_rate]
    data.frame_rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [exposure_time]
    data.exposure_time = _deserializer.float64(buffer, bufferOffset);
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
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fast_cam/CameraSpecs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5d47430f74f6ea64f601ee2640f4e549';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # CameraSpecs.msg
    string name
    string model
    string serial_number
    string ip_address
    string resolution
    float64 frame_rate
    float64 exposure_time
    string gain
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CameraSpecs(null);
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

    if (msg.frame_rate !== undefined) {
      resolved.frame_rate = msg.frame_rate;
    }
    else {
      resolved.frame_rate = 0.0
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

module.exports = CameraSpecs;
