// Auto-generated. Do not edit!

// (in-package custom_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Speed_tick {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.right_speed = null;
      this.left_speed = null;
      this.right_tick = null;
      this.left_tick = null;
    }
    else {
      if (initObj.hasOwnProperty('right_speed')) {
        this.right_speed = initObj.right_speed
      }
      else {
        this.right_speed = 0.0;
      }
      if (initObj.hasOwnProperty('left_speed')) {
        this.left_speed = initObj.left_speed
      }
      else {
        this.left_speed = 0.0;
      }
      if (initObj.hasOwnProperty('right_tick')) {
        this.right_tick = initObj.right_tick
      }
      else {
        this.right_tick = 0;
      }
      if (initObj.hasOwnProperty('left_tick')) {
        this.left_tick = initObj.left_tick
      }
      else {
        this.left_tick = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Speed_tick
    // Serialize message field [right_speed]
    bufferOffset = _serializer.float64(obj.right_speed, buffer, bufferOffset);
    // Serialize message field [left_speed]
    bufferOffset = _serializer.float64(obj.left_speed, buffer, bufferOffset);
    // Serialize message field [right_tick]
    bufferOffset = _serializer.int32(obj.right_tick, buffer, bufferOffset);
    // Serialize message field [left_tick]
    bufferOffset = _serializer.int32(obj.left_tick, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Speed_tick
    let len;
    let data = new Speed_tick(null);
    // Deserialize message field [right_speed]
    data.right_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_speed]
    data.left_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_tick]
    data.right_tick = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [left_tick]
    data.left_tick = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msgs/Speed_tick';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '49db339e4cdd08bb4996af3a92cbcc3d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This represents the right and left speed plus the right and left tick
    
    float64 right_speed
    float64 left_speed
    int32 right_tick
    int32 left_tick
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Speed_tick(null);
    if (msg.right_speed !== undefined) {
      resolved.right_speed = msg.right_speed;
    }
    else {
      resolved.right_speed = 0.0
    }

    if (msg.left_speed !== undefined) {
      resolved.left_speed = msg.left_speed;
    }
    else {
      resolved.left_speed = 0.0
    }

    if (msg.right_tick !== undefined) {
      resolved.right_tick = msg.right_tick;
    }
    else {
      resolved.right_tick = 0
    }

    if (msg.left_tick !== undefined) {
      resolved.left_tick = msg.left_tick;
    }
    else {
      resolved.left_tick = 0
    }

    return resolved;
    }
};

module.exports = Speed_tick;
