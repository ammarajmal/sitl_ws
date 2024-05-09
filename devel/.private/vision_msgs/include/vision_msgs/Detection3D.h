// Generated by gencpp from file vision_msgs/Detection3D.msg
// DO NOT EDIT!


#ifndef VISION_MSGS_MESSAGE_DETECTION3D_H
#define VISION_MSGS_MESSAGE_DETECTION3D_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <vision_msgs/BoundingBox3D.h>
#include <sensor_msgs/PointCloud2.h>

namespace vision_msgs
{
template <class ContainerAllocator>
struct Detection3D_
{
  typedef Detection3D_<ContainerAllocator> Type;

  Detection3D_()
    : header()
    , results()
    , bbox()
    , source_cloud()  {
    }
  Detection3D_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , results(_alloc)
    , bbox(_alloc)
    , source_cloud(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::vision_msgs::ObjectHypothesisWithPose_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::vision_msgs::ObjectHypothesisWithPose_<ContainerAllocator> >> _results_type;
  _results_type results;

   typedef  ::vision_msgs::BoundingBox3D_<ContainerAllocator>  _bbox_type;
  _bbox_type bbox;

   typedef  ::sensor_msgs::PointCloud2_<ContainerAllocator>  _source_cloud_type;
  _source_cloud_type source_cloud;





  typedef boost::shared_ptr< ::vision_msgs::Detection3D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vision_msgs::Detection3D_<ContainerAllocator> const> ConstPtr;

}; // struct Detection3D_

typedef ::vision_msgs::Detection3D_<std::allocator<void> > Detection3D;

typedef boost::shared_ptr< ::vision_msgs::Detection3D > Detection3DPtr;
typedef boost::shared_ptr< ::vision_msgs::Detection3D const> Detection3DConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vision_msgs::Detection3D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vision_msgs::Detection3D_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vision_msgs::Detection3D_<ContainerAllocator1> & lhs, const ::vision_msgs::Detection3D_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.results == rhs.results &&
    lhs.bbox == rhs.bbox &&
    lhs.source_cloud == rhs.source_cloud;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vision_msgs::Detection3D_<ContainerAllocator1> & lhs, const ::vision_msgs::Detection3D_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vision_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vision_msgs::Detection3D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision_msgs::Detection3D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision_msgs::Detection3D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision_msgs::Detection3D_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision_msgs::Detection3D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision_msgs::Detection3D_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vision_msgs::Detection3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7f3d8e29f3ab9853108801543aec1a5d";
  }

  static const char* value(const ::vision_msgs::Detection3D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7f3d8e29f3ab9853ULL;
  static const uint64_t static_value2 = 0x108801543aec1a5dULL;
};

template<class ContainerAllocator>
struct DataType< ::vision_msgs::Detection3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vision_msgs/Detection3D";
  }

  static const char* value(const ::vision_msgs::Detection3D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vision_msgs::Detection3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Defines a 3D detection result.\n"
"#\n"
"# This extends a basic 3D classification by including position information,\n"
"#   allowing a classification result for a specific position in an image to\n"
"#   to be located in the larger image.\n"
"\n"
"Header header\n"
"\n"
"# Class probabilities. Does not have to include hypotheses for all possible\n"
"#   object ids, the scores for any ids not listed are assumed to be 0.\n"
"ObjectHypothesisWithPose[] results\n"
"\n"
"# 3D bounding box surrounding the object.\n"
"BoundingBox3D bbox\n"
"\n"
"# The 3D data that generated these results (i.e. region proposal cropped out of\n"
"#   the image). This information is not required for all detectors, so it may\n"
"#   be empty.\n"
"sensor_msgs/PointCloud2 source_cloud\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: vision_msgs/ObjectHypothesisWithPose\n"
"# An object hypothesis that contains position information.\n"
"\n"
"# The unique numeric ID of object detected. To get additional information about\n"
"#   this ID, such as its human-readable name, listeners should perform a lookup\n"
"#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\n"
"int64 id\n"
"\n"
"# The probability or confidence value of the detected object. By convention,\n"
"#   this value should lie in the range [0-1].\n"
"float64 score\n"
"\n"
"# The 6D pose of the object hypothesis. This pose should be\n"
"#   defined as the pose of some fixed reference point on the object, such a\n"
"#   the geometric center of the bounding box or the center of mass of the\n"
"#   object.\n"
"# Note that this pose is not stamped; frame information can be defined by\n"
"#   parent messages.\n"
"# Also note that different classes predicted for the same input data may have\n"
"#   different predicted 6D poses.\n"
"geometry_msgs/PoseWithCovariance pose\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseWithCovariance\n"
"# This represents a pose in free space with uncertainty.\n"
"\n"
"Pose pose\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
"float64[36] covariance\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: vision_msgs/BoundingBox3D\n"
"# A 3D bounding box that can be positioned and rotated about its center (6 DOF)\n"
"# Dimensions of this box are in meters, and as such, it may be migrated to\n"
"#   another package, such as geometry_msgs, in the future.\n"
"\n"
"# The 3D position and orientation of the bounding box center\n"
"geometry_msgs/Pose center\n"
"\n"
"# The size of the bounding box, in meters, surrounding the object's center\n"
"#   pose.\n"
"geometry_msgs/Vector3 size\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: sensor_msgs/PointCloud2\n"
"# This message holds a collection of N-dimensional points, which may\n"
"# contain additional information such as normals, intensity, etc. The\n"
"# point data is stored as a binary blob, its layout described by the\n"
"# contents of the \"fields\" array.\n"
"\n"
"# The point cloud data may be organized 2d (image-like) or 1d\n"
"# (unordered). Point clouds organized as 2d images may be produced by\n"
"# camera depth sensors such as stereo or time-of-flight.\n"
"\n"
"# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n"
"# points).\n"
"Header header\n"
"\n"
"# 2D structure of the point cloud. If the cloud is unordered, height is\n"
"# 1 and width is the length of the point cloud.\n"
"uint32 height\n"
"uint32 width\n"
"\n"
"# Describes the channels and their layout in the binary data blob.\n"
"PointField[] fields\n"
"\n"
"bool    is_bigendian # Is this data bigendian?\n"
"uint32  point_step   # Length of a point in bytes\n"
"uint32  row_step     # Length of a row in bytes\n"
"uint8[] data         # Actual point data, size is (row_step*height)\n"
"\n"
"bool is_dense        # True if there are no invalid points\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/PointField\n"
"# This message holds the description of one point entry in the\n"
"# PointCloud2 message format.\n"
"uint8 INT8    = 1\n"
"uint8 UINT8   = 2\n"
"uint8 INT16   = 3\n"
"uint8 UINT16  = 4\n"
"uint8 INT32   = 5\n"
"uint8 UINT32  = 6\n"
"uint8 FLOAT32 = 7\n"
"uint8 FLOAT64 = 8\n"
"\n"
"string name      # Name of field\n"
"uint32 offset    # Offset from start of point struct\n"
"uint8  datatype  # Datatype enumeration, see above\n"
"uint32 count     # How many elements in the field\n"
;
  }

  static const char* value(const ::vision_msgs::Detection3D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vision_msgs::Detection3D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.results);
      stream.next(m.bbox);
      stream.next(m.source_cloud);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Detection3D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vision_msgs::Detection3D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vision_msgs::Detection3D_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "results[]" << std::endl;
    for (size_t i = 0; i < v.results.size(); ++i)
    {
      s << indent << "  results[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::vision_msgs::ObjectHypothesisWithPose_<ContainerAllocator> >::stream(s, indent + "    ", v.results[i]);
    }
    s << indent << "bbox: ";
    s << std::endl;
    Printer< ::vision_msgs::BoundingBox3D_<ContainerAllocator> >::stream(s, indent + "  ", v.bbox);
    s << indent << "source_cloud: ";
    s << std::endl;
    Printer< ::sensor_msgs::PointCloud2_<ContainerAllocator> >::stream(s, indent + "  ", v.source_cloud);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISION_MSGS_MESSAGE_DETECTION3D_H
