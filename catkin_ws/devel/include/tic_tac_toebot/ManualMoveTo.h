// Generated by gencpp from file tic_tac_toebot/ManualMoveTo.msg
// DO NOT EDIT!


#ifndef TIC_TAC_TOEBOT_MESSAGE_MANUALMOVETO_H
#define TIC_TAC_TOEBOT_MESSAGE_MANUALMOVETO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tic_tac_toebot
{
template <class ContainerAllocator>
struct ManualMoveTo_
{
  typedef ManualMoveTo_<ContainerAllocator> Type;

  ManualMoveTo_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  ManualMoveTo_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> const> ConstPtr;

}; // struct ManualMoveTo_

typedef ::tic_tac_toebot::ManualMoveTo_<std::allocator<void> > ManualMoveTo;

typedef boost::shared_ptr< ::tic_tac_toebot::ManualMoveTo > ManualMoveToPtr;
typedef boost::shared_ptr< ::tic_tac_toebot::ManualMoveTo const> ManualMoveToConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator1> & lhs, const ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator1> & lhs, const ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tic_tac_toebot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc153912f1453b70ULL;
  static const uint64_t static_value2 = 0x8d221682bc23d9acULL;
};

template<class ContainerAllocator>
struct DataType< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tic_tac_toebot/ManualMoveTo";
  }

  static const char* value(const ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x # xyz positions of where we want to move the robot in cm \n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ManualMoveTo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tic_tac_toebot::ManualMoveTo_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TIC_TAC_TOEBOT_MESSAGE_MANUALMOVETO_H