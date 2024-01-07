// Generated by gencpp from file tic_tac_toebot/StartGame.msg
// DO NOT EDIT!


#ifndef TIC_TAC_TOEBOT_MESSAGE_STARTGAME_H
#define TIC_TAC_TOEBOT_MESSAGE_STARTGAME_H


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
struct StartGame_
{
  typedef StartGame_<ContainerAllocator> Type;

  StartGame_()
    : start(false)  {
    }
  StartGame_(const ContainerAllocator& _alloc)
    : start(false)  {
  (void)_alloc;
    }



   typedef uint8_t _start_type;
  _start_type start;





  typedef boost::shared_ptr< ::tic_tac_toebot::StartGame_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tic_tac_toebot::StartGame_<ContainerAllocator> const> ConstPtr;

}; // struct StartGame_

typedef ::tic_tac_toebot::StartGame_<std::allocator<void> > StartGame;

typedef boost::shared_ptr< ::tic_tac_toebot::StartGame > StartGamePtr;
typedef boost::shared_ptr< ::tic_tac_toebot::StartGame const> StartGameConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tic_tac_toebot::StartGame_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tic_tac_toebot::StartGame_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tic_tac_toebot::StartGame_<ContainerAllocator1> & lhs, const ::tic_tac_toebot::StartGame_<ContainerAllocator2> & rhs)
{
  return lhs.start == rhs.start;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tic_tac_toebot::StartGame_<ContainerAllocator1> & lhs, const ::tic_tac_toebot::StartGame_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tic_tac_toebot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::tic_tac_toebot::StartGame_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tic_tac_toebot::StartGame_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tic_tac_toebot::StartGame_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tic_tac_toebot::StartGame_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tic_tac_toebot::StartGame_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tic_tac_toebot::StartGame_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tic_tac_toebot::StartGame_<ContainerAllocator> >
{
  static const char* value()
  {
    return "676aa7bfb3ec2071e814f2368dfd5fb5";
  }

  static const char* value(const ::tic_tac_toebot::StartGame_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x676aa7bfb3ec2071ULL;
  static const uint64_t static_value2 = 0xe814f2368dfd5fb5ULL;
};

template<class ContainerAllocator>
struct DataType< ::tic_tac_toebot::StartGame_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tic_tac_toebot/StartGame";
  }

  static const char* value(const ::tic_tac_toebot::StartGame_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tic_tac_toebot::StartGame_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool start # Technically we don't care what this value is, just need a signal to start\n"
;
  }

  static const char* value(const ::tic_tac_toebot::StartGame_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tic_tac_toebot::StartGame_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.start);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StartGame_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tic_tac_toebot::StartGame_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tic_tac_toebot::StartGame_<ContainerAllocator>& v)
  {
    s << indent << "start: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.start);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TIC_TAC_TOEBOT_MESSAGE_STARTGAME_H
