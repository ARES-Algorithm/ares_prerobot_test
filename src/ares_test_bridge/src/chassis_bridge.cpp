#include "ares_protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <array>
#include <cmath>
#include <cstring>
#include <chrono>
#include <map>
using namespace std::chrono_literals;
class Bridge : public rclcpp::Node {
 public:
  Bridge():Node("chassis_bridge") {
    auto profile=declare_parameter<std::string>("firmware_profile", "unconfirmed");
    if(profile=="r2v2_4f") count_=4;
    else if(profile=="r2_3f") count_=3;
    else throw std::runtime_error("Set firmware_profile to r2v2_4f or r2_3f after checking the flashed firmware");
    timeout_=declare_parameter<double>("command_timeout",0.05);
    if(!std::isfinite(timeout_) || timeout_<=0) throw std::runtime_error("command_timeout must be positive");
    status_=create_publisher<std_msgs::msg::Bool>("/prerobot/bridge_ready",10);
    sent_=create_publisher<geometry_msgs::msg::TwistStamped>("/prerobot/sent_cmd",10);
    // Publishers are created before the USB read thread starts.
    raw_=create_publisher<std_msgs::msg::Float32MultiArray>("/prerobot/usb_rx",100);
    protocol_.register_sync_callback([this](uint16_t id,const uint8_t* bytes,size_t len){
      if(len%4) return;
      std_msgs::msg::Float32MultiArray m;
      m.layout.data_offset=id; m.data.resize(len/4);
      std::memcpy(m.data.data(),bytes,len); raw_->publish(m);
    });
    if(!protocol_.connect()) throw std::runtime_error("ARES USB 1209:0001 unavailable");
    ready_=true;
    sub_=create_subscription<geometry_msgs::msg::TwistStamped>("/prerobot/cmd_vel",rclcpp::QoS(1),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){
        double age=(now()-rclcpp::Time(m->header.stamp)).seconds();
        if(age<0 || age>timeout_ || m->header.frame_id!="command") return;
        auto t=m->twist;
        if(!std::isfinite(t.linear.x)||!std::isfinite(t.linear.y)||!std::isfinite(t.angular.z)) return;
        cmd_={float(t.linear.x),float(t.linear.y),float(t.angular.z),0};
        last_=std::chrono::steady_clock::now();
      });
    timer_=create_wall_timer(10ms,[this]{tick();});
  }
  ~Bridge() override { protocol_.set_heartbeat_enabled(false); protocol_.disconnect(); }
 private:
  void tick(){
    const bool fresh = std::chrono::duration<double>(
      std::chrono::steady_clock::now()-last_).count() <= timeout_;
    protocol_.set_heartbeat_enabled(ready_ && fresh);
    if (ready_ && fresh) {
      if (!protocol_.send_sync(0x1101,reinterpret_cast<uint8_t*>(cmd_.data()),count_*4)) {
        ready_=false;
        protocol_.set_heartbeat_enabled(false);
        RCLCPP_ERROR(get_logger(),"USB TX failed; restart bridge before next command");
      } else {
        geometry_msgs::msg::TwistStamped m;
        m.header.stamp=now(); m.header.frame_id="command";
        m.twist.linear.x=cmd_[0]; m.twist.linear.y=cmd_[1]; m.twist.angular.z=cmd_[2];
        sent_->publish(m);
      }
    }
    std_msgs::msg::Bool s; s.data=ready_; status_->publish(s);
  }
  ares::Protocol protocol_;
  int count_; double timeout_; bool ready_=false;
  std::array<float,4> cmd_{};
  std::chrono::steady_clock::time_point last_{};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr raw_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr sent_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){ rclcpp::init(argc,argv); try {auto n=std::make_shared<Bridge>(); rclcpp::spin(n);} catch(const std::exception&e){fprintf(stderr,"%s\n",e.what()); rclcpp::shutdown();return 1;} rclcpp::shutdown(); }
