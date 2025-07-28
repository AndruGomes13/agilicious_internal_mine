#include <ros/ros.h>
#include "ros/console.h"
#include "ros/node_handle.h"
#include "agiros/time.hpp"
#include "agilib/bridge/sbus/sbus_bridge.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agiros_msgs/Command.h"
#include "agiros/ros_eigen.hpp"

using namespace agi;

class RosToSbusBridge {
 public:
  RosToSbusBridge(const ros::NodeHandle& nh = ros::NodeHandle(),
                  const ros::NodeHandle& pnh = ros::NodeHandle("~"))
    : nh_(nh),
     pnh_(pnh),
    sbus_bridge_(loadQuadrotorFile(pnh), loadSbusParamsFile(pnh), RosTime)
    {
        command_sub_ = nh_.subscribe("/command", 1, &RosToSbusBridge::commandCallback, this);
        arm_sub_ = nh_.subscribe("/arm", 1, &RosToSbusBridge::armCallback, this);
    }

    void commandCallback(const agiros_msgs::Command& msg) {
        if (!sbus_bridge_.isOpen()) {
            ROS_ERROR("SBUS bridge is not open!");
            return;
        }
        Command command = fromRosCommand(msg);
        sbus_bridge_.send(command);
    }

    void armCallback(const std_msgs::Bool& msg) {
        if (!sbus_bridge_.isOpen()) {
            ROS_ERROR("SBUS bridge is not open!");
            return;
        }
        if (msg.data) {
            arm();
        } else {
            sbus_bridge_.deactivate();
            sbus_bridge_.reset();
            if (sbus_bridge_.active()) {
                ROS_ERROR("Failed to deactivate SBUS bridge!");
                return;
            }
            else {
                ROS_INFO("SBUS bridge deactivated.");
            }
        }
    }

    void arm(){
        if (!sbus_bridge_.isOpen()) {
            ROS_ERROR("SBUS bridge is not open!");
            return;
        }
        sbus_bridge_.activate();
        if (!sbus_bridge_.active()) {
            ROS_ERROR("Failed to activate SBUS bridge!");
        } else {
            ROS_INFO("SBUS bridge activated.");
        }
    }
    
    private:
    static SbusParams loadSbusParamsFile(const ros::NodeHandle& nh) {
    std::string filename;
    SbusParams params;
    if (!nh.getParam("sbus_config", filename)) {
      ROS_ERROR("Could not load SBUS parameters from %s!", filename.c_str());
    } else {
      ROS_INFO("Loading SBUS parameters from %s", filename.c_str());
      params.load(filename);
    }
    ROS_INFO_STREAM(params);
    return params;
    }
    static Quadrotor loadQuadrotorFile(const ros::NodeHandle& nh) {
        std::string filename;
        Quadrotor quad;
        if (!nh.getParam("quadrotor", filename)) {
            ROS_INFO("Could not load quadrotor parameters from %s!", filename.c_str());
        } else {
            ROS_INFO("Loading quadrotor from %s", filename.c_str());
            quad.load(filename);
        }
        if (!quad.valid()) {
            ROS_ERROR("Quadrotor parameters not valid!");
        }

        return quad;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber command_sub_;
    ros::Subscriber arm_sub_;
    SbusBridge sbus_bridge_;

    
    
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_to_sbus_bridge");
  RosToSbusBridge bridge;

  ros::spin();
  return 0;
}