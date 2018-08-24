/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <FPSensorROS_plugin.h>


/* Specify that the class XBotPlugin::FPSensorROS is a XBot RT plugin with name "FPSensorROS" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::FPSensorROS)

namespace XBotPlugin {

bool FPSensorROS::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    right_fps = std::dynamic_pointer_cast<XBot::IXBotFootPressureSensor>(get_xbotcore_halInterface()->getSensorId(148));
    left_fps = std::dynamic_pointer_cast<XBot::IXBotFootPressureSensor> (get_xbotcore_halInterface()->getSensorId(158));
    
    _pub_right = handle->getRosHandle()->advertise<std_msgs::Float32MultiArray>("/right_footPS", 1);
    _pub_left = handle->getRosHandle()->advertise<std_msgs::Float32MultiArray>("/left_footPS", 1);
     
    _logger = XBot::MatLogger::getLogger("/tmp/FPSensorROS_log");

    return true;


}

void FPSensorROS::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /FPSensorROS_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the robot starting config to a class member */
    _robot->getMotorPosition(_q0);

    /* Save the plugin starting time to a class member */
    _start_time = time;
}

void FPSensorROS::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /FPSensorROS_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void FPSensorROS::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* The following code checks if any command was received from the plugin standard port
     * (e.g. from ROS you can send commands with
     *         rosservice call /FPSensorROS_cmd "cmd: 'MY_COMMAND_1'"
     * If any command was received, the code inside the if statement is then executed. */

    if(!current_command.str().empty()){

        if(current_command.str() == "MY_COMMAND_1"){
            /* Handle command */
        }

        if(current_command.str() == "MY_COMMAND_2"){
            /* Handle command */
        }

    }
    
    if(right_fps){        
        if(right_fps->get_forcexy(148,right_xy)){
            std_msgs::Float32MultiArray msg;             
            for(int i = 0; i< sizeof(right_xy)/sizeof(uint8_t); i++){
                msg.data.push_back(right_xy[i]);
            }
            _pub_right->pushToQueue(msg);
        }        
    }
    
    if(left_fps){        
        if(left_fps->get_forcexy(158,left_xy)){
            std_msgs::Float32MultiArray msg;             
            for(int i = 0; i< sizeof(left_xy)/sizeof(uint8_t); i++){
                msg.data.push_back(left_xy[i]);
            }
            _pub_left->pushToQueue(msg);
        }        
    }
    

}

bool FPSensorROS::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

FPSensorROS::~FPSensorROS()
{
  
}

}
