//
// Created by tom on 27/10/16.
//

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57142
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

bool initDxl(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler);

int main(int argc, char** argv) {

    ros::init(argc, argv, "dynamixel_test_node");
    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);


    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl_present_position = 0;              // Present position

    //Init dxl motor protocol ver 1.0
    if(initDxl(portHandler, packetHandler)) {
        while(ros::ok())
        {

            // Write goal position
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                packetHandler->printTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
                packetHandler->printRxPacketError(dxl_error);
            }

            do
            {
                // Read present position
                dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->printTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->printRxPacketError(dxl_error);
                }

                ROS_INFO("[%s]: [ID:%03d] GoalPos:%03d  PresPos:%03d", ros::this_node::getName().c_str(),DXL_ID, dxl_goal_position[index], dxl_present_position);

            }while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

            // Change goal position
            if (index == 0)
            {
                index = 1;
            }
            else
            {
                index = 0;
            }
        }

        // Disable Dynamixel Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            packetHandler->printTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
            packetHandler->printRxPacketError(dxl_error);
        }

        // Close port
        portHandler->closePort();

    }
    else {
        ros::shutdown();
    }
    ros::waitForShutdown();
    return 0;
}

bool initDxl(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error;

    //Open port
    if(portHandler->openPort()) {
        ROS_INFO("[%s]: Succeeded to open port: %s", ros::this_node::getName().c_str(), DEVICENAME);
        if(portHandler->setBaudRate(BAUDRATE)) {
            ROS_INFO("[%s]: Succeeded to change baudrate: %d", ros::this_node::getName().c_str(), BAUDRATE);

            // Enable Dynamixel Torque
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            if(dxl_comm_result != COMM_SUCCESS) {
                packetHandler->printTxRxResult(dxl_comm_result);
            } else if(dxl_error != 0) {
                packetHandler->printRxPacketError(dxl_error);
            } else {
                ROS_INFO("[%s]: Succeeded to open port: %s", ros::this_node::getName().c_str(), DEVICENAME);
                return true;
            }

        } else {
            ROS_INFO("[%s]: Failed to change baudrate: %d", ros::this_node::getName().c_str(), BAUDRATE);
        }
    } else {
        ROS_INFO("[%s]: Failed to open port: %s", ros::this_node::getName().c_str(), DEVICENAME);
    }

    return false;
}