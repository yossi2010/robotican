//
// Created by tom on 09/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_IDEVICE_H
#define ROBOTICAN_HARDWARE_INTERFACE_IDEVICE_H

#include <ros/ros.h>
#include <robotican_hardware_interface/Protocol.h>
#include <robotican_hardware_interface/TransportLayer.h>
#include <robotican_hardware_interface/jointInfo.h>

namespace robotican_hardware {
    /*!
     * @brief An abstract class of devices, when a new device need to be created its must inherit from this class
     */
    class Device {
    private:
        byte _id;                                                                   //!< This represent the unique id of he device
        bool _ready;                                                                //!< Flag to indicate if the device is ready or not. means that the RiCBoard is also build the device in its side
    protected:
        ros::NodeHandle _nodeHandle;
        TransportLayer* _transportLayer;                                            //!< Needed to send msgs to the RiCBoard
    public:
        Device(byte id, TransportLayer *transportLayer);
        byte getId();
        void setId(byte id);
        bool isReady();
        void setReady(bool ready);
        /*!
         * @brief Call one time when the device gets its unique ack msg
         * @param ack: The ack to be check
         */
        virtual void deviceAck(const DeviceAck *ack);
        /*!
         * @brief This method will receive the new data from the RiCBoard, associated to the instance of the device.
         * @param deviceMessage: Msg that contain data associated to the instance of the device
         */
        virtual void update(const DeviceMessage * deviceMessage)=0;
        /*!
         * @brief This method will send msgs to the RiCBoard.
         */
        virtual void write() =0;
        /*!
         * @brief This method will build the device from both ends.(RiCBoard side and the pc)
         */
        virtual void buildDevice() =0;
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_IDEVICE_H
