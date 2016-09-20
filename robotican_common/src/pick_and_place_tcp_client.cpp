//
// Created by tom on 20/09/16.
//


#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_srvs/Trigger.h>
#include <boost/smart_ptr.hpp>


#define MAX_LEN 128
using boost::asio::ip::tcp;

typedef boost::shared_ptr<tcp::socket> socket_ptr;

std_srvs::Trigger::Response pickAndPlace(ros::ServiceClient &pickAndPlaceClient);

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_and_place_client");

    ros::NodeHandle nodeHandle;
    ros::ServiceClient pickAndPlaceClient = nodeHandle.serviceClient<std_srvs::Trigger>("pick_go");
    pickAndPlaceClient.waitForExistence();

    boost::asio::io_service io_service;
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(tcp::v4(), "localhost", "5001");
    tcp::resolver::iterator iterator = resolver.resolve(query);
    tcp::socket connection(io_service);
    connection.connect(*iterator);

    ROS_INFO("[%s]: Connected to server", ros::this_node::getName().c_str());

    while(ros::ok()) {
        char data[MAX_LEN] = {'\0'};
        boost::asio::read(connection, boost::asio::buffer(data, 3));
        if (std::strcmp(data, "go\n") == 0) {
            if (pickAndPlace(pickAndPlaceClient).success) {
                boost::asio::write(connection, boost::asio::buffer("go\n", 3));
            } else {
                ros::shutdown();
            }
        } else {
            ROS_WARN("[%s]: Unknown syntax: %s", ros::this_node::getName().c_str(), data);
        }
    }



    return 0;
}

std_srvs::Trigger::Response pickAndPlace(ros::ServiceClient &pickAndPlaceClient) {
    std_srvs::Trigger::Request pickAndPlaceReq;
    std_srvs::Trigger::Response pickAndPlaceRes;

    if (pickAndPlaceClient.call(pickAndPlaceReq, pickAndPlaceRes)) {
        ROS_INFO_STREAM(pickAndPlaceRes);
    }
    return pickAndPlaceRes;
}
