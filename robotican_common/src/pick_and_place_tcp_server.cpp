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

void recover(ros::ServiceClient &pickAndPlaceClient, socket_ptr &client);

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_and_place_server");

    ros::NodeHandle nodeHandle;
    ros::ServiceClient pickAndPlaceClient = nodeHandle.serviceClient<std_srvs::Trigger>("pick_go");
    pickAndPlaceClient.waitForExistence();

    boost::asio::io_service io_service;
    tcp::acceptor server(io_service, tcp::endpoint(tcp::v4(), 5001));
    socket_ptr client(new tcp::socket(io_service));

    ROS_INFO("[%s]: Server up and waiting for client to connect....", ros::this_node::getName().c_str());

    server.accept(*client);
    ROS_INFO("[%s]: Got new connection", ros::this_node::getName().c_str());
    std_srvs::Trigger::Response pickAndPlaceRes = pickAndPlace(pickAndPlaceClient);

    if (pickAndPlaceRes.success) {
        boost::asio::write(*client, boost::asio::buffer("go\n", 3));
    } else {
        recover(pickAndPlaceClient, client);
    }

    while(ros::ok()) {
        char data[MAX_LEN] = {'\0'};

        boost::asio::read(*client, boost::asio::buffer(data, 3));
        if (std::strcmp(data, "go\n") == 0) {
            if (pickAndPlace(pickAndPlaceClient).success) {
                boost::asio::write(*client, boost::asio::buffer("go\n", 3));
            } else {
                recover(pickAndPlaceClient, client);
            }
        } else {
            ROS_WARN("[%s]: Unknown syntax: %s", ros::this_node::getName().c_str(), data);
        }
    }



    return 0;
}

void recover(ros::ServiceClient &pickAndPlaceClient, socket_ptr &client) {
    ROS_WARN("[%s]: plan failed type 'r' to re-plan or 'q' to quit", ros::this_node::getName().c_str());
    char choice;
    do {
        std::cin >> choice;
        if(choice == 'r') {
            if (pickAndPlace(pickAndPlaceClient).success) {
                write(*client, boost::asio::buffer("go\n", 3));
                break;
            }
        } else if(choice == 'q') {
            ros::shutdown();
            break;
        }
    } while(ros::ok());
}


std_srvs::Trigger::Response pickAndPlace(ros::ServiceClient &pickAndPlaceClient) {
    std_srvs::Trigger::Request pickAndPlaceReq;
    std_srvs::Trigger::Response pickAndPlaceRes;

    if (pickAndPlaceClient.call(pickAndPlaceReq, pickAndPlaceRes)) {
        ROS_INFO_STREAM(pickAndPlaceRes);
    }
    return pickAndPlaceRes;
}
