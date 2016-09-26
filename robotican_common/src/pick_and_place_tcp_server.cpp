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

void changeColor(std::string colorName, ros::NodeHandle &nodeHandle);

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_and_place_server");

    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::ServiceClient pickAndPlaceClient = nodeHandle.serviceClient<std_srvs::Trigger>("pick_go");
    pickAndPlaceClient.waitForExistence();

    boost::asio::io_service io_service;
    tcp::acceptor server(io_service, tcp::endpoint(tcp::v4(), 5001));
    socket_ptr client(new tcp::socket(io_service));
    ros::Duration(10.0).sleep();
    changeColor("green", nodeHandle);
    ros::Duration(10.0).sleep();
    changeColor("blue", nodeHandle);
    ros::Duration(10.0).sleep();
    changeColor("red", nodeHandle);
    ros::Duration(10.0).sleep();

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


void changeColor(std::string colorName, ros::NodeHandle &nodeHandle) {
    if(colorName == "red") {
        nodeHandle.setParam("/find_object_node/H_min", 3);
        nodeHandle.setParam("/find_object_node/H_max", 160);
        nodeHandle.setParam("/find_object_node/S_min", 70);
        nodeHandle.setParam("/find_object_node/S_max", 255);
        nodeHandle.setParam("/find_object_node/V_min", 10);
        nodeHandle.setParam("/find_object_node/V_max", 255);
        nodeHandle.setParam("/find_object_node/A_min", 0);
        nodeHandle.setParam("/find_object_node/A_max", 50000);
        nodeHandle.setParam("/find_object_node/gaussian_ksize", 0);
        nodeHandle.setParam("/find_object_node/gaussian_sigma", 0);
        nodeHandle.setParam("/find_object_node/morph_size", 0);
        nodeHandle.setParam("/find_object_node/invert_Hue", true);
        ROS_INFO("[%s]: Change to red", ros::this_node::getName().c_str());
    }
    else if(colorName == "green") {
        nodeHandle.setParam("/find_object_node/H_min", 37);
        nodeHandle.setParam("/find_object_node/H_max", 79);
        nodeHandle.setParam("/find_object_node/S_min", 40);
        nodeHandle.setParam("/find_object_node/S_max", 219);
        nodeHandle.setParam("/find_object_node/V_min", 5);
        nodeHandle.setParam("/find_object_node/V_max", 91);
        nodeHandle.setParam("/find_object_node/A_min", 0);
        nodeHandle.setParam("/find_object_node/A_max", 50000);
        nodeHandle.setParam("/find_object_node/gaussian_ksize", 0);
        nodeHandle.setParam("/find_object_node/gaussian_sigma", 0);
        nodeHandle.setParam("/find_object_node/morph_size", 0);
        nodeHandle.setParam("/find_object_node/invert_Hue", false);
        ROS_INFO("[%s]: Change to green", ros::this_node::getName().c_str());
    }    else if(colorName == "blue") {
        nodeHandle.setParam("/find_object_node/H_min", 96);
        nodeHandle.setParam("/find_object_node/H_max", 109);
        nodeHandle.setParam("/find_object_node/S_min", 151);
        nodeHandle.setParam("/find_object_node/S_max", 255);
        nodeHandle.setParam("/find_object_node/V_min", 85);
        nodeHandle.setParam("/find_object_node/V_max", 152);
        nodeHandle.setParam("/find_object_node/A_min", 0);
        nodeHandle.setParam("/find_object_node/A_max", 50000);
        nodeHandle.setParam("/find_object_node/gaussian_ksize", 0);
        nodeHandle.setParam("/find_object_node/gaussian_sigma", 0);
        nodeHandle.setParam("/find_object_node/morph_size", 0);
        nodeHandle.setParam("/find_object_node/invert_Hue", false);
        ROS_INFO("[%s]: Change to blue", ros::this_node::getName().c_str());
    }
}