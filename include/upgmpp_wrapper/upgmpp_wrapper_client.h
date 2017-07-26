#ifndef PGMPPNODE_H
#define PGMPPNODE_H

#include <ros/ros.h>
#include <upgmpp_wrapper/CreateGraph.h>
#include <upgmpp_wrapper/AddNodes.h>
#include <upgmpp_wrapper/AddEdges.h>
#include <upgmpp_wrapper/MAPInference.h>
#include <upgmpp_wrapper/RemoveNodes.h>
#include <upgmpp_wrapper/RemoveEdges.h>
#include <upgmpp_wrapper/RemoveGraph.h>
#include <upgmpp_wrapper/Training.h>
#include <upgmpp_wrapper/LoadModel.h>
#include <upgmpp_wrapper/StoreModel.h>

namespace upgmpp_wrapper
{

class UpgmPPTestClient
{
public:
    UpgmPPTestClient();

    void run_test();

private:
    ros::NodeHandle m_nh;
};

}

#endif // PGMPPNODE_H
