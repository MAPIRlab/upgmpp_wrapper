#ifndef UpgmppNode_H
#define UpgmppNode_H

#include <ros/ros.h>
#include <upgmpp_wrapper/AddNodes.h>
#include <upgmpp_wrapper/AddEdges.h>
#include <upgmpp_wrapper/CreateGraph.h>
#include <upgmpp_wrapper/LoadModel.h>
#include <upgmpp_wrapper/RemoveNodes.h>
#include <upgmpp_wrapper/RemoveEdges.h>
#include <upgmpp_wrapper/RemoveGraph.h>
#include <upgmpp_wrapper/MAPInference.h>
#include <upgmpp_wrapper/StoreModel.h>
#include <upgmpp_wrapper/Training.h>

#include <base.hpp>
#include <inference_MAP.hpp>
#include <training.hpp>

namespace upgmpp_wrapper
{

    class UpgmppNode
    {
    public:

        UpgmppNode();

    private:

        std::vector<UPGMpp::CGraph> m_graphs;
        UPGMpp::CNodeTypePtr        m_nodeTypePtr;
        UPGMpp::CEdgeTypePtr        m_edgeTypePtr;

        bool m_upgmpp_model_ready;      // The model has been loaded from file or has been trained
        bool m_upgmpp_model_in_process; // The model is not ready, but graphs are being created for its training
        bool m_weightsLoadedFromFile;
        bool m_weightsTrained;

        ros::ServiceServer m_load_model_srv;
        ros::ServiceServer m_store_model_srv;
        ros::ServiceServer m_create_graph_srv;
        ros::ServiceServer m_add_nodes_srv;
        ros::ServiceServer m_add_edges_srv;
        ros::ServiceServer m_remove_graph_srv;
        ros::ServiceServer m_remove_nodes_srv;
        ros::ServiceServer m_remove_edges_srv;
        ros::ServiceServer m_map_inference_srv;
        ros::ServiceServer m_training_srv;
        ros::ServiceServer m_get_object_probable_locations_srv;

        ros::NodeHandle m_nh_priv;

        UPGMpp::CGraph* get_graph( const size_t graph_id );

        bool load_model(upgmpp_wrapper::LoadModel::Request &req,
                        upgmpp_wrapper::LoadModel::Response &resp);

        bool store_model(upgmpp_wrapper::StoreModel::Request &req,
                         upgmpp_wrapper::StoreModel::Response &resp);

        bool create_graph(upgmpp_wrapper::CreateGraph::Request &req,
                          upgmpp_wrapper::CreateGraph::Response &resp);

        bool add_nodes(upgmpp_wrapper::AddNodes::Request &req,
                       upgmpp_wrapper::AddNodes::Response &resp);

        bool add_edges(upgmpp_wrapper::AddEdges::Request &req,
                       upgmpp_wrapper::AddEdges::Response &resp);

        bool remove_nodes(upgmpp_wrapper::RemoveNodes::Request &req,
                       upgmpp_wrapper::RemoveNodes::Response &resp);

        bool remove_edges(upgmpp_wrapper::RemoveEdges::Request &req,
                       upgmpp_wrapper::RemoveEdges::Response &resp);

        bool remove_graph(upgmpp_wrapper::RemoveGraph::Request &req,
                       upgmpp_wrapper::RemoveGraph::Response &resp);

        bool MAP_inference(upgmpp_wrapper::MAPInference::Request &req,
                           upgmpp_wrapper::MAPInference::Response &resp);

        bool training(upgmpp_wrapper::Training::Request &req,
                           upgmpp_wrapper::Training::Response &resp);
    };

}

#endif // UpgmppNode_H

