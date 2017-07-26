#include <upgmpp_wrapper/upgmpp_wrapper_client.h>
#include <ros/package.h>

using namespace upgmpp_wrapper;

//------------------------------------------------------------------------------
//                           UpgmPPTestClient
//------------------------------------------------------------------------------

UpgmPPTestClient::UpgmPPTestClient()
{
    // advertise services
    ROS_INFO("upgmpp_wrapper_test_client initialized.");
}

//------------------------------------------------------------------------------
//                               run_test
//------------------------------------------------------------------------------

void UpgmPPTestClient::run_test()
{

    //
    // LOAD MODEL FROM FILE
    //

    ROS_INFO("Testing load model from file...");

    ros::ServiceClient load_model_srv = m_nh.serviceClient<upgmpp_wrapper::LoadModel>("upgmpp_wrapper/load_model");
    load_model_srv.waitForExistence();

    upgmpp_wrapper::LoadModel::Request req_load;
    upgmpp_wrapper::LoadModel::Response resp_load;

    std::string package_path = ros::package::getPath("upgmpp_wrapper");
    std::string model_file_name = package_path + "/models/model_dummy_example.txt";

    req_load.model_file_name = model_file_name;

    bool call_success = load_model_srv.call(req_load, resp_load);
    if (!call_success)
        ROS_ERROR("Service call failed!");
    else if (!resp_load.success)
        ROS_ERROR("load_model returned success == false!");
    else
        ROS_INFO("load_model successful!");

    //
    // TEST CREATE_GRAPH SERVICE
    //

    ROS_INFO("Testing create graph...");

    ros::ServiceClient create_graph_srv = m_nh.serviceClient<upgmpp_wrapper::CreateGraph>("upgmpp_wrapper/create_graph");
    create_graph_srv.waitForExistence();

    upgmpp_wrapper::CreateGraph::Request req_create_graph;
    upgmpp_wrapper::CreateGraph::Response resp_create_graph;

    req_create_graph.id = 42;

    req_create_graph.edge_feature_names.push_back( std::string("heightDiff") );
    req_create_graph.edge_feature_names.push_back( std::string("sizeDiff") );
    req_create_graph.edge_feature_names.push_back( std::string("bias") );

    req_create_graph.node_feature_names.push_back( std::string("height") );
    req_create_graph.node_feature_names.push_back( std::string("size") );
    req_create_graph.node_feature_names.push_back( std::string("bias") );

    req_create_graph.class_names.push_back("table");
    req_create_graph.class_names.push_back("book");

    call_success = create_graph_srv.call(req_create_graph, resp_create_graph);
    if (!call_success)
        ROS_ERROR("Service call failed!");
    else if (!resp_create_graph.success)
        ROS_ERROR("create_graph returned success == false!");
    else
        ROS_INFO("create_graph successful!");

    //
    // TEST ADD_NODES SERVICE
    //

    ROS_INFO("Testing add nodes...");

    ros::ServiceClient add_nodes_srv = m_nh.serviceClient<upgmpp_wrapper::AddNodes>("upgmpp_wrapper/add_nodes");

    upgmpp_wrapper::AddNodes::Request req_add_nodes;
    upgmpp_wrapper::AddNodes::Response resp_add_nodes;

    req_add_nodes.graph_id = 42;

    req_add_nodes.nodes.resize(2);

    req_add_nodes.nodes[0].id = 5;
    req_add_nodes.nodes[0].label = std::string("object1");
    req_add_nodes.nodes[0].features.resize(3);
    req_add_nodes.nodes[0].features[0] = 17;
    req_add_nodes.nodes[0].features[1] = 28;
    req_add_nodes.nodes[0].features[2] = 1;

    req_add_nodes.nodes[1].id = 7;
    req_add_nodes.nodes[1].label = std::string("object2");
    req_add_nodes.nodes[1].features.resize(3);
    req_add_nodes.nodes[1].features[0] = 102;
    req_add_nodes.nodes[1].features[1] = 74;
    req_add_nodes.nodes[1].features[2] = 1;

    call_success = add_nodes_srv.call(req_add_nodes, resp_add_nodes);
    if (!call_success)
        ROS_ERROR("Service call add_nodes failed!");
    else if (!resp_add_nodes.success)
        ROS_ERROR("add_nodes returned success == false!");
    else
        ROS_INFO("add_nodes successful!");


    //
    // TEST ADD_EDGES SERVICE
    //

    ROS_INFO("Testing add edges...");

    ros::ServiceClient add_edges_srv = m_nh.serviceClient<upgmpp_wrapper::AddEdges>("upgmpp_wrapper/add_edges");

    upgmpp_wrapper::AddEdges::Request req_add_edges;
    upgmpp_wrapper::AddEdges::Response resp_add_edges;

    req_add_edges.graph_id = 42;

    req_add_edges.edges.resize(1);
    req_add_edges.edges[0].id = 1;
    req_add_edges.edges[0].node1_id = 5;
    req_add_edges.edges[0].node2_id = 7;
    req_add_edges.edges[0].features.resize(3);
    req_add_edges.edges[0].features[0] = 85;
    req_add_edges.edges[0].features[1] = 46;
    req_add_edges.edges[0].features[2] = 1;

    call_success = add_edges_srv.call(req_add_edges, resp_add_edges);
    if (!call_success)
        ROS_ERROR("Service call add_edges failed!");
    else if (!resp_add_edges.success)
        ROS_ERROR("add_edges returned success == false!");
    else
        ROS_INFO("add_edges successful!");

    //
    // ADD A NEW GRAPH
    //

    ROS_INFO("Adding a new graph...");

    req_create_graph.id = 43;

    call_success = create_graph_srv.call(req_create_graph, resp_create_graph);
    if (!call_success)
        ROS_ERROR("Service call failed!");
    else if (!resp_create_graph.success)
        ROS_ERROR("create_graph returned success == false!");
    else
        ROS_INFO("create_graph successful!");

    //
    // ADD_NODES
    //

    req_add_nodes.graph_id = 43;

    req_add_nodes.nodes.resize(2);

    req_add_nodes.nodes[0].id = 8;
    req_add_nodes.nodes[0].label = std::string("object3");
    req_add_nodes.nodes[0].features.resize(3);
    req_add_nodes.nodes[0].features[0] = 97;
    req_add_nodes.nodes[0].features[1] = 78;
    req_add_nodes.nodes[0].features[2] = 1;

    req_add_nodes.nodes[1].id = 9;
    req_add_nodes.nodes[1].label = std::string("object4");
    req_add_nodes.nodes[1].features.resize(3);
    req_add_nodes.nodes[1].features[0] = 13;
    req_add_nodes.nodes[1].features[1] = 25;
    req_add_nodes.nodes[1].features[2] = 1;

    call_success = add_nodes_srv.call(req_add_nodes, resp_add_nodes);
    if (!call_success)
        ROS_ERROR("Service call add_nodes failed!");
    else if (!resp_add_nodes.success)
        ROS_ERROR("add_nodes returned success == false!");
    else
        ROS_INFO("add_nodes successful!");

    //
    // ADD_EDGES
    //

    req_add_edges.graph_id = 43;

    req_add_edges.edges.resize(1);
    req_add_edges.edges[0].id = 1;
    req_add_edges.edges[0].node1_id = 8;
    req_add_edges.edges[0].node2_id = 9;
    req_add_edges.edges[0].features.resize(3);
    req_add_edges.edges[0].features[0] = 84;
    req_add_edges.edges[0].features[1] = 53;
    req_add_edges.edges[0].features[2] = 1;

    call_success = add_edges_srv.call(req_add_edges, resp_add_edges);
    if (!call_success)
        ROS_ERROR("Service call add_edges failed!");
    else if (!resp_add_edges.success)
        ROS_ERROR("add_edges returned success == false!");
    else
        ROS_INFO("add_edges successful!");

    //
    // TEST INFERENCE SERVICE
    //

    ROS_INFO("Testing inference...");

    ros::ServiceClient map_inference_srv = m_nh.serviceClient<upgmpp_wrapper::MAPInference>("upgmpp_wrapper/map_inference");

    upgmpp_wrapper::MAPInference::Request req_inference;
    upgmpp_wrapper::MAPInference::Response resp_inference;

    req_inference.graph_id = 42;
    req_inference.inference_method = std::string("ICMGreedy");


    call_success = map_inference_srv.call(req_inference, resp_inference);
    if (!call_success)
        ROS_ERROR("Service call MAP_inference failed!");
    else if (!resp_inference.success)
        ROS_ERROR("MAP_inference returned success == false!");
    else
        ROS_INFO("MAP_inference successful!");

    for ( size_t node = 0; node < resp_inference.results_node_id.size(); node++ )
    {
        std::cout << "Node id: " << resp_inference.results_node_id[node]
                     << " classified as " << resp_inference.results_value[node] << std::endl;
    }

    //
    // TEST TRAINING
    //

    ROS_INFO("Testing training...");

    ros::ServiceClient training_srv = m_nh.serviceClient<upgmpp_wrapper::Training>("upgmpp_wrapper/training");

    upgmpp_wrapper::Training::Request reqTraining;
    upgmpp_wrapper::Training::Response respTraining;

    reqTraining.file_name= "weights.txt";

    // Graphs used during training
    reqTraining.graph_ids.push_back( 42 );
    reqTraining.graph_ids.push_back( 43 );

    // Ground truths
    upgmpp_wrapper::GroundTruth groundTruth1;
    upgmpp_wrapper::GroundTruth groundTruth2;

    groundTruth1.node_ids.push_back( 5 );
    groundTruth1.ground_truth.push_back( 1 );
    groundTruth1.node_ids.push_back( 7 );
    groundTruth1.ground_truth.push_back( 0 );

    groundTruth2.node_ids.push_back( 8 );
    groundTruth2.ground_truth.push_back( 0 );
    groundTruth2.node_ids.push_back( 9 );
    groundTruth2.ground_truth.push_back( 1 );

    reqTraining.ground_truths.push_back( groundTruth1 );
    reqTraining.ground_truths.push_back( groundTruth2 );

    // Type of the edge features
    reqTraining.edge_feature_types.push_back(0);
    reqTraining.edge_feature_types.push_back(0);
    reqTraining.edge_feature_types.push_back(0);

    // Let's go!
    call_success = training_srv.call(reqTraining, respTraining);
    if (!call_success)
        ROS_ERROR("Service call training failed!");
    else if (!respTraining.success)
        ROS_ERROR("training returned success == false!");
    else
        ROS_INFO("training successful!");

    //
    // STORE MODEL IN FILE
    //

    ROS_INFO("Testing store model in file...");

    ros::ServiceClient store_model_srv = m_nh.serviceClient<upgmpp_wrapper::StoreModel>("upgmpp_wrapper/store_model");
    store_model_srv.waitForExistence();

    upgmpp_wrapper::StoreModel::Request req_store;
    upgmpp_wrapper::StoreModel::Response resp_store;

    model_file_name = package_path + "/models/stored_model_dummy_example.txt";

    req_store.model_file_name = model_file_name;

    call_success = store_model_srv.call(req_store, resp_store);
    if (!call_success)
        ROS_ERROR("Service call failed!");
    else if (!resp_store.success)
        ROS_ERROR("store_model returned success == false!");
    else
        ROS_INFO("store_model successful!");


    //
    // TEST DELETING NODES AND EDGES
    //

    ROS_INFO("Testing remove nodes and edges...");

    // 1. Add a new node with an edge linking an already existing node.
    // 2. Delete the edge.
    // 3. Add the edge again.
    // 4. Delete the node, which also deletes the edge.

    // 1. Add a new node with an edge linking an already existing node.

    upgmpp_wrapper::AddNodes::Request req51;
    upgmpp_wrapper::AddNodes::Response resp52;

    req51.graph_id = 42;

    req51.nodes.resize(1);

    req51.nodes[0].id = 14;
    req51.nodes[0].label = std::string("object3");
    req51.nodes[0].features.resize(3);
    req51.nodes[0].features[0] = 24;
    req51.nodes[0].features[1] = 27;
    req51.nodes[0].features[2] = 1;
    req51.nodes[0].class_multipliers.resize(2);
    req51.nodes[0].class_multipliers[0] = 1;
    req51.nodes[0].class_multipliers[1] = 1;

    add_nodes_srv.call(req51, resp52);

    upgmpp_wrapper::AddEdges::Request req53;
    upgmpp_wrapper::AddEdges::Response resp54;

    req53.graph_id = 42;

    req53.edges.resize(1);

    req53.edges[0].id = 2;
    req53.edges[0].node1_id = 5;
    req53.edges[0].node2_id = 14;
    req53.edges[0].features.resize(3);
    req53.edges[0].features[0] = 75;
    req53.edges[0].features[1] = 34;
    req53.edges[0].features[2] = 1;

    call_success = add_edges_srv.call(req53, resp54 );
    if (!call_success)
        ROS_ERROR("Service call add_edges failed!");
    else if (!resp54.success)
        ROS_ERROR("add_edges returned success == false!");
    else
        ROS_INFO("add_edges successful!");

    // 2. Remove the edge

    ros::ServiceClient remove_edges_srv = m_nh.serviceClient<upgmpp_wrapper::RemoveEdges>("upgmpp_wrapper/remove_edges");

    upgmpp_wrapper::RemoveNodes::Request req55;
    upgmpp_wrapper::RemoveNodes::Response resp56;

    req55.graph_id = 42;

    req55.ids.resize(1);
    req55.ids[0] = 2;

    call_success = remove_edges_srv.call(req55, resp56);
    if (!call_success)
        ROS_ERROR("Service call remove_edges failed!");
    else if (!resp56.success)
        ROS_ERROR("remove_edges returned success == false!");
    else
        ROS_INFO("remove_edges successful!");

    // 3. Add the edge again.

    call_success = add_edges_srv.call(req53, resp54 );
    if (!call_success)
        ROS_ERROR("Service call add_edges failed!");
    else if (!resp54.success)
        ROS_ERROR("add_edges returned success == false!");
    else
        ROS_INFO("add_edges successful!");

    // 4. Remove the node.

    ros::ServiceClient remove_nodes_srv = m_nh.serviceClient<upgmpp_wrapper::RemoveNodes>("upgmpp_wrapper/remove_nodes");

    upgmpp_wrapper::RemoveNodes::Request req57;
    upgmpp_wrapper::RemoveNodes::Response resp58;

    req57.graph_id = 42;

    req57.ids.resize(1);
    req57.ids[0] = 14;

    call_success = remove_nodes_srv.call(req57, resp58);
    if (!call_success)
        ROS_ERROR("Service call remove_nodes failed!");
    else if (!resp58.success)
        ROS_ERROR("remove_nodes returned success == false!");
    else
        ROS_INFO("remove_nodes successful!");

    //
    // TEST REMOVE GRAPH
    //

    ROS_INFO("Testing remove graph...");

    ros::ServiceClient remove_graph_srv = m_nh.serviceClient<upgmpp_wrapper::RemoveGraph>("upgmpp_wrapper/remove_graph");

    upgmpp_wrapper::RemoveGraph::Request req_rm_graph;
    upgmpp_wrapper::RemoveGraph::Response resp_rm_graph;

    req_rm_graph.graph_id = 42;

    call_success = remove_graph_srv.call(req_rm_graph, resp_rm_graph);
    if (!call_success)
        ROS_ERROR("Service call remove_graph failed!");
    else if (!resp_rm_graph.success)
        ROS_ERROR("remove_graph returned success == false!");
    else
        ROS_INFO("remove_graph successful!");

}

//------------------------------------------------------------------------------
//                                  main
//------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pgmpp_test_client");
  upgmpp_wrapper::UpgmPPTestClient node;

  node.run_test();

  ros::spinOnce();
  return 0;
}

