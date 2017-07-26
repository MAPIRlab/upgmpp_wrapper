#include <upgmpp_wrapper/upgmpp_wrapper_node.h>

#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

using namespace upgmpp_wrapper;

//------------------------------------------------------------------------------
//                                UpgmppNode
//------------------------------------------------------------------------------

UpgmppNode::UpgmppNode() : m_nodeTypePtr( new UPGMpp::CNodeType() ),
                         m_edgeTypePtr( new UPGMpp::CEdgeType() ),
                         m_weightsLoadedFromFile(false),
                         m_weightsTrained(false),
                         m_nh_priv(ros::NodeHandle("~")),
                         m_upgmpp_model_ready(false),
                         m_upgmpp_model_in_process(false)
{
    // advertise services
    m_load_model_srv    = m_nh_priv.advertiseService("load_model", &UpgmppNode::load_model, this);
    m_store_model_srv   = m_nh_priv.advertiseService("store_model", &UpgmppNode::store_model, this);
    m_create_graph_srv  = m_nh_priv.advertiseService("create_graph", &UpgmppNode::create_graph, this);
    m_add_nodes_srv     = m_nh_priv.advertiseService("add_nodes", &UpgmppNode::add_nodes, this);
    m_add_edges_srv     = m_nh_priv.advertiseService("add_edges",&UpgmppNode::add_edges, this);
    m_remove_nodes_srv  = m_nh_priv.advertiseService("remove_nodes",&UpgmppNode::remove_nodes, this);
    m_remove_edges_srv  = m_nh_priv.advertiseService("remove_edges",&UpgmppNode::remove_edges, this);
    m_remove_graph_srv  = m_nh_priv.advertiseService("remove_graph",&UpgmppNode::remove_graph, this);
    m_map_inference_srv = m_nh_priv.advertiseService("map_inference",&UpgmppNode::MAP_inference, this);
    m_training_srv      = m_nh_priv.advertiseService("training",&UpgmppNode::training, this);

    ROS_INFO("%s initialized.", m_nh_priv.getNamespace().c_str());
}

UPGMpp::CGraph* UpgmppNode::get_graph( const size_t graph_id )
{
    UPGMpp::CGraph *graph = NULL;

    for ( size_t i = 0; i < m_graphs.size(); i++ )
    {
        if ( m_graphs[i].getID() == graph_id )
            graph = &m_graphs[i];
    }

    if ( !graph )
        ROS_ERROR("Invalid graph id");

    return graph;
}

//------------------------------------------------------------------------------
//                               load_model
//------------------------------------------------------------------------------

bool UpgmppNode::load_model(upgmpp_wrapper::LoadModel::Request &req,
                  upgmpp_wrapper::LoadModel::Response &resp)
{

      ROS_INFO("Loading model from file %s", req.model_file_name.c_str());

      // create and open an archive for input
      std::ifstream ifs(req.model_file_name.c_str());

      if ( ifs.is_open() )
      {
          boost::archive::text_iarchive ia(ifs);

          ROS_INFO("Loading node information");
          ia >> m_nodeTypePtr;
          ROS_INFO("Loading edge information");
          ia >> m_edgeTypePtr;

          ROS_INFO("Closing file");
          ifs.close();

          resp.success = true;

          m_upgmpp_model_ready = true;
      }
      else
      {
          ROS_ERROR("Unable to open model file. Does it exist?");
          resp.success = false;
      }

}

//------------------------------------------------------------------------------
//                              store_model
//------------------------------------------------------------------------------

bool UpgmppNode::store_model(upgmpp_wrapper::StoreModel::Request &req,
                  upgmpp_wrapper::StoreModel::Response &resp)
{

    ROS_INFO("Storing model to file %s", req.model_file_name.c_str());

    if ( req.model_file_name == "" )
    {
        ROS_ERROR("The model file name cannot be empty!");
        resp.success = false;
        return false;
    }
    else if ( ! m_upgmpp_model_ready )
    {
        ROS_ERROR("There is no model ready to be stored, please load or train a model first.");
        resp.success = false;
        return false;
    }

    // Create and open a character archive for output

    std::ofstream ofs( req.model_file_name.c_str() , std::ofstream::out);

    boost::archive::text_oarchive oa(ofs);

    // Write weigths to file
    oa << m_nodeTypePtr;
    oa << m_edgeTypePtr;

    // Archive and stream closed when destructors are called
    ofs.close();

    resp.success = true;
    return true;

}

//------------------------------------------------------------------------------
//                               create_graph
//------------------------------------------------------------------------------

bool UpgmppNode::create_graph(upgmpp_wrapper::CreateGraph::Request &req, upgmpp_wrapper::CreateGraph::Response &resp)
{
    ROS_INFO("Creating graph with id %d", req.id);


    if ( m_upgmpp_model_ready || m_upgmpp_model_in_process )
    {
        // Check if the graph parameters are the same than those in the upgmpp model
        //
        bool theSame;
        bool error = false;

        theSame = UPGMpp::compareTwoVectors( m_nodeTypePtr->getClassNames(),
                                             req.class_names );
        if ( !theSame )
        {
            ROS_ERROR("The provided vector of class names is different from the one initially created");
            error = true;
        }

        theSame = UPGMpp::compareTwoVectors( m_nodeTypePtr->getFeatureNames(),
                                             req.node_feature_names );
        if ( !theSame )
        {
            ROS_ERROR("The provided vector of node feature names is different from the one initially created");
            error = true;
        }

        theSame = UPGMpp::compareTwoVectors( m_edgeTypePtr->getFeatureNames(),
                                             req.edge_feature_names );
        if ( !theSame )
        {
            ROS_ERROR("The provided vector of edge feature names is different from the one initially created");
            error = true;
        }

        if (error)
        {
            resp.success = false;
            return false;
        }
    }
    else
    {
        // not m_upgmpp_model_in_process, start preparing it!
        //
        UPGMpp::CNodeType nodeType( req.class_names, req.node_feature_names );
        *m_nodeTypePtr = nodeType;

        UPGMpp::CEdgeType edgeType( req.edge_feature_names, m_nodeTypePtr, m_nodeTypePtr);
        *m_edgeTypePtr = edgeType;

        m_upgmpp_model_in_process = true;
    }

    // Create graph, set the ID and insert it into the graphs vector.

    UPGMpp::CGraph graph;
    graph.setID( req.id );
    m_graphs.push_back( graph );

    ROS_INFO("Graph successfully created");

    // All done, party time

    resp.success = true;

    return true;
}

//------------------------------------------------------------------------------
//                               add_nodes
//------------------------------------------------------------------------------

bool UpgmppNode::add_nodes(upgmpp_wrapper::AddNodes::Request &req, upgmpp_wrapper::AddNodes::Response &resp)
{

    ROS_INFO("Adding nodes to graph with id %d", req.graph_id );

    // Get the graph where nodes are to be added

    UPGMpp::CGraph *graph = get_graph( (unsigned int)req.graph_id );

    // Add the nodes!! gogogogo

    size_t N_nodes = req.nodes.size();

    for ( size_t i = 0; i < N_nodes; i++ )
    {
        size_t nodeID = req.nodes[i].id;

        UPGMpp::CNodePtr nodePtr = graph->getNodeWithID( nodeID );

        // Check if the node has been already inserted in the graph. If so,
        // employ the given features to update the node features.
        if ( nodePtr )
        {            
            nodePtr->setFeatures( req.nodes[i].features );
        }
        else
        {            
            // Ok, new node!
            UPGMpp::CNodePtr nodePtr( new UPGMpp::CNode( m_nodeTypePtr,
                                                         req.nodes[i].features,
                                                         (std::string)req.nodes[i].label ) );
            nodePtr->setID ( req.nodes[i].id );

            // INFO: This functionality is disabled in this version of the code
            // Check if the class_multipliers vector is empty. If so, create a
            // class_multipliers vector with an uniform distribution
            /*if ( req.nodes[i].class_multipliers.empty() )
            {
                size_t N_classes = m_nodeTypePtr->getNumberOfClasses();
                vector<float> multipliers(N_classes, 1.0/N_classes);

                nodePtr->setClassMultipliers( multipliers );
            }
            else
                nodePtr->setClassMultipliers( req.nodes[i].class_multipliers );*/

            graph->addNode( nodePtr );
        }

    }

    //ROS_INFO_STREAM( std::endl << *graph << std::endl );

    // All done, dance time

    resp.success = true;

    return true;
}

//------------------------------------------------------------------------------
//                               add_edges
//------------------------------------------------------------------------------

bool UpgmppNode::add_edges(upgmpp_wrapper::AddEdges::Request &req, upgmpp_wrapper::AddEdges::Response &resp)
{

    ROS_INFO("adding edges to graph with id %d", req.graph_id );

    // Get the graph where edges will be inserted

    UPGMpp::CGraph *graph = get_graph( (unsigned int)req.graph_id );

    // Add the edges!

    size_t N_edges = req.edges.size();

    for ( size_t edge = 0; edge < N_edges; edge++ )
    {
        size_t edgeID = req.edges[edge].id;

        if ( graph->getEdgeWithID( edgeID ) )
        {
            graph->getEdgeWithID( edgeID )->setFeatures( req.edges[edge].features );
        }
        else
        {
            UPGMpp::CNodePtr node1Ptr = graph->getNodeWithID( req.edges[edge].node1_id );
            UPGMpp::CNodePtr node2Ptr = graph->getNodeWithID( req.edges[edge].node2_id );

            UPGMpp::CEdgePtr edgePtr ( new UPGMpp::CEdge( node1Ptr, node2Ptr,
                                                          m_edgeTypePtr,
                                                          req.edges[edge].features ) );
            edgePtr->setID( edgeID );

            graph->addEdge( edgePtr );
        }
    }

    //ROS_INFO_STREAM( std::endl << *graph << std::endl );

    //ROS_INFO("Edges added with success");

    // All done, bar time

    resp.success = true;

    return true;
}

//------------------------------------------------------------------------------
//                             remove_nodes
//------------------------------------------------------------------------------

bool UpgmppNode::remove_nodes(upgmpp_wrapper::RemoveNodes::Request &req, upgmpp_wrapper::RemoveNodes::Response &resp)
{

    ROS_INFO("removing nodres from graph id %d", req.graph_id );

    // Get the graph where nodes are to be deleted from

    UPGMpp::CGraph *graph = get_graph( (unsigned int)req.graph_id );

    //cout << "Initial graph:" << endl << *graph << endl;

    // Remove the nodes! This also removes the edges between that nodes and
    // any others.

    vector<int>::const_iterator it;

    for ( it = req.ids.begin(); it != req.ids.end(); it++ )
    {        
        size_t ID = *it;

        ROS_INFO("Removing node id %d", (int)ID );

        graph->deleteNode( ID );
    }

    //cout << "Resulting graph:" << endl << *graph << endl;

    resp.success = true;

    return true;

}

//------------------------------------------------------------------------------
//                             remove_edges
//------------------------------------------------------------------------------

bool UpgmppNode::remove_edges(upgmpp_wrapper::RemoveEdges::Request &req, upgmpp_wrapper::RemoveEdges::Response &resp)
{
    ROS_INFO("removing edges from graph id %d", req.graph_id );

    // Get the graph where edges are to be deleted from

    UPGMpp::CGraph *graph = get_graph( (unsigned int)req.graph_id );

    //cout << "Initial graph:" << endl << *graph << endl;

    // Remove the edges!

    vector<int>::const_iterator it;

    for ( it = req.ids.begin(); it != req.ids.end(); it++ )
    {
        graph->deleteEdge( *it );
    }

    //cout << "Resulting graph:" << endl << *graph << endl;

    resp.success = true;

    return true;

}

//------------------------------------------------------------------------------
//                             remove_edges
//------------------------------------------------------------------------------

bool UpgmppNode::remove_graph(upgmpp_wrapper::RemoveGraph::Request &req, upgmpp_wrapper::RemoveGraph::Response &resp)
{

    ROS_INFO("removing graph id %d", req.graph_id );

    // Get a pointer to the graph in m_grahps

    vector<UPGMpp::CGraph>::iterator it;

    for ( it = m_graphs.begin(); it != m_graphs.end(); it++ )
    {
        if ( it->getID() == req.graph_id )
            break;
    }


    if ( it == m_graphs.end() )
    {
        ROS_ERROR("There is no graph with id %d", req.graph_id);
        resp.success = false;
        return false;
    }

    // Remove it!

    m_graphs.erase(it);

    ROS_INFO( "Removed!" );

    resp.success = true;

    return true;

}

//------------------------------------------------------------------------------
//                             map_inference
//------------------------------------------------------------------------------

bool UpgmppNode::MAP_inference(upgmpp_wrapper::MAPInference::Request &req,
                   upgmpp_wrapper::MAPInference::Response &resp)
{
    ROS_INFO("Computing %s inference in grahp with id %d",req.inference_method.c_str(),req.graph_id);

    if ( !m_upgmpp_model_ready )
        ROS_ERROR("Model not ready, so the node and edge types don't have valid weights. They have to be loaded from file or obtained after a training process.");

    // Get the graph

    UPGMpp::CGraph *graph = get_graph( req.graph_id );

    // Set the inference options

    UPGMpp::TInferenceOptions options;

    options.maxIterations = 100;
    options.convergency   = 0.0001;

    std::map< size_t, size_t > results;

    // Ok, MAPInference!

    graph->computePotentials();

    if ( req.inference_method == "ICM" )
    {        
        UPGMpp::CICMInferenceMAP decodeICM;
        decodeICM.setOptions( options );
        decodeICM.infer(*graph, results );
    }
    else if ( req.inference_method == "ICMGreedy" )
    {
        UPGMpp::CICMGreedyInferenceMAP decodeICMGreedy;
        decodeICMGreedy.setOptions( options );
        decodeICMGreedy.infer( *graph, results );
    }
    else if ( req.inference_method == "Exact" )
    {
        UPGMpp::CExactInferenceMAP decodeExact;
        decodeExact.setOptions( options );
        decodeExact.infer( *graph, results );
    }
    else if ( req.inference_method == "LBP" )
    {
        UPGMpp::CLBPInferenceMAP decodeLBP;
        decodeLBP.setOptions( options );
        decodeLBP.infer( *graph, results );
    }
    else if ( req.inference_method == "AlphaExpansion" )
    {
        UPGMpp::CAlphaExpansionInferenceMAP decodeAlphaExpansion;
        decodeAlphaExpansion.setOptions( options );
        decodeAlphaExpansion.infer( *graph, results );
    }
    else if ( req.inference_method == "AlphaBetaSwap" )
    {
        UPGMpp::CAlphaBetaSwapInferenceMAP decodeAlphaBetaSawp;
        decodeAlphaBetaSawp.setOptions( options );
        decodeAlphaBetaSawp.infer( *graph, results );
    }
    else if ( req.inference_method == "TRPBP" )
    {
        UPGMpp::CTRPBPInferenceMAP    decodeTRPBP;
        decodeTRPBP.setOptions( options );
        decodeTRPBP.infer( *graph, results );
    }
    else if ( req.inference_method == "RBP" )
    {
        UPGMpp::CRBPInferenceMAP      decodeRBP;
        decodeRBP.setOptions( options );
        decodeRBP.infer( *graph, results );
    }
    else
        ROS_ERROR("Unknown MAP inference method");


    // Convert the resulting map structure to the expected vector ones

    std::map<size_t,size_t>::iterator it;

    for ( it = results.begin(); it != results.end(); it++ )
    {
        std::cout << "Node id " << it->first << " labeled as " << it->second << std::endl;

        resp.results_node_id.push_back(it->first);
        resp.results_value.push_back(it->second);
    }

    // All done! Series time.

    resp.success = true;

    return true;
}

//------------------------------------------------------------------------------
//                             training
//------------------------------------------------------------------------------

bool UpgmppNode::training(upgmpp_wrapper::Training::Request &req, upgmpp_wrapper::Training::Response &resp)
{

    ROS_INFO("Training the model! i.e. fitting weights of node and edge types using %zu graphs", req.graph_ids.size());

    // Consistecy check
    //
    if ( !req.graph_ids.size() )
        ROS_ERROR( "Vectors of graph ids cannot be empty.");

    if ( req.graph_ids.size() != req.ground_truths.size() )
        ROS_ERROR( "Vectors of graph ids and ground_truths has a different size.");

    if ( req.edge_feature_types.size() > 0 )
        if ( req.edge_feature_types.size() != m_edgeTypePtr->getWeights().size() )
            ROS_ERROR( "Vector of feature types has a different size than the number of features of the used edge type.");

    // Ok, let's go!
    //

    // Create dataset for training
    //
    UPGMpp::CTrainingDataSet trainingDataset;


    // Add node and edge types
    //
    trainingDataset.addNodeType( m_nodeTypePtr );

    if ( req.edge_feature_types.size() > 0 )
        trainingDataset.addEdgeType( m_edgeTypePtr, req.edge_feature_types );
    else
        trainingDataset.addEdgeType( m_edgeTypePtr );

    // Add graphs and ground truths
    //
    ROS_INFO("Adding graphs and ground truths to the training dataset");

    std::vector<int>::const_iterator it;

    for ( it = req.graph_ids.begin(); it != req.graph_ids.end(); it++ )
    {
        UPGMpp::CGraph *graph = get_graph( *it );

        int pos = std::find(req.graph_ids.begin(), req.graph_ids.end(), *it) - req.graph_ids.begin();

        if ( graph->getNodes().size() != req.ground_truths[pos].ground_truth.size() )
            ROS_ERROR("The number of nodes in the graph (%zu) and in the ground truth vector (%zu) are different.",
                      graph->getNodes().size(),
                      req.ground_truths[pos].ground_truth.size());
        else
        {
            std::map<size_t,size_t> groundTruth;

            for ( size_t node = 0; node < req.ground_truths[pos].ground_truth.size(); node++ )
                groundTruth[ req.ground_truths[pos].node_ids[node] ] =
                                                req.ground_truths[pos].ground_truth[node];

            trainingDataset.addGraph( *graph );
            trainingDataset.addGraphGroundTruth( groundTruth );
        }
    }

    //std::cout << *m_nodeTypePtr << std::endl;
    //std::cout << *m_edgeTypePtr << std::endl;

    // Time to train!
    //
    ROS_INFO("Starting training!");

    UPGMpp::TTrainingOptions to;
    to.linearSearchMethod   = 5;
    to.l2Regularization     = true;
    to.nodeLambda           = 10;
    to.edgeLambda           = 1;
    to.showTrainedWeights   = true;
    to.showTrainingProgress = true;
    to.maxIterations        = 5000;

    trainingDataset.setTrainingOptions( to );

    trainingDataset.train();

    m_upgmpp_model_in_process = false;
    m_upgmpp_model_ready = true;

    ROS_INFO("Training successfully completed.");

    m_weightsTrained = true;

    resp.success = true;

    return true;

}

//------------------------------------------------------------------------------
//                                main
//------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "upgmpp_wrapper");
  upgmpp_wrapper::UpgmppNode node;

  ros::spin();
  return 0;
}
