# upgmpp_wrapper
### A ROS wrapper for the UPGMpp library

<em>upgmpp_wrapper</em> provides a collection of services within the ROS environment for building, training, and performing inference over Undirected Graphical Models (like Conditional Random Fields), based on the [Undirected Probabilistic Graphical Models in C++ (UPGMpp) library](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/201-the-upgmpp-library.html). It is simple to use, and also comes with a test node that illustrates how it works.

## Dependencies

All you need to run the <em>upgmpp_wrapper</em> is to install the UPGMpp library in yout system. Please check the [library repository](https://github.com/jotaraul/upgmpp) for further information.

## Workflow

The goal of the graphical model is to be able to perform inference queries, whose give you the most probable assigment to the nodes (random variables) in the graph. For that, it is needed to previously train a model.

The pipeline for that is:

- Create the training dataset
  - Create graphs
  - Populate them with nodes (random variables) and edges (relations between them).
- Train the model
  - Give the ground truth information
- Store the model in a file 

After the training, you can use the resultant model for performing inference queries. Models can be also loaded from files produced by previous training processes.

## Provided services

For manipulating graphs:

- create_graph
- add_edges
- add_nodes
- remove_graph
- remove_edges
- remove_nodes

For loading/storing models:

- load_model
- store_model

For performing inference and training

- map_inference
- training
