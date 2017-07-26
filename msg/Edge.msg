# ID of this edge (client must make sure it is unique)
# - all edges must be globally unique at each point in time (across all graphs)
# - removing an edge and later adding one with the same id is allowed
int32 id

# ids of the nodes that this edge connects (must already exist)
int32 node1_id
int32 node2_id

# features of this edge
float32[] features
