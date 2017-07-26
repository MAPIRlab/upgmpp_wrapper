# ID of this node (client must make sure it is unique)
int32 id

# human-readable label (optional)
string label

# features of this node
float32[] features

# class multipliers ("prior probabilities")
# - can be empty (= equal prior probabilities)
# - if non-empty: same length as number of classes
float32[] class_multipliers
