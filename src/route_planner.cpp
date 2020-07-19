#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // use manhattan distance formula for h value
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // populate current node neighbors vector with all the neighbors
    current_node->FindNeighbors();
    // get current node neighbors
    // std::vector<RouteModel::Node*>  parent_neighbors = current_node->neighbors;
    // loop thrugh each neighbor
    for (int i=0 ; i < current_node->neighbors.size(); ++i){
      RouteModel::Node *neighbor = current_node->neighbors[i];
      // set parent
      neighbor->parent  =  current_node;
      // set g value
      neighbor->g_value =  current_node->g_value + current_node->distance(*neighbor);
      // set h value
      neighbor->h_value =  this->CalculateHValue(neighbor);
      // set node visited
      neighbor->visited =  true;
      // add node open_list
      open_list.emplace_back(neighbor);
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // lambdas function to sort nodes
    auto NodesSort = [&] (std::vector<RouteModel::Node*> *list){
    std::sort(list->begin(),list->end(), [](const RouteModel::Node *a, const RouteModel::Node *b){
      return (a->g_value + a->h_value) > (b->g_value + b->h_value);
    });
    };
    // sort the open_list according to sum of h+ g values (high->low)
    NodesSort(&open_list);

    // lambdas function to sort nodes
    // std::sort(open_list.begin(),open_list.end(), [](const auto &a, const auto &b){
    //   return (a->g_value + a->h_value) > (b->g_value + b->h_value);
    // });


    // std::sort(open_list.begin(),open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b){
    //   return (a->g_value + a->h_value) > (b->g_value + b->h_value);
    // });


    // pointer to node with lowest sum at back
    RouteModel::Node *ptr_node = open_list.back();
    // remove node from open list
    open_list.pop_back();
   	// open_list_.erase(open_list_.back()); // or
    return ptr_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    // while(current_node->parent != nullptr) //or
    while(current_node!=start_node){
      path_found.insert(path_found.begin(), *current_node); // alternative solution of insert use emplace_back and after loop use  >  std::reverse(path_found.begin(), path_found.end())
    	const RouteModel::Node parent = *(current_node->parent);
      distance += current_node->distance(parent);
      current_node = current_node->parent;
    }
    path_found.insert(path_found.begin(), *start_node); // add start node to front of vector
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    // TODO: Implement your solution here.
    current_node = start_node;
    current_node->visited = true;
    /* while goal is not reached  */
    while(current_node->distance(*end_node) != 0){
      // expand and get best next node to explore
      this->AddNeighbors(current_node);
      current_node  = this->NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);
}
