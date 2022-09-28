#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    // Store the nodes found in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// AddNeighbors method: to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto & node: current_node->neighbors)
    {
        node->parent = current_node;
        node->g_value = current_node->g_value + node->distance(*current_node);
        node->h_value = this->CalculateHValue(node);
        node->visited = true;
        this->open_list.push_back(node);
    }
}

// NextNode method: to sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node * node_1, RouteModel::Node * node_2) 
    {
        return (node_1->g_value + node_1->h_value) > (node_2->g_value + node_2->h_value); 
    }); // third parameter of std::sort is passed as lambda function

    RouteModel::Node * lowest_sum_node = open_list.back();
    open_list.pop_back();
    return lowest_sum_node;
}


// ConstructFinalPath method: to return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.push_back(*current_node);

    while(current_node->parent != nullptr)
    {
        RouteModel::Node parent = *(current_node->parent);
        path_found.insert(path_found.begin(), parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// The A* Search algorithm

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    open_list.push_back(start_node);
    start_node->visited = true;

    while(!open_list.empty())
    {
        current_node = NextNode();
        
        if(current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        AddNeighbors(current_node);
    }
}