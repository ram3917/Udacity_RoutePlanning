#include "route_planner.h"
#include <algorithm>
#include <iostream>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);  
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{  
  // distance to the end node
  return node->distance(*(end_node));
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    // find neighbours
    current_node->FindNeighbors();

    for (auto iter : current_node->neighbors)
    {
        // fill data
        iter->parent = current_node;
        iter->h_value = CalculateHValue(iter);
        iter->g_value = current_node->g_value + iter->distance(*current_node);
        
        // is visited
        iter->visited = true;

        // add to list
        open_list.push_back(iter);
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() 
{   
    // pointer for output
    RouteModel::Node* closest_node = nullptr;

    for (auto i : open_list)
    {
        if (!i->visited)
            std::cout << "not ok" << std::endl;
    }

    // Sort open list
    std::sort(open_list.begin(), open_list.end(), []
                (RouteModel::Node* a, RouteModel::Node* b) 
                { return (a->g_value + a->h_value) < (b->g_value + b->h_value); });

    // store the nearest node and remove from list
    closest_node = open_list.front();
    open_list.erase(open_list.begin());

    //  return smallest value
    return closest_node;
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

    // add end to list
    path_found.push_back(*current_node);

    // bool to control for loop
    bool isPathFound = false;

    // store parent data
    RouteModel::Node* prev_node = current_node; 

    // TODO: Implement your solution here.
    while (!isPathFound)
    {
        // get the parent node
        auto parent_node = prev_node->parent;

        // update distance
        distance += parent_node->distance(*prev_node);

        // add to vector
        path_found.insert(path_found.begin(), *parent_node);

        // Check if parent is start
        if (IsEqual(parent_node, start_node))
        {
            // found the path
            isPathFound = true;
        }

        //update path 
        prev_node = parent_node;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() 
{
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    // get neighbours for start node
    AddNeighbors(start_node);

    while(!open_list.empty())
    {
        // get next node
        current_node = NextNode();

        if (IsEqual(current_node, end_node))
        {
            break;
        }
        
        // neighbours for next node
        AddNeighbors(current_node);        
    }

    // Get path
    m_Model.path = ConstructFinalPath(end_node);
}