#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: TODONE
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    start_node->h_value = CalculateHValue(start_node);
}


// TODO 3: TODONE

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(*node);
}

// Helper functions to mantain heap invariant.
void RoutePlanner::PushToOpen(RouteModel::Node *node) {
  open_list.push_back(node);
  std::push_heap(open_list.begin(), open_list.end(), cmp);
}

// TODO 4: TODONE

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
  current_node->visited = true;

	for (auto n : current_node->neighbors) {
		n->parent = current_node;
        n->g_value = current_node->g_value + current_node->distance(*n);
		n->h_value = CalculateHValue(n);

    // Check by FindNeighbors so no need to check again here
		n->visited = true;
    PushToOpen(n);
	}
}

// Helper functions to mantain heap invariant.
 RouteModel::Node* RoutePlanner::PopFromOpen() {
  RouteModel::Node *n = open_list.front();
  std::pop_heap(open_list.begin(), open_list.end(), cmp);
  open_list.pop_back();
  return n;
}

// TODO 5: TODONE

RouteModel::Node *RoutePlanner::NextNode() {
  return PopFromOpen();
}

// TODO 6: TODONE

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != nullptr) {
        path_found.push_back(*current_node);
        if (current_node->parent != nullptr) {
            distance += current_node->distance(*(current_node->parent));
        }
        current_node = current_node->parent;
    }

    // The loop above includes the start node as the last element; reverse to get correct order
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: TODONE

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    open_list.clear();
    while(current_node != end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(end_node);
}
