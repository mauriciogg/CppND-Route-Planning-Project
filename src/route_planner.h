#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"

class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() const {return distance;}
    void AStarSearch();

    // The following methods have been made public so we can test them individually.
    void AddNeighbors(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode();

  private:
    // Add private variables or methods declarations here.
    std::vector<RouteModel::Node*> open_list;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;

    float distance = 0.0f;
    RouteModel &m_Model;

    // Helper functions to operate on open_list as heap.
    void PushToOpen(RouteModel::Node *node);
    RouteModel::Node* PopFromOpen();

    struct NodeCompare {
      bool operator()(RouteModel::Node* a, RouteModel::Node* b) const {
        // Sort in reversed order since we want highest f-value (h + g) at the back
        return (a->g_value + a->h_value) >= (b->g_value + b->h_value);
      }
    };

    NodeCompare cmp;
};

#endif
