/*
 * Implementation of Priority Inheritance with Backtracking (PIBT)
 *
 * - ref
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. In Proceedings of the Twenty-Eighth International Joint Conference
 * on Artificial Intelligence (pp. 535–542).
 */

#pragma once
#include "solver.hpp"
#include "orientation.hpp"
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

class PIBT : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    std::optional<Orientation> ott_now;
    std::optional<Orientation> ott_next;
    int elapsed;        // eta
    int init_d;         // initial distance
    float tie_breaker;  // epsilon, tie-breaker
  };
  using Agents = std::vector<Agent*>;
  
  //
  struct Request {
    Agent* agent;      
    Node* requested_node; 
  };
  std::vector<Request> request_chain;
  bool cycle_detected = false;       
  bool cycle_handled = false;   
  Agent* initial_requester = nullptr; 
  
  private:
  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table 
  Agents occupied_now;
  Agents occupied_next;

  // option
  bool disable_dist_init = false;

  // result of priority inheritance: true -> valid, false -> invalid
  bool funcPIBT(Agent* ai, Agent* aj = nullptr, bool is_initial = true);

  // main
  void run();
  
  // minimal distance to 4 states of goal in cost table
  float getMinDistToGoal(int agent_id, Node* node, Orientation current_dir);

  // handle cycle
  void handleCycleWithOrientation();

public:
  PIBT(MAPF_Instance* _P);
  ~PIBT() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
