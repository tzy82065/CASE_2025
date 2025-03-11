#include "../include/pibt.hpp"

// 测试：输出方向

std::string orientationToString(Orientation dir) {
    switch (dir) {
        case Orientation::X_PLUS:  return "X_PLUS";
        case Orientation::X_MINUS: return "X_MINUS";
        case Orientation::Y_PLUS:  return "Y_PLUS";
        case Orientation::Y_MINUS: return "Y_MINUS";
        default: return "UNKNOWN";
    }
}

const std::string PIBT::SOLVER_NAME = "PIBT";

PIBT::PIBT(MAPF_Instance* _P)
    : MAPF_Solver(_P), 
      occupied_now(Agents(G->getNodesSize(), nullptr)),
      occupied_next(Agents(G->getNodesSize(), nullptr))
{
  solver_name = PIBT::SOLVER_NAME;
}


void PIBT::run()
{
  
  std::cout << "=== Starting PIBT::run() ===" << std::endl;
  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
    // use initial distance
    if (a->init_d != b->init_d) return a->init_d > b->init_d;
    return a->tie_breaker > b->tie_breaker;
  };
  Agents A;

  // initialize
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Node* g = P->getGoal(i);
    int d = disable_dist_init ? 0 : pathDist(i,s,Orientation::Y_MINUS);
    Agent* a = new Agent{i,                          // id
                         s,                          // current location
                         nullptr,                    // next location
                         g,                          // goal
                         Orientation::Y_MINUS,        // current orientation
                         std::nullopt,               // next orientation
                         0,                          // elapsed
                         d,                          // dist from s -> g
                         getRandomFloat(0, 1, MT)};  // tie-breaker
    A.push_back(a);
    occupied_now[s->id] = a;
  }
  Config initial_config = P->getConfigStart();
  std::vector<Orientation> initial_orients(P->getNum(), Orientation::Y_MINUS);
  solution.addWithOrientation(initial_config, initial_orients);

  // main loop
  int timestep = 0;
  int max_loop = 25;
  while (true) {
  //while (timestep < max_loop) {
    info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

    for (size_t i = 0; i < occupied_next.size(); ++i) {
            if (occupied_next[i] != nullptr) {
                std::cout << "Warning: occupied_next[" << i << "] not cleared from previous timestep" << std::endl;}}
            
    // planning
    std::sort(A.begin(), A.end(), compare);
    for (auto a : A) {
      // if the agent has next location, then skip
      if (a->v_next == nullptr) {
        // determine its next location
        funcPIBT(a);
      }
    }

    // plan.cpp
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr); //plan.cpp
    std::vector<Orientation> orients(P->getNum());

    for (auto a : A) {
      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
      occupied_next[a->v_next->id] = nullptr;
      
      // set next location and orientation
      config[a->id] = a->v_next;
      orients[a->id] = a->ott_next.value(); 
      occupied_now[a->v_next->id] = a;

      // check goal condition
      check_goal_cond &= (a->v_next == a->g);
      // update priority
      a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
      // reset params
      a->v_now = a->v_next;
      a->v_next = nullptr;

      a->ott_now = a->ott_next;
      a->ott_next = std::nullopt;
    }

    // update plan
    solution.addWithOrientation(config, orients);

    ++timestep;

    // success
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // failed
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }

  // memory clear
  for (auto a : A) {
    if (a) {
        delete a;
    }
  }
  A.clear();
}

bool PIBT::funcPIBT(Agent* ai, Agent* aj, bool is_initial)
{
  if (is_initial) {
        request_chain.clear();
        cycle_handled = false;
        initial_requester = ai;
    }

  // compare two nodes
  auto compare = [&](Node* const v, Node* const u) {
    //float d_v = pathDist(ai->id, v);
    //float d_u = pathDist(ai->id, u);
    
    // tie-break for turn actions
    /*
    if (v != ai->v_now) {
        Orientation relative_pos_v = solution.getRelativePosition(ai->v_now, v);
        int angle_diff_v = solution.getAngleDifference(ai->ott_now.value(), relative_pos_v);
        if (angle_diff_v == 90) {
            d_v += 0.1;
        } else if (angle_diff_v == 180) {
            d_v += 0.2;
        }
    }
    
    if (u != ai->v_now) {
        Orientation relative_pos_u = solution.getRelativePosition(ai->v_now, u);
        int angle_diff_u = solution.getAngleDifference(ai->ott_now.value(), relative_pos_u);
        if (angle_diff_u == 90) {
            d_u += 0.1;
        } else if (angle_diff_u == 180) {
            d_u += 0.2;
        }
    }
    */
    
    float d_v;
    if (v == ai->v_now) {
        d_v = getMinDistToGoal(ai->id, v, ai->ott_now.value()) + 1;
    } else {
        Orientation target_dir_v = solution.getRelativePosition(ai->v_now, v);
        d_v = getMinDistToGoal(ai->id, v, target_dir_v);
    }

    float d_u;
    if (u == ai->v_now) {
        d_u = getMinDistToGoal(ai->id, u, ai->ott_now.value()) + 1;
    } else {
        Orientation target_dir_u = solution.getRelativePosition(ai->v_now, u);
        d_u = getMinDistToGoal(ai->id, u, target_dir_u);
    }
    
    if (v != ai->v_now) {
        Orientation target_dir_v = solution.getRelativePosition(ai->v_now, v);
        int angle_diff_v = solution.getAngleDifference(ai->ott_now.value(), target_dir_v);
        if (angle_diff_v == 0) d_v += 1;
        else if (angle_diff_v == 90) d_v += 2;
        else if (angle_diff_v == 180) d_v += 3;
    }
    
    if (u != ai->v_now) {
        Orientation target_dir_u = solution.getRelativePosition(ai->v_now, u);
        int angle_diff_u = solution.getAngleDifference(ai->ott_now.value(), target_dir_u);
        if (angle_diff_u == 0) d_u += 1;
        else if (angle_diff_u == 90) d_u += 2;
        else if (angle_diff_u == 180) d_u += 3;
    }

    if (d_v != d_u) return d_v < d_u;

    // tie break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    return false;
  };

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);


  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr) {
      //std::cout << " - vertex occupied by Agent " 
                  //<< occupied_next[u->id]->id << std::endl;
                  continue;
    }
    if (aj != nullptr && u == aj->v_now) {
        //std::cout << " - blocked by agent " << aj->id << std::endl;
        continue;
    }

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // check if cycle occurs
    if (!is_initial && u == initial_requester->v_now) {
        std::cout << "Cycle detected: Agent " << ai->id 
                    << " requests node occupied by initial requester " 
                    << initial_requester->id << std::endl;

    // [Debug] available orientation or not
        if (!ai->ott_now.has_value()) {
            std::cerr << "Error: Agent " << ai->id << " has no valid orientation during cycle detection" << std::endl;
            exit(1);
        }

        request_chain.push_back({ai, u});
        handleCycleWithOrientation();
        cycle_handled = true;
        return true;
    }

    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == nullptr) {
      request_chain.push_back({ai, u});
      if (!funcPIBT(ak, ai, false)) {
        request_chain.pop_back();
        occupied_next[u->id] = nullptr;
        ai->v_next = nullptr;
        continue;
      }  // replanning
    }

    // if action has been determined when handling cycle, further planning is unnecessary
    if (cycle_handled) {
        return true;
    }

    // compute the first to move to next vertex u
    auto [next_node, next_orientation] = solution.computeAction(
        ai->v_now,    
        u,           
        ai->ott_now.value()  
    );

    /*
    std::cout << "Agent " << ai->id 
          << " at (" << ai->v_now->pos.x << "," << ai->v_now->pos.y 
          << ") facing " << orientationToString(ai->ott_now.value())
          << " considering move to (" << u->pos.x << "," << u->pos.y 
          << ") computed next: (" 
          << next_node->pos.x << "," << next_node->pos.y 
          << ") facing " << orientationToString(next_orientation) << std::endl;
    */
      
    // if agent cannot move to a new vertex, change its orientation
    if (next_node == ai->v_now) {
        // reset the vertex occupied and change orientation when needed
        ai->v_next = ai->v_now;
        occupied_next[u->id] = nullptr;
        occupied_next[ai->v_next->id] = ai;
        ai->ott_next = next_orientation;
    }
    else {
        // if agent can moving forward then do so
        ai->v_next = next_node;
        ai->ott_next = next_orientation;
        occupied_next[ai->v_next->id] = ai;
    }

    auto al = occupied_now[u->id];
    if (al != nullptr && al->v_next == al->v_now) {
        // other agent must stay because it will adjust orientation, current agent must also stay
        if(next_node!=ai->v_now){ //if current agent wants to moving forward
        occupied_next[ai->v_now->id] = ai;
        ai->v_next = ai->v_now; // reserve current vertex
        ai->ott_next = ai->ott_now; 
        }
    }

    return true;
  }


  // failed to secure node
  //std::cout << "invalid" << aj->id << std::endl;
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  ai->ott_next = ai->ott_now;
  return false;
}

float PIBT::getMinDistToGoal(int agent_id, Node* node, Orientation current_dir) {
    float min_dist = std::numeric_limits<float>::max();
    for (Orientation goal_dir : {Orientation::X_PLUS, Orientation::X_MINUS, 
                                Orientation::Y_PLUS, Orientation::Y_MINUS}) {
        float dist = pathDist(agent_id, node, current_dir);
        min_dist = std::min(min_dist, dist);
    }
    return min_dist;
}

// check whether all agents in cycle is heading to their requesting node
// if no, adjust the orientation; if yes, moving forward
void PIBT::handleCycleWithOrientation() {
    //std::cout << "Cycle detected at timestep " << solution.getMakespan() << std::endl;
    
    if (request_chain.empty()) {
        std::cout << "[Error] Empty request chain" << std::endl;
        return;
    }
    
    bool all_oriented_correctly = true;
    std::vector<bool> correct_orientations(request_chain.size());
    
    // check the orientation of all agents in the cycle
    for (size_t i = 0; i < request_chain.size(); ++i) {
        Agent* current_agent = request_chain[i].agent;
        Node* requested_node = request_chain[i].requested_node;
        
        if (!current_agent || !requested_node || !current_agent->v_now) {
            std::cout << "[Error] Invalid vertex in request chain" << std::endl;
            continue;
        }

        Orientation target_orientation = solution.getRelativePosition(
            current_agent->v_now, requested_node);  
              
        if (!current_agent->ott_now.has_value()) {
            std::cout << "[Error] Agent " << current_agent->id << " has no orientation" << std::endl;
            all_oriented_correctly = false;
            correct_orientations[i] = false;
            continue;
        }
        
        correct_orientations[i] = (current_agent->ott_now.value() == target_orientation);
        if (!correct_orientations[i]) {
            all_oriented_correctly = false;
        }
    }

    if (!all_oriented_correctly) {
        // adjust the orientation if needed
        for (size_t i = 0; i < request_chain.size(); ++i) {
            Agent* current_agent = request_chain[i].agent;
            Node* requested_node = request_chain[i].requested_node;

            if (!current_agent->ott_now.has_value()) {
            std::cout << "[Error] Agent " << current_agent->id 
                      << " has no orientation in cycle handling!" << std::endl;
            }
            
            if (!correct_orientations[i]) {
                if (!current_agent->ott_now.has_value()) {
                    current_agent->ott_now = Orientation::Y_MINUS;
                }

                auto [next_node, next_orientation] = solution.computeAction(
                    current_agent->v_now,
                    requested_node,
                    current_agent->ott_now.value()
                );
                
                current_agent->v_next = current_agent->v_now;
                current_agent->ott_next = next_orientation;
                occupied_next[current_agent->v_now->id] = current_agent;
            } else {
                // hold current vertex and orientation
                current_agent->v_next = current_agent->v_now;
                current_agent->ott_next = current_agent->ott_now;
                occupied_next[current_agent->v_now->id] = current_agent;
            }
        }
    } else {
        // all agents in cycle are facing to their desired node, then moving forward
        for (size_t i = 0; i < request_chain.size(); ++i) {
            Agent* current_agent = request_chain[i].agent;
            Node* requested_node = request_chain[i].requested_node;
            
            current_agent->v_next = requested_node;
            current_agent->ott_next = current_agent->ott_now;
            occupied_next[requested_node->id] = current_agent;
        }
    }
}

void PIBT::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      default:
        break;
    }
  }
}

void PIBT::printHelp()
{
  std::cout << PIBT::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}
