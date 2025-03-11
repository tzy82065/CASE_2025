#pragma once
#include "problem.hpp"
#include "orientation.hpp"
#include "iostream"


/*
 * array of configurations
 */

struct Plan {
private:
  Configs configs;  // main
  std::vector<std::vector<Orientation>> orientations;  
  Orientation rotateCounterClockwise(Orientation orient) const;

public:
  ~Plan() {}

  // timestep -> configuration
  Config get(const int t) const;

  // timestep, agent -> location
  Node* get(const int t, const int i) const;

  // consider orientation
  void addOrientation(const std::vector<Orientation>& orients);
  Orientation getOrientation(const int t, const int i) const;
  std::vector<Orientation> getOrientations(const int t) const;
  int getAngleDifference(Orientation dir1, Orientation dir2) const;
  Orientation getRelativePosition(Node* current, Node* target) const;
  void clearOrientations();

  // add config with orientation to solution
  void addWithOrientation(const Config& c, const std::vector<Orientation>& orients);
  
  // compute the first action based on the requested node
  std::pair<Node*, Orientation> computeAction(
      Node* current, Node* target, Orientation current_orient) const;

  // path
  Path getPath(const int i) const;

  // path cost
  int getPathCost(const int i) const;

  // last configuration
  Config last() const;

  // last configuration
  Node* last(const int i) const;

  // become empty
  void clear();

  // add new configuration to the last
  void add(const Config& c);

  // whether configs are empty
  bool empty() const;

  // configs.size
  int size() const;

  // size - 1
  int getMakespan() const;

  // sum of cost
  int getSOC() const;

  // join with other plan
  Plan operator+(const Plan& other) const;
  void operator+=(const Plan& other);

  // check the plan is valid or not
  bool validate(MAPF_Instance* P) const;
  bool validate(MAPD_Instance* P) const;
  bool validate(const Config& starts, const Config& goals) const;
  bool validate(const Config& starts) const;

  // when updating a single path,
  // the path should be longer than this value to avoid conflicts
  int getMaxConstraintTime(const int id, Node* s, Node* g, Graph* G) const;
  int getMaxConstraintTime(const int id, MAPF_Instance* P) const;

  // error
  void halt(const std::string& msg) const;
  void warn(const std::string& msg) const;
};

using Plans = std::vector<Plan>;
