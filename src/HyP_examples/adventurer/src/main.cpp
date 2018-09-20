#include <despot/planner.h>
#include "adventurer.h"

using namespace despot;

class TUI: public Planner {
public:
	TUI() {
	}

  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = NULL;

    int num_goals = options[E_SIZE] ? atoi(options[E_SIZE].arg) : 50;
     model = !options[E_PARAMS_FILE] ?
       new Adventurer(num_goals) : new Adventurer(options[E_PARAMS_FILE].arg);
    return model;
  }

  void InitializeDefaultParameters() {
     Globals::config.search_depth = 5;
     Globals::config.sim_len = 5;
  }
  
  World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options){
    return InitializePOMDPWorld(world_type, model, options);
  }

  std::string ChooseSolver(){ // Specify the solver used in the planner to be DESPOT
    return "DESPOT";
  }
};

int main(int argc, char* argv[]) {
	return TUI().RunPlanning(argc, argv);
}

