#include <despot/planner.h>
#include "tag.h"

using namespace despot;

class TUI: public Planner {
public:
  TUI() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = !options[E_PARAMS_FILE] ?
      new Tag() : new Tag(options[E_PARAMS_FILE].arg);
    return model;
  }

  void InitializeDefaultParameters() {
    Globals::config.pruning_constant = 0.01;
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
