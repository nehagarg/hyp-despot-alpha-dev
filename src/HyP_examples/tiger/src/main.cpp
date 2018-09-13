#include <despot/planner.h>
#include "tiger.h"
#include "tiger2actions.h"

using namespace despot;

class TUI: public Planner {
public:
  TUI() {
  }
 
  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model;
    model = !options[E_PARAMS_FILE] ?
       new  Tiger2actions(): new Tiger();
    return model;
  }
  
  void InitializeDefaultParameters() {
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
