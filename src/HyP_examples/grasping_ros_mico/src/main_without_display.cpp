#include "grasping_ros_tui.h"
#include "grasping_real_arm.h"
#include "ros/ros.h"
#include "log_file_reader.h"

using namespace despot;


class RosWithoutDisplayTUI: public TUI {
public:
  RosWithoutDisplayTUI() {
  }

    DSPOMDP* InitializeModel(option::Option* options) {
        std::cout << "Initializing model" << std::endl;
        return InitializeModelCommon(options);
  }
};




int main(int argc, char* argv[]) {
    std::cout << "In main" << std::endl;
    main_init(argc, argv);
     std::cout << "In main" << std::endl;
    
  //  GatherSimulationData();
  //  return 0;
    
    //test_python();
    //return 0;
   
 return RosWithoutDisplayTUI().RunPlanning(argc, argv);
  
 // return RosTUI().run(argc, argv);
}


