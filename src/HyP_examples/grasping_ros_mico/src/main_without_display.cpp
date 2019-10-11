#include "grasping_ros_tui.h"
#include "grasping_real_arm.h"
#include "ros/ros.h"
#include "log_file_reader.h"
//#include "inference_cc.h"
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


/*int main(int argc, char* argv[])
{
	tensorflow::Session* sess =   start_session(argc);
	for(int i = 0; i < 10; i++)
	{
		double step_start = despot::get_time_second();
	double start_2 = inference_keras_main(sess);
	double step_end = despot::get_time_second();
	std::cout << "Inference " << i << "took " << step_end-step_start << ", " << step_end-start_2 << "s" << endl;

	}




}*/


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
