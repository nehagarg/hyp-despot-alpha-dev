#include "cupDisplay.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "graspcup_display");
	ros::NodeHandle n;
        std::istringstream iss( argv[1] );
        int val = 0;

        if (iss >> val)
        {
            std::cout << "Start state is : " << val << std::endl;
        }
        else{
            std::cout << "Start state is : " << val << std::endl;
        }
	CUPDISPLAY cup_display = CUPDISPLAY(n, val);
	cup_display.DrawRviz();

	return 0;
}