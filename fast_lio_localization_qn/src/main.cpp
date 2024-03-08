#include "main.h"
#include <csignal>

void SigHandle(int sig)
{
    std::cout << "You pressed Ctrl + C, exiting" << std::endl;
    exit(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio_localization_sc_qn_node");
  ros::NodeHandle nh_private("~");

  FastLioLocalizationScQnClass FastLioLocalizationScQn_(nh_private);

  signal(SIGINT, SigHandle);

  ros::AsyncSpinner spinner(3); // Use multi threads
  spinner.start();
  ros::waitForShutdown();
 
  return 0;
}