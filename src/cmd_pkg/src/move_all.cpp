//
// Created by wang on 12/7/19.
//

#include "move_all_base.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_all");

    ros::AsyncSpinner spin(2);//multiple thread , for multiple callback function
    spin.start();

    bool is_sim = false;
    MOVE_ALL move_all;
    move_all.sim = is_sim;

    ros::waitForShutdown();
    return 0;
}