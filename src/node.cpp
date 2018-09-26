#include "kitti_player.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_player");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    KittiPlayer player;
    player.loop();

    return 0;
}
