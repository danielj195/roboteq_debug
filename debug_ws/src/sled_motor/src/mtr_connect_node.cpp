#include "sled_motor/mtr_connect.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mtr_connect_node");
    std::string port = "/dev/ttyACM0";
    Mtr_Connect cnt(port);
    cnt.make_connection();
    return 0;
}