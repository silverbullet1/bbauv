#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace nodelet_test
{
    class Test : public nodelet::Nodelet
    {
        public:
            Test(){}
            ~Test(){}
        private:
            virtual void onInit();
    };

    PLUGINLIB_DECLARE_CLASS(nodelet_test, Test, nodelet_test::Test,
                            nodelet::Nodelet);
}
