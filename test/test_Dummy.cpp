#include <boost/test/unit_test.hpp>
#include <robot_frames/Dummy.hpp>

using namespace robot_frames;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    robot_frames::DummyClass dummy;
    dummy.welcome();
}
