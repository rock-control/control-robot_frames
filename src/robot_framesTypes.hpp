#ifndef robot_frames_TYPES_HPP
#define robot_frames_TYPES_HPP

#include <string>

namespace robot_frames {
    struct Chain{
        Chain(){
            name = root_link = root_link_renamed = tip_link = tip_link_renamed = "";
        }

        std::string name;
        std::string root_link;
        std::string root_link_renamed;
        std::string tip_link;
        std::string tip_link_renamed;
    };
}

#endif

