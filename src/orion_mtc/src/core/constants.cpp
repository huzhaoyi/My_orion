#include "orion_mtc/core/constants.hpp"

namespace orion_mtc
{

const std::vector<std::string> OBJECT_GRASP_ALLOWED_LINKS = { "Link6", "Link7", "Link8" };

const std::vector<std::string> RETURN_HOME_OBJECT_ALLOWED_LINKS = { "Link5", "Link6", "Link7", "Link8" };

const std::vector<std::string> ARM_JOINTS = {
    "joint_base_link_Link1", "joint_Link1_Link2", "joint_Link2_Link3",
    "joint_LinkVirtual_Link4", "joint_Link4_Link5", "joint_Link5_Link6"
};
const std::vector<std::string> HAND_JOINTS = { "joint_Link6_Link7", "joint_Link6_Link8" };

}  // namespace orion_mtc
