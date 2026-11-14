/* Minimal example: no glove needed; checks that the SDK links and can be called.
 *   Prints the version and the model fingerprint, runs left-hand FK with all 21 joint angles set to 0 (PTV1 mapping),
 *   and prints the position of the last link in the joint table.
 * Exit code 0 = all calls succeeded. */
#include <stdio.h>
#include <string.h>
#include <revohuman_fk.h>

int main(void)
{
    printf("revohuman %s\n", rh_version());
    printf("hand model sha256 %s\n", rh_hand_model_sha256());

    double deg[RH_JOINT_COUNT];
    double q[RH_JOINT_COUNT];
    memset(deg, 0, sizeof deg);
    if (rh_joints_to_radians("PTV1", deg, q) != RH_OK) {
        fprintf(stderr, "rh_joints_to_radians failed\n");
        return 1;
    }

    RhLinkPose links[RH_JOINT_COUNT];
    if (rh_hand_fk(RH_HAND_LEFT, q, NULL, links) != RH_OK) {
        char err[256];
        rh_last_error(err, sizeof err);
        fprintf(stderr, "rh_hand_fk failed: %s\n", err);
        return 1;
    }

    const uint32_t last = RH_JOINT_COUNT - 1;
    char name[64];
    if (rh_hand_link_name(RH_HAND_LEFT, last, name, sizeof name) < 0) {
        fprintf(stderr, "rh_hand_link_name failed\n");
        return 1;
    }
    printf("%s at (%.6f, %.6f, %.6f) m in palm frame\n",
           name, links[last].p[0], links[last].p[1], links[last].p[2]);
    return 0;
}
