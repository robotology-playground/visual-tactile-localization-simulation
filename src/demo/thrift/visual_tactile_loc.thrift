service VIS_TAC_IDL
{
    string init();
    string start_visual_localization();
    string visuo_tactile_matching();
    string stop_localization();
    string reset_filter();
    string set_min_allowed_z(1:double minZ);
    string get_min_allowed_z();
    string get_approach_position(1:string whereToApproach);
    string move_hand_upward(1:string armToMove);
    string move_arm_rest_pose(1:string armToMove);
    string home_arm(1:string armToPutHome);
    string home_head();
    string approach(1:string armToUse, 2:string whereToApproach);
    string fingers_approach(1:string armToUse);
    string fingers_restore(1:string armToUse);
    string enable_contacts_probe(1:string armToUse);
    string disable_contacts_probe(1:string armToUse);
    string pull(1:string armToUse);
    string rotate(1:string armToUse);
    string stop();
    string quit();
}