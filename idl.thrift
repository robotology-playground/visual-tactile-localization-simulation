service VIS_TAC_IDL
{
    string localize();
    string reset_filter();
    string set_min_allowed_z(1:double minZ);
    string get_min_allowed_z();
    string get_approach_position(1:string whereToApproach);
    string move_hand_upward(1:string armToMove);
    string home(1:string armToPutHome);
    string approach(1:string armToUse, 2:string whereToApproach);
    string fingers_approach(1:string armToUse);
    string fingers_restore(1:string armToUse);
    string enable_contacts_probe(1:string armToUse);
    string pull(1:string armToUse);
    string rotate(1:string armToUse);
    string stop();
    string quit();
}