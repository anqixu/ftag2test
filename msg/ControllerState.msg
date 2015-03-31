float64 comm_pos_x
float64 comm_pos_y
float64 comm_pos_z
float64 comm_rot_r
float64 comm_rot_p
float64 comm_rot_y

#ftag2test/DisplayTag displayer

#int32 frameID

#float64[] encoded_phases # empty == end of sequence

string command
string fsm
int32 pos_count
int32 rot_count

#int32 num_detections
#int32 num_images
#int32 num_poses

#int32 image_count
#int32 image_count_in_pos

int32 total_image_count
int32 tag_count_in_pose
int32 detection_count
int32 num_successful_detections

bool failed_image
bool failed_pose

string tag_payload
string tag_family
int32 num_tag_family
#int32 images_family_count
