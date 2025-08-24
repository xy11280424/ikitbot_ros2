# msg definition

# feedback enum
int32 NONE              = 0 # none
                            # others...

int32                 feedback 	       # result feedback
string 		          feedback_str     # feedback to string
geometry_msgs/Vector3 path_position    # 當前路徑點位置
int32                 path_id 
