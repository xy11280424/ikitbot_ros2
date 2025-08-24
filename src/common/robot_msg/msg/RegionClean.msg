#       failure_reason
         #   "bcd fial"
        #   "no path"


#       env_type
         #   0      未设置定
         #    1      大堂环境
         #    2      走道环境
         #    3      超市环境
         #    4      办公室环境

#       segmentation_algorithm
         #   0      未设置定 
        #   1    MorphologicalSegmentation
        #   2    DistanceSegmentation
        #   3    VoronoiSegmentation     
        #   6    自研算法             default
        #   99    PassThrough Segmentation  不做自动分区


int32 region_id
float32[] clean_region
uint8                      mode   # 0 扫吸 1 尘推 2 洗地
float32                   line_speed  #zigzag /follow edge line speed
uint8                      zigzag_space  # 只能是偶数4,6,8,10 个栅格
float32[]               start_position #  0=x,   1=y , 2=w
uint8                      env_type  
uint8                      path_type    # 0  zigzag  ,1 回字形
uint8                      segmentation_algorithm  

bool          result
string        failure_reason
string coverage_path   # 弓子/回字形路径
string boundary_path  # 沿边路径
float32                          clean_area
float32                          time_require


