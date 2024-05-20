# ROS2 bag recorder
# --max-bag-size BYTES --> generate multiple bag files (chucks) of max BYTES

# ASTRA COLOR AND DEPTH
#ros2 bag record --max-bag-size 1000000000 --regex "(.*)(depth|color)(.*)camera_info|tf|tf_static|imu|range_image|intensity_image|ambient_image|(.*)extrinsic(.*)depth_to_color|(.*)(color|depth)(.*)image_raw"

# ASTRA ONLY COLOR
#ros2 bag record --max-bag-size 1000000000 --regex "(.*)(color)(.*)camera_info|tf|tf_static|(-*)fix|(-*)vel|(.*)heading|(.*)imu|(.*)range_image|(.*)intensity_image|ambient_image|(.*)(color)(.*)image_raw"

# Methane
ros2 bag record --max-bag-size 1000000000 --regex "tf|tf_static|(-*)fix|(-*)reading"