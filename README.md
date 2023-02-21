# lawn_tractor_embedded_code


These two programs work with each other using the Lilygo TTGO LoRa board http://www.lilygo.cn/prod_view.aspx?TypeId=50060&Id=1135&FId=t3:50060:3
1. radio_control - provides manual control and e-stop functionality  
2. tractor_control - receives control instructions from radio control or cmd_vel and instructs the steering motor controller and transmission controller accordingly

These two programs provide speed and distance travelled data on a ROS topic by reading a AS5048 magnetic rotational sensor mounted on the rear axle assembly.
3. left_speed
4. right_speed