

def control_surge(self):
        """PID Controller of the error in the X-direction"""
        tolerance = 0.08
        
        for x_position in self.x_list:
            error_x = self.current_pos.position.x - x_position # positive to left
            print(f"target position x: {x_position}")
            print(f"current position x: {self.current_pos.position.x}")

            while (abs(error_x) > tolerance):
                error_x = self.current_pos.position.x - x_position
                portional_error_x = error_x * self.surge_KP

                self.current_time = rospy.get_time()
                delta_t = self.current_time - self.last_time

                if delta_t:
                    derevative_error_x = (error_x - self.last_error_x) * self.surge_KD / delta_t
                    self.last_error_x = error_x
                    self.last_time = self.current_time
                else:
                    derevative_error_x = 0

                self.x_controller_pub.publish(self.saturate_controller(portional_error_x + derevative_error_x))
    
def control_sway(self):
        """PID Controller of the error in the Y-direction"""
        tolerance = 0.08
        
        for y_position in self.y_list:
            error_y = y_position - self.current_pos.position.y   # positive forward
            print(f"target position: {y_position}")
            print(f"current position: {self.current_pos.position.y}")

            while (abs(error_y) > tolerance):
                # print(self.current_pos.position.y)

                error_y = y_position - self.current_pos.position.y 
                portional_error_y = error_y * self.sway_KP

                self.current_time = rospy.get_time()
                delta_t = self.current_time - self.last_time

                if delta_t:
                    derevative_error_y = (error_y - self.last_error_y) * self.sway_KD / delta_t
                    self.last_error_y = error_y
                    self.last_time = self.current_time
                else:
                    derevative_error_y = 0

                self.y_controller_pub.publish(portional_error_y + derevative_error_y)
            
            self.y_controller_pub.publish(0)


def control_yaw(self):
        """PID Controller of the error in the Yaw-direction"""
        """ right: (z:1), left: (z:0), forward: (z:0.7), backward:(z:-0.7) """
        tolerance = 0.08
        
        for yaw_orient in self.orientations:
            error_yaw = yaw_orient - self.current_pos.orientation.z   # positive ccw
            print(f"target position: {yaw_orient}")
            print(f"current position: {self.current_pos.orientation.z}")
            self.current_target_yaw.publish(yaw_orient)
            while (abs(error_yaw) > tolerance):
                error_yaw = yaw_orient - self.current_pos.orientation.z 
                portional_error_yaw = error_yaw * self.yaw_KP

                self.current_time = rospy.get_time()
                delta_t = self.current_time - self.last_time

                if delta_t:
                    derevative_error_yaw = (error_yaw - self.last_error_yaw) * self.yaw_KD / delta_t
                    self.last_error_yaw = error_yaw
                    self.last_time = self.current_time
                else:
                    derevative_error_yaw = 0

                self.yaw_controller_pub.publish(self.saturate_controller(portional_error_yaw + derevative_error_yaw))
            
            self.yaw_controller_pub.publish(0)

            print(f"{yaw_orient} has been reached" )
            print('########################################')
            time.sleep(2)


def control_surge_sway(self):
        """PID Controller of the error in the X-Y-direction"""
        
        tolerance = 0.08
        
        for x_position, y_position in zip(self.y_list,self.x_list):

            error_x = self.current_pos.position.x - x_position # positive to left
            error_y = y_position - self.current_pos.position.y   # positive forward

            print(f"target position: {x_position}")
            print(f"current position: {self.current_pos.position.x}")

            while (abs(error_x) > tolerance) and (abs(error_y) > tolerance):
                error_x = self.current_pos.position.x - x_position
                error_y = y_position - self.current_pos.position.y   # positive forward

                portional_error_x = error_x * self.surge_KP
                portional_error_y = error_y * self.sway_KP

                self.current_time = rospy.get_time()
                delta_t = self.current_time - self.last_time

                if delta_t:
                    derevative_error_x = (error_x - self.last_error_x) * self.surge_KD / delta_t
                    derevative_error_y = (error_y - self.last_error_y) * self.sway_KD / delta_t

                    self.last_error_x = error_x
                    self.last_error_y = error_y

                    self.last_time = self.current_time
                else:
                    derevative_error_x = 0
                    derevative_error_y = 0

                self.x_controller_pub.publish(portional_error_x + derevative_error_x)
                self.y_controller_pub.publish(portional_error_y + derevative_error_y)


def follow(self,wait_time=3):

        ###############################################
        ########## Apply Controller Here ##############
        ###############################################

        # generate_waypoints = rospy.ServiceProxy('generate_waypoint_service', GenerateWaypoints) # call waypoint generation service
        # if self.ready:
        #     self.last_time = self.current_time
        #     self.current_target_point= generate_waypoints(self.ready).current_target_position
        #     print(self.current_target_point)

        #     self.current_target_pos_pub.publish(self.current_target_point) # to use in the webui

        # self.current_time = time.time()

        # error_x = self.current_pos.position.x  - self.current_target_point.position.x * self.x_scale # positive out >> move left 
        # error_y = self.current_target_point.position.y - self.current_pos.position.y * self.y_scale # positive out >> move forward 

        # self.ready = self.update_controller_state(error_x,error_y) # or (self.current_time - self.last_time >= wait_time)

        # if abs(error_x) < 0.2:
        #     error_x = 0

        # if abs(error_y) < 0.2:
        #     error_y = 0

        # self.x_controller_pub.publish(self.saturate_controller(error_x*self.KP))
        # self.y_controller_pub.publish(self.saturate_controller(error_y*self.KP))

        # print(f"error_x: {error_x}")
        # print(f"error_y: {error_y}")
        # print("=======================")
        # print(f"target_pos:{self.current_target_point.position}")
        # self.control_surge()
        # self.control_sway()
        self.control_surge_sway_yaw()
        # self.control_surge_sway()