import math

import rospy


class Metaheuristic:
    def __init__(self):
        self.simulation_time = 0
        self.concentration_curr = 0
        self.concentration_prev = 0
        
        # Define some epsilon based on the sensor configuration
        # Random value. Change later
        self._concentration_epsilon = 1e-4
        
        # Define some lamba. Based on what?
        # Random value. Change later
        self._probability_threshold = 1e-4

        # Start with some initial guess. How?


    def callback(self, concentration_reading):
        self.concentration_curr = concentration_reading.data

        # Substract the current sensor reading from the previous one
        concentration_change = self.concentration_curr - self.concentration_prev

        if concentration_change >= self._concentration_epsilon:
            # If the concentration is higher than or equal to epsilon, continue in the same direction
            self.keepDirection()
        else:
            # else, find the probability that the current direction is right
            maintain_direction_prob = math.exp((concentration_change - self._concentration_epsilon)/self.simulation_time)
            
            if maintain_direction_prob > self._probability_threshold:
                self.keepDirection()
            else:
                self.getNewHeuristic()
                pass


    def isSource(self):
        # If the current cell is the source, stop
        return True

    def getNewHeuristic(self):
        pass

    def keepDirection(self):
        if self.isSource():
            # stop. source located
            pass
        else:
            # keep following the current direction
            # and update simulation_time
            pass
        # Store the current sensor reading as the previous sensor reading


if __name__ == "__main__":
    rospy.init_node("heuristic")
    mh = Metaheuristic()
    rospy.Subscriber("abc", abc, queue_size=1, callback=mh.callback)

    rospy.spin()
