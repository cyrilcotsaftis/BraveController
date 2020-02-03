#!/usr/bin/env python

import rospy
import numpy as np
import utm
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from controller.msg import Command
from controller.msg import Waypoints
from ublox.msg import Gps
from ublox.msg import Compass
from ublox.msg import Meteo
from ublox.msg import Wind

from roblib import * 


pi = np.pi
r,zeta,δr_max,beta = 10, pi/4, 1, pi/4
rho = 6371000
gamma_inf = pi/4
delta_rmax = 1


class Controller():


    def __init__(self, pwm_max=1500, pwm_neutral=1500,rosrate=4):
        self.pub_pwm = rospy.Publisher('/Command', Command, queue_size=10)

        rospy.Subscriber('/ublox_GPRMC', Gps, self._callback_gps)
        rospy.Subscriber('/ublox/HCHDG', Compass, self._callback_compass)
        rospy.Subscriber('/ublox/WIMDA', Meteo, self._callback_meteo)
        rospy.Subscriber('/ublox/WIMWV', Wind, self._callback_wind)
       
        rospy.Subscriber('/Waypoints', Waypoints, self._callback_waypoints)

        self.rate = rospy.Rate(rosrate)

        self.pwm_min_rudder = 900
        self.pwm_min_main_sail = 1181
        self.pwm_min_fore_sail = 920

        self.pwm_mid_rudder = 1250 
        self.pwm_mid_main_sail = 1460
        self.pwm_mid_fore_sail = 1710     

        self.pwm_max_rudder = 1600
        self.pwm_max_main_sail = 1460
        self.pwm_max_fore_sail = 1710 

        self.q = 0
        self.lxa, self.lya = 0., 0.
        self.lxb, self.lyb = 0., 0.
        self.lxm, self.lym = 0., 0.
        self.heading = 0.

      

    def _callback_gps(self, msg):
    	"""
    	float64 timeStamp
		string validation
		float64 latitude
		string latitude_indic
		float64 longitude
		string longitude_indic
		float64 speed
		float64 heading
		float64 date
		float64 magnetic_declination
		string declination_indic
		string positionning_mode
    	"""

    	self.lym = msg.latitude
    	self.lxm = msg.longitude
    	# self.heading = msg.heading


    def _callback_compass(self, msg):
    	"""
    	float64 heading
		string heading_indic
		float64 magnetic_declination
		"""

		self.heading = msg.heading


    def _callback_meteo(self, msg):
    	"""
    	float64 barometric_pressure_mercury
		float64 barometric_pressure_bars
		float64 temperature
		float64 true_wind_direction #angle of the wind en degre par rapport au nord dans le sens horaire
		float64 magnetic_wind_direction
		float64 wind_speed
		"""

        self.psi = self.wind_direction2psi( msg.true_wind_direction )  


    def _callback_wind(self, msg):
    	"""
		float64 wind_direction
		string reference
		float64 wind_speed
		string wind_speed_units
		string status
		"""

        # self.psi = self.wind_direction2psi( msg.wind_direction ) 


    def _callback_waypoints(self, msg):
        self.lxa, self.lya = msg.lxa, msg.lya
        self.lxb, self.lyb = msg.lxb, msg.lyb
          

    # def control(self):

    # 	(EASTING, NORTHING, ZONE NUMBER, ZONE LETTER) = utm.from_latlon(self.latitude, self.longitude)
    #     m = array([[EASTING], [NORTHING]]) #position x,y robot
    #     theta = self.heading #cap robot
    #     e = det(hstack((self.b-self.a,m-self.a)))/norm(self.b-self.a)
    #     if abs(e) > r:
    #         self.q = sign(e)
    #     phi = arctan2(self.b[1,0]-self.a[1,0],self.b[0,0]-self.a[0,0])
    #     theta_bar = phi-arctan(e/r)
    #     if (cos(self.ψ-theta_bar) + cos(zeta) < 0) or ( (abs(e)<r) and (cos(self.ψ-phi) + cos(zeta) < 0)):
    #         theta_bar = pi + self.ψ - self.q*zeta
    #     δr = (δr_max/pi)*self.sawtooth(theta-theta_bar)
    #     δsmax = (pi/2)*( (cos(self.ψ-theta_bar)+1)/2 )**(log(pi/2*beta)/log(2))
    #     return  δr, δsmax


    def control(self):

        a = self.gps2geographic(self.lxa, self.lya)
        b = self.gps2geographic(self.lxb, self.lyb)
        m = self.gps2geographic(self.lxm, self.lym)
        n = np.cross(a,b)/(norm(a)*norm(b))
        e = m@n.T
        theta = self.heading

        if abs(e) > r/2 :
            self.q = sign(e)

        M = array([ [-sin(self.lxm), cos(self.lxm), 0],
                    [-cos(self.lxm)*sin(self.lym), -sin(self.lxm)*sin(self.lym), cos(self.lym)] ])
        P = M@(b-a).T
        phi = arctan2(P[1,0], P[0,0])

        theta_bar = phi - (2*gamma_inf/pi)*arctan(e/r)

        if (cos(self.psi-theta_bar) + cos(zeta) < 0) or ( (abs(e)<r) and (cos(self.psi-phi) + cos(zeta) < 0)):
            theta_bar = pi + self.psi - self.q*zeta

        if cos(theta-theta_bar) >= 0 :
            delta_r = delta_rmax*sin(theta-theta_bar)
        else :
            delta_r = delta_rmax*sign(sin(theta-theta_bar))

        delta_smax = (pi/2)*( (cos(self.psi-theta_bar)+1)/2 )**(log(pi/2*beta)/log(2))

        return delta_r, delta_smax


    def gps2geographic(self, lx, ly): #lx : longitude, ly : latitude

        return array([[rho*cos(ly)*cos(lx), rho*cos(ly)*sin(lx), rho*sin(ly)]])
    
        
    def saturation(self, pwm, pwm_min, pwm_max):
        """Saturate command
        
        Input:
        ------
        pwm: pwm from self.control 
        
        Return:
        -------
        pwm: int, published on '/Command'
        """

        if pwm > pwm_max:
            pwm = pwm_max
        if pwm < pwm_min:
            pwm = pwm_min
        return pwm


	def sawtooth (self, x):
        """Deal with 2*PI modulo
        
        Input:
        ------
        x: rad 
        """
        return (x+PI)%(2*PI)-PI   


    def deg2rad(self, x):

        return x*2*pi/360

    def wind_direction2psi(self, wind_direction):
        """
        Input:
        ------
        Wind direction in degres, 0° is pointing north
        clockwise rotation

        Return:
        -------
        Angle of the wind in rad, in trigonometric circle
        """
        wind_direction = deg2rad(wind_direction)
        psi = sawtooth(pi/2 - wind_direction)
        return psi



    def main(self):

            
        u1, u2 = self.control() # δr, δsmax en rad
        delta_rudder, delta_main_sail, delta_fore_sail = u1, u2, u2


        #faire mapping angle/pwm servo

        pwm_rudder = self.pwm_mid_rudder - delta_rudder
        pwm_main_sail = self.pwm_mid_main_sail - delta_main_sail
        pwm_fore_sail = self.pwm_mid_fore_sail - delta_fore_sail


        pwm_rudder = self.saturation(pwm_rudder, self.pwm_min_rudder,self.pwm_max_rudder)
        pwm_main_sail = self.saturation(pwm_main_sail, self.pwm_min_main_sail, self.pwm_max_main_sail)
        pwm_fore_sail = self.saturation(pwm_fore_sail, self.pwm_min_fore_sail, self.pwm_max_fore_sail)


        pwm = Command()
        pwm.pwm_rudder = pwm_rudder
        pwm.pwm_main_sail = pwm_main_sail
        pwm.pwm_fore_sail = pwm_fore_sail

        self.pub_pwm.publish(pwm)




if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    controller = Controller()
    while not rospy.is_shutdown():
        controller.main()
        controller.rate.sleep()
