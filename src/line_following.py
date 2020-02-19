#!/usr/bin/env python

import rospy
import numpy as np
import utm

from numpy import cos, sin, arctan, arctan2, pi, cross, hstack, array, log, sign
from numpy.linalg import det, norm

from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from controller.msg import Command
from controller.msg import Waypoints
from controller.msg import Gps
from controller.msg import Compass
from controller.msg import Meteo
from controller.msg import Wind

import math

r,zeta = 10, pi/4
rho = 6371000
delta_rmax = pi/3
EARTH_RADIUS = 6371000.
lat0, lon0 = (48.198427, -3.014750)


class Controller():


	def __init__(self,rosrate=10):
		self.pub_pwm = rospy.Publisher('/Command', Command, queue_size=10)

		rospy.Subscriber('/ublox/GPRMC', Gps, self._callback_gps)
		rospy.Subscriber('/ublox/HCHDG', Compass, self._callback_compass)
		rospy.Subscriber('/ublox/WIMDA', Meteo, self._callback_meteo)
		rospy.Subscriber('/ublox/WIMWV', Wind, self._callback_wind)
	   
		# rospy.Subscriber('/Waypoints', Waypoints, self._callback_waypoints)

		self.rate = rospy.Rate(rosrate)

		self.pwm_min_rudder = 900
		self.pwm_min_main_sail = 900
		self.pwm_min_fore_sail = 920

		self.pwm_mid_rudder = 1250 
		self.pwm_mid_main_sail = 1180
		self.pwm_mid_fore_sail = 1460    

		self.pwm_max_rudder = 1600
		self.pwm_max_main_sail = 1460
		self.pwm_max_fore_sail = 1710 

		self.q = 0

		self.lxa, self.lya = -3.015894, 48.197031
		self.lxb, self.lyb = -3.016162, 48.202015
		self.lxm, self.lym = 0., 0.
		self.heading = 0.
		self.psi = 0.
		self.lat_str = ''
		self.lon_str = ''



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
		if msg.latitude_indic == "N":
			lat  = 1
		else: 
			lat = -1
		if msg.longitude_indic == "W":
			lon = -1
		else:
			lon = 1

		self.lat_str = str(msg.latitude)
		lat_str = self.lat_str.split(".")
		self.lon_str = str(msg.longitude)
		lon_str = self.lon_str.split(".")

		#conv degres.minute _> degres.decimal
	
		self.lym = lat*(float(lat_str[0][0:len(lat_str[0])-2]) + float(lat_str[0][len(lat_str[0])-2:])/60. + float(lat_str[1])/10**len(lat_str[1])/60.)
		self.lxm = lon*(float(lon_str[0][0:len(lon_str[0])-2]) + float(lon_str[0][len(lon_str[0])-2:])/60. + float(lon_str[1])/10**len(lon_str[1])/60.)

	def _callback_compass(self, msg):
		"""
		float64 heading
		string heading_indic
		float64 magnetic_declination
		"""

		self.heading = self.north2east( msg.heading )  


	def _callback_meteo(self, msg):
		"""
		float64 barometric_pressure_mercury
		float64 barometric_pressure_bars
		float64 temperature
		float64 true_wind_direction #angle of the wind en degre par rapport au nord dans le sens horaire
		float64 magnetic_wind_direction0.
		float64 wind_speed
		"""

		self.psi = self.north2east( msg.true_wind_direction )  


	def _callback_wind(self, msg):
		"""
		float64 wind_direction
		string reference
		float64 wind_speed
		string wind_speed_units
		string status
		"""
		#self.psi = self.north2east( msg.wind_direction ) 


	# def _callback_waypoints(self, msg):

	#     self.lxa, self.lya = msg.lxa, msg.lya
	#     self.lxb, self.lyb = msg.lxb, msg.lyb
		  

	def control(self):
		ke=0.5
		# a = self.gps2cart(self.lxa, self.lya)
		# b = self.gps2cart(self.lxb, self.lyb)
		# m = self.gps2cart(self.lxm, self.lym)
		xa, ya = self.WGS84_to_cart(self.lya, self.lxa)
		xb, yb = self.WGS84_to_cart(self.lyb, self.lxb)
		xm, ym = self.WGS84_to_cart(self.lym, self.lxm)

		a = array([[xa],[ya]])
		b = array([[xb],[yb]])
		m = array([[xm],[ym]])

		# print("m:",m)


		
		theta = self.heading #cap robot
		# print("theta", math.degrees(theta)+90)
		"""
		#print("theta",theta)
		e = det(hstack((b-a,m-a)))/norm(b-a)
		
		if abs(e) > r:
			self.q = sign(e)

		phi = arctan2(b[1,0]-a[1,0],b[0,0]-a[0,0])
		#print("e",e)
		theta_bar = phi-ke*arctan(e/r)
		#print("thetabar", theta_bar)
		print("psi",self.psi)
		#print("theta_bar", math.degrees(theta_bar)+90)
		#print("psi: ", math.degrees(self.psi)+90)
		#if (cos(self.psi-theta_bar) + cos(zeta) < 0) or ( (abs(e)<r) and (cos(self.psi-phi) + cos(zeta) < 0)):
		#	theta_bar = pi + self.psi - self.q*zeta

		if cos(theta-theta_bar) >= 0 :
			delta_r = delta_rmax*sin(theta-theta_bar)
		else :
			delta_r = delta_rmax*sign(sin(theta-theta_bar))

		# delta_r = (delta_rmax/pi)*self.sawtooth(theta-theta_bar)
		delta_smax = (pi/2)*( (cos(self.psi-theta_bar)+1)/2 )

		return  delta_r, delta_smax
		"""

		theta_bar = arctan2(b[1,0]-a[1,0],b[0,0]-a[0,0])
		cap = self.sawtooth(theta-theta_bar)
		delta_r = -(delta_rmax/pi)*cap

		return delta_r, pi/2




	def control2(self):
		a = self.gps2geographic2(self.lxa, self.lya)
		b = self.gps2geographic2(self.lxb, self.lyb)
		m = self.gps2geographic2(self.lxm, self.lym)
		n = cross(a,b)/(norm(a)*norm(b))
		e = np.dot(m,n.T)
		#print(a)
		#print(b)
		print("e: ", e)
		theta = self.heading
		print("theta", theta)
		if abs(e) > r/2 :
			self.q = sign(e)

		M = array([ [-sin(self.lxm), cos(self.lxm), 0],
					[-cos(self.lxm)*sin(self.lym), -sin(self.lxm)*sin(self.lym), cos(self.lym)] ])
		P = np.dot(M,(b-a).T)
		phi = arctan2(P[1,0], P[0,0])
		#print("phi", phi)
		theta_bar = phi - arctan(e/r)
		#print("theta_bar", theta_bar)

		#if (cos(self.psi-theta_bar) + cos(zeta) < 0) or ( (abs(e)<r) and (cos(self.psi-phi) + cos(zeta) < 0)):
		#	theta_bar = pi + self.psi - self.q*zeta

		# if cos(theta-theta_bar) >= 0 :
		# 	delta_r = delta_rmax*sin(theta-theta_bar)
		# else :
		# 	delta_r = delta_rmax*sign(sin(theta-theta_bar))

		delta_r = (delta_rmax/pi)*self.sawtooth(theta-theta_bar)
		delta_smax = (pi/2)*( (cos(self.psi-theta_bar)+1)/2 )
	
		#print("delta_r", delta_r)
		#print("delta_smax", delta_smax)    

		return delta_r, delta_smax


	def gps2geographic(self, lx, ly): #lx : longitude, ly : latitude

		(EASTING, NORTHING, ZONE_NUMBER, ZONE_LETTER) = utm.from_latlon(ly, lx)
		return array([[EASTING], [NORTHING]])


	def gps2geographic2(self, lx, ly): #lx : longitude, ly : latitude

		return array([[rho*cos(ly)*cos(lx), rho*cos(ly)*sin(lx), rho*sin(ly)]]) 

	def gps2cart(self, lx, ly):
		x = rho*cos(ly)*cos(lx)
		y = rho*cos(ly)*sin(lx)
		return array([[x], [y]])
		
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
		return (x+pi)%(2*pi)-pi   


	def deg2rad(self, x):

		return x*2*pi/360

	def north2east(self, x):
		"""
		Input:
		------
		Wind direction in degres, 0 is pointing north
		clockwise rotation

		Return:
		-------
		Angle of the wind in rad, in trigonometric circle
		"""

		x = self.deg2rad(x)
		return self.sawtooth(pi/2 - x)

	def WGS84_to_cart(self, lat, lon):
		x = (pi/180.)*EARTH_RADIUS*(lon-lon0)*cos((pi/180.)*lat)
		y = (pi/180.)*EARTH_RADIUS*(lat-lat0)
		return x, y

	def cart_to_WGS84(self, x, y):
		EPSILON=0.00000000001
		lat = y*180./pi/EARTH_RADIUS+lat0
		if abs(lat-90.) < EPSILON or abs(lat+90.) < EPSILON:
			lon = 0
		else:
			lon = (x/EARTH_RADIUS)*(180./pi)/cos((pi/180.)*(lat))+lon0
		return lat, lon



	def rad2pwm(self, x, sail_name):
		"""
		Bijection de [0, pi/2] sur [pwm_min, pwm_max] pour les voiles.
	[-pi/3,pi/3] pour rudder
		"""
		if sail_name == "main":
			return (2/pi)*(self.pwm_max_main_sail - self.pwm_min_main_sail)*x + self.pwm_min_main_sail
		elif sail_name == "fore":
			return (2/pi)*(self.pwm_max_fore_sail - self.pwm_min_fore_sail)*x + self.pwm_min_fore_sail    
		elif sail_name == "rudder":
			x = x+pi/3
			return (3/(2*pi))*(self.pwm_max_rudder - self.pwm_min_rudder)*x + self.pwm_min_rudder



	def main(self):


		u1, u2 = self.control() 
		delta_rudder, delta_main_sail, delta_fore_sail = u1, u2, u2

		pwm_rudder = self.rad2pwm(delta_rudder, "rudder")
		pwm_main_sail = self.rad2pwm(delta_main_sail, "main")
		pwm_fore_sail = self.rad2pwm(delta_fore_sail, "fore")

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
