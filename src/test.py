from roblib import *


rho = 6371000
gamma_inf = pi/4
delta_rmax = 1
r,zeta,δr_max,beta = 10, pi/4, 1, pi/4


def deg2rad(x):
	return x*2*pi/360

def wind_direction2psi(wind_direction):
	"""
	input : wind direction in degres, 0° in pointing north
			clockwise rotation
	return : angle of the wind in rad, in trigonometric circle
	"""
	wind_direction = deg2rad(wind_direction)
	psi = sawtooth(pi/2 - wind_direction)
	return psi



def control2(lxa, lya, lxb, lyb, lxm, lym):
    a = gps2geographic(lxa, lya)
    b = gps2geographic(lxb, lyb)
    m = gps2geographic(lxm, lym)
    n = np.cross(a,b)/(norm(a)*norm(b))
    e = m@n.T
    theta = 0
    q = 1
    if abs(e) > r/2 :
    	q = sign(e)

    M = array([ [-sin(lxm), cos(lxm), 0],
                [-cos(lxm)*sin(lym), -sin(lxm)*sin(lym), cos(lym)] ])

    P = M@(b-a).T
    phi = arctan2(P[1,0], P[0,0])
    theta_bar = phi - (2*gamma_inf/pi)*arctan(e/r)

    if (cos(psi-theta_bar) + cos(zeta) < 0) or ( (abs(e)<r) and (cos(psi-phi) + cos(zeta) < 0)):
        theta_bar = pi + psi - q*zeta

    if cos(theta-theta_bar) >= 0 :
        delta_r = delta_rmax*sin(theta-theta_bar)
    else :
        delta_r = delta_rmax*sign(sin(theta-theta_bar))

    delta_smax = (pi/2)*( (cos(psi-theta_bar)+1)/2 )**(log(pi/2*beta)/log(2))
    print(psi)

    return delta_r, delta_smax

def gps2geographic(lx, ly): #lx : longitude, ly : latitude
    return array([[rho*cos(ly)*cos(lx), rho*cos(ly)*sin(lx), rho*sin(ly)]])
    # return array([   [rho*cos(ly)*cos(lx)],
    # 			     [rho*cos(ly)*sin(lx)],
    # 			     [rho*sin(ly)]         ])

if __name__ == "__main__" :
	psi = wind_direction2psi(60)
	delta_r, delta_smax = control2(1,2,3,4,5,6)
	print(delta_r, delta_smax)
	# psi = wind_direction2psi(271)
	# print(psi)