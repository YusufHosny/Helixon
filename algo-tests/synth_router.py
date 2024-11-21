import random
import math
import numpy as np

# wifi data
routers_n = 10

rssi_measured_power = -40 # typical 1-meter-RSSI for WiFi routers is between -40 and -60 dBm
rssi_path_loss_exponent = 2 # typical indoor path loss exponent is between 4 and 6


# generates random MAC address
def generate_mac_address():
    return ":".join(f"{random.randint(0, 255):02x}" for _ in range(6))


# generates array of routers, each with x y z coordinates with corresponding MAC address
wifi_routers = [[-1.9625912635830327, -0.042755400999599624, 0.236777918073553, '9e:2a:ea:04:ac:93'],
                [-2.86743255185108, 10.230164478313519, 2.373233293431522, 'e2:8c:54:f4:47:9f'],
                [-2.6049383725563793, 10.992929854309349, 4.82598431890248, 'cc:41:11:fa:b6:f4'],
                [11.27152174407813, 10.361298068605526, 6.438432335099835, '2d:7a:ed:50:7f:a9'],
                [-0.9229859736350821, -0.8181204089944636, 8.208368772600491, '45:84:bd:7b:45:8f'],
                [10.304070899801548, 11.943232998360061, 10.297419378938036, 'cc:c5:88:bb:70:f8'],
                [10.548371853778425, -2.882304115246261, 12.740700625938995, 'de:be:c6:8f:32:26'],
                [12.843273134642441, 11.65724374052579, 14.151707456823424, 'c1:ef:d9:17:b4:54'],
                [12.615689578047608, 10.765756456022414, 16.13282069036388067, '1f:3c:d0:f0:ec:04'],
                [-1.3179787074473732, -1.2796112394304908, 18.53774228686071, 'ec:2a:13:a2:94:dc']]



X, Y, Z, MAC = 0, 1, 2, 3


def get_RSSI_MAC(position):

    wifi_data = []
    wifi_data.append(routers_n)

    for i in range(routers_n):
        distance = np.sqrt(
                (wifi_routers[i][X] - position[X]) ** 2 +
                (wifi_routers[i][Y] - position[Y]) ** 2 +
                (wifi_routers[i][Z] - position[Z]) ** 2
            )

        rssi = rssi_measured_power - 10 * rssi_path_loss_exponent * math.log10(distance)
        wifi_data.append(rssi)

        mac = wifi_routers[i][MAC]
        wifi_data.append(mac)
    

    return wifi_data


### TESTING ###



position = [0, 0, 0]



wifi_data = get_RSSI_MAC(position)
print(wifi_data)