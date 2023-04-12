import canopen
import time

network = canopen.Network()
network.connect(bustype='socketcan', channel='can0')

P_MIN = -12.5
P_MAX = 12.5
V_MIN = -65.0
V_MAX = 65.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -18.0
T_MAX = 18.0

I_MAX = 15.0

position = 0.5
velocity = 5.0
kd = 1.0
kp = 0.5 
torque = 0.0

# Enter Motor Mode
enter_motor_mode = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
# Exit Motor Mode
exit_motor_mode = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
# Zero Position Sensor - sets the mechanical position to zero.
zero_position_sensor = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]


def float_to_uint(x, x_min, x_max, bits):
    # Converts a float to an unsigned int, given range and number of bits ///
    span = x_max - x_min
    offset = x_min
    return int((x-offset)*(((1 << bits)-1))/span)


def uint_to_float(x_int, x_min, x_max, bits):
    # /// converts unsigned int to float, given range and number of bits ///
    span = x_max - x_min
    offset = x_min
    return (x_int)*span/(((1 << bits)-1)) + offset


def unpack_reply(can_id, data, timestamp):
    # /// unpack ints from can buffer ///
    id = data[0]
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    i_int = ((data[4] & 0xF) << 8) | data[5]
    # // / convert ints to floats // /
    p = uint_to_float(p_int, P_MIN, P_MAX, 16)
    v = uint_to_float(v_int, V_MIN, V_MAX, 12)
    i = uint_to_float(i_int, -I_MAX, I_MAX, 12)

    # if (id == 2){
    #     theta1 = p;
    #     dtheta1 = v;}
    # else if (id == 3){
    #     theta2 = p;
    #     dtheta2 = v;}

    #print("id: {}\tp: {}\tv: {}\ti: {}".format(id, p, v, i))
    print("id: {}".format(id))
    print("p: {}".format(p))
    print("v: {}".format(v))
    print("i: {}".format(i))

network.subscribe(0, unpack_reply)
network.send_message(1, enter_motor_mode)
network.send_message(1, zero_position_sensor)
time.sleep(1)


initial_data = [0, 0, 0, 0, 0, 0, 0, 0]
p_int = float_to_uint(0, P_MIN, P_MAX, 16)
v_int = float_to_uint(0, V_MIN, V_MAX, 12)
kp_int = float_to_uint(0, KP_MIN, KP_MAX, 12)
kd_int = float_to_uint(0, KD_MIN, KD_MAX, 12)
t_int = float_to_uint(0, T_MIN, T_MAX, 12)

initial_data[0] = p_int >> 8
initial_data[1] = p_int & 0xFF
initial_data[2] = v_int >>4
initial_data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
initial_data[4] = kp_int & 0xFF
initial_data[5] = kd_int  >> 4
initial_data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
initial_data[7] = t_int & 0xff
network.send_message(1, initial_data)

time.sleep(1)

num_msgs = 0
while True:
    position += velocity * 0.001
    #position += 0.001
    print("pos_d:", position)
    if (position > P_MAX):
        position = P_MIN

    p_int = float_to_uint(position, P_MIN, P_MAX, 16)
    v_int = float_to_uint(velocity, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(torque, T_MIN, T_MAX, 12)

    data = [0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = p_int >> 8
    data[1] = p_int & 0xFF
    data[2] = v_int >> 4
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
    data[4] = kp_int & 0xFF
    data[5] = kd_int >> 4
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
    data[7] = t_int & 0xff

    network.send_message(1, data)
    num_msgs = num_msgs + 1

    time.sleep(.0003)

network.send_message(1, exit_motor_mode)

