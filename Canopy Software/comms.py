import serial
import struct

ser = serial.Serial('COM6', 115200)
# ser.baudrate = 115200
# ser.port = "COM5"
# ser.open()
print(ser.name)


# Define the motor_data struct format
# 'f' for float, 'b' for int8_t
setpoint_data_struct_format = 'iiiiiBBBx'

motor_data_struct_format = 'iBxxxiBxxxiBxxxiBxxxiBxxx'

# Calculate the size of the struct
setpoint_data_struct_size = struct.calcsize(setpoint_data_struct_format)

# Calculate the size of the struct
motor_data_struct_size = struct.calcsize(motor_data_struct_format)

lead  = 0.008

def read_motor_data():
    # # Read the number of bytes corresponding to the struct size
    # data = ser.read(setpoint_data_struct_size)

    # # Unpack the data according to the struct format
    # unpacked_data = struct.unpack(setpoint_data_struct_format, data)

    # # Create a dictionary for better readability
    # recieved_setpoint_data = {
    #     'x_step': unpacked_data[0],
    #     'y_step': unpacked_data[1],
    #     'z_step': unpacked_data[2]
    # }

    # Read the number of bytes corresponding to the struct size
    data = ser.read(motor_data_struct_size)

    # Unpack the data according to the struct format
    unpacked_data = struct.unpack(motor_data_struct_format, data)

    # Create a dictionary for better readability
    received_setpoint_data = {
        'step z': float(unpacked_data[0])/6400*lead,
        'run z': unpacked_data[1],
        'step y': float(unpacked_data[2])/6400*lead,
        'run y': unpacked_data[3],
        'step x': float(unpacked_data[4])/6400*lead,
        'run x': unpacked_data[5],
        'step r': float(unpacked_data[6])/6400*lead,
        'run r': unpacked_data[7],
        'step n': float(unpacked_data[8])/6400*lead,
        'run n': unpacked_data[9]
    }

    return received_setpoint_data

#in meters
# lead  = 0.008

# #can use this to position based off of percentage of extrusion
# max_dist_z = 0
# max_dist_y = 0
# max_dist_x = 0
# max_dist_r = 0
# max_dist_n = 0

# max_z_steps = int(max_dist_z/lead*6400)
# max_y_steps = int(max_dist_y/lead*6400)
# max_x_steps = int(max_dist_x/lead*6400)
# max_r_steps = int(max_dist_r/lead*6400)
# max_n_steps = int(max_dist_n/lead*6400)

# z = int(0*max_z_steps)
# y = int(0*max_y_steps)
# x = int(0*max_x_steps)
# r = int(0*max_r_steps)
# n = int(0*max_n_steps)
# nail = 0
# vacuum = 0
# cal = 0

###############################################################################


#can use this to position based on actual distance from the starting position
dist_z = -0.01
dist_y = -0.3
dist_x = -0.25
dist_r = 0
dist_n = 0

z_steps = int(dist_z/lead*6400)
y_steps = int(dist_y/lead*6400)
x_steps = int(dist_x/lead*6400)
r_steps = int(dist_r/lead*6400)
n_steps = int(dist_n/lead*6400)


z = z_steps
y = y_steps
x = x_steps
r = r_steps
n = n_steps
nail = 0
vacuum = 0
cal = 0


ser.write(b'$')
position_data_struct = (z,
                        y,
                        x,
                        r,
                        n,
                        nail,
                        vacuum,
                        cal)

ser.write(struct.pack(setpoint_data_struct_format, *position_data_struct))

motor_data = read_motor_data()
print(motor_data)