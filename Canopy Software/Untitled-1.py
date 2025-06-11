import serial
import struct

ser = serial.Serial('COM6', 115200)
# ser.baudrate = 115200
# ser.port = "COM5"
# ser.open()
print(ser.name)


# Define the motor_data struct format
# 'f' for float, 'b' for int8_t
setpoint_data_struct_format = 'iii'

motor_data_struct_format = 'iB'

# Calculate the size of the struct
setpoint_data_struct_size = struct.calcsize(setpoint_data_struct_format)

# Calculate the size of the struct
motor_data_struct_size = struct.calcsize(motor_data_struct_format)

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
    recieved_setpoint_data = {
        'step': unpacked_data[0],
        'run': unpacked_data[1]
    }
    

    return recieved_setpoint_data

# check = ser.read()
# print(check)
motor_data = read_motor_data()
print(motor_data)