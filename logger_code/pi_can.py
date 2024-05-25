"""\
This script reads the data from the CAN bus and both prints it to the
console and adds it to the database specified.
It is intended to be used with the RS485-CAN HAT on a Raspberry Pi.
https://www.waveshare.com/rs485-can-hat.htm
"""

import os
import can
import sqlite3

os.system("sudo ip link set can0 type can bitrate 100000")
os.system("sudo ifconfig can0 up")

can0 = can.interface.Bus(channel="can0", bustype="socketcan")  # socketcan_native
conn = sqlite3.connect('../daq_gui_1/database_2/the_database.db')	# UPDATE FILEPATH AS NEEDED
cursor = conn.cursor()


def translate_u8_to_xyz(rx_buf):
    tmp = (rx_buf[0] << 16) | rx_buf[1] << 8 | rx_buf[2] << 0

    return tmp
    
def create_table(table):
    conn.execute('CREATE TABLE IF NOT EXISTS ' +table + ' (data_1 INTEGER)')


buf = b''
while True:
	# receive message
	msg = can0.recv(10.0)
	if msg is None:
		print("Timeout occurred, no message.")
	else:
		if msg.arbitration_id == 123:
			value = translate_u8_to_xyz(msg.data)
			create_table("adc_data") 
			cursor.execute("INSERT INTO adc_data (data_1) VALUES (" + str(value) + ")")
			conn.commit()
		elif msg.arbitration_id == 100:
			value = msg.data[0]
			create_table("gps_data")
			cursor.execute("INSERT INTO gps_data (data_1) VALUES (" + str(value) + ")")
			conn.commit()
		else:
			value = msg.data[0]
			create_table("mkr_data")
			cursor.execute("INSERT INTO mkr_data (data_1) VALUES (" + str(value) + ")")
			conn.commit()
			
		print(msg)
	
		#(x, y, z) = translate_u8_to_xyz(msg.data)
		#print(f"x:{x:>5}, y:{y:>5}, z:{z:>5}")
		#print(msg.arbitration_id)

# os.system("sudo ifconfig can0 down")

    

"""
	# GPS code that worked on its own but we didnt get it to work WITH the other stuff as well (remapping GPIOs on PI may have messed with the CAN hat even though it shouldn't have...)
	# this code requires updating GPIOs on the PI and using them for UART communication with this GPS module: https://www.makerfocus.com/products/gt-u7-gps-module-compatible-with-51-microcontroller-stm32-arduino-uno-r3-and-ipex-antenna
	# Steps for Pi UART config using pins not used by the CAN hat are in the file "Pi UART Special config instructions"

	# before main
	import pynmea2 # use: sudo apt install python3-nmea2
	import serial
	serial_port = serial.Serial("/dev/ttyAMA2", 9600) # This is using UART2
	 
	# GPS stuff in main loop
	try:
		if serial_port.in_waiting > 0:
			received_data = serial_port.read(serial_port.in_waiting)
			buf += received_data
			if b'\n' in buf:
				sentences = buf.split(b'\n')
				for sentence in sentences[:-1]:
					
#					print(sentence.decode("utf-8"))
					
					try:
						msg = pynmea2.parse(sentence.decode("utf-8"))
						try:
							lat = msg.latitude
							lon = msg.longitude
							print("Lat: ", lat, ", Lon: ", lon, "\n------------------------\n")
							create_table("gps_data") 
							cursor.execute("INSERT INTO gps_data (data_1, data_2) VALUES (", lat, lon, ")")
							conn.commit()							
						except:
							pass	# probably doesnt have a good signal, ignore it
					except:
						pass		# probably started reading from a funky spot, ignore it
				buf = sentences[-1]
	except serial.SerialException as e:
		print(e)
"""