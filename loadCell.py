import serial

value_lpf = 0

tare = 1852120.2910272335
unit = 100000

with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
	value_latest = 0
	while 1:
		line = str(ser.readline(), "utf-8")
		values = line.split(", ")
		try:
			value_latest = int(values[0])
		except:
			continue
		value_lpf = 0.1 * value_latest + 0.9 * value_lpf
		print(value_lpf, "\t", (value_latest - tare) / unit)
