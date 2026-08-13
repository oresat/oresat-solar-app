build:
	west build
write:
	echo "0" > /dev/ttyACM3
	echo "0" > /dev/ttyACM3
	west flash
	echo "1" > /dev/ttyACM3
