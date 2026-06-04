build:
	west build
write:
	echo "0" > /dev/ttyACM3
	west flash -r pyocd	
	echo "1" > /dev/ttyACM3
