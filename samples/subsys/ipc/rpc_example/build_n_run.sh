#!/bin/bash

while getopts "s:" o; do
	case "${o}" in
		s)
			s=${OPTARG}
			;;
		*)
			;;
	esac
done

echo -e "\e[1m[-] Erasing APP and NET cores\e[0m"
nrfjprog --eraseall

cd ./application
echo -e "\e[1m[-] Building APP core\e[0m"

if [ "$s" = "nolib" ]; then
	west build -p -b nrf5340_dk_nrf5340_cpuapp -- -DOVERLAY_CONFIG=nolib.conf
	a="TinyCBOR"
elif [ "$s" = "mpack" ]; then
	west build -p -b nrf5340_dk_nrf5340_cpuapp -- -DOVERLAY_CONFIG=mpack.conf
	a="MPack"
elif [ "$s" = "cmp" ]; then
	west build -p -b nrf5340_dk_nrf5340_cpuapp -- -DOVERLAY_CONFIG=cmp.conf
	a="CMP"
else
	west build -p -b nrf5340_dk_nrf5340_cpuapp
	a="TinyCBOR"
fi

echo -e "\e[1m[-] Flashing APP core\e[0m"
west flash

cd ../network
echo -e "\e[1m[-] Building NET core\e[0m"

if [ "$s" = "nolib" ]; then
	west build -p -b nrf5340_dk_nrf5340_cpunet -- -DOVERLAY_CONFIG=nolib.conf
elif [ "$s" = "mpack" ]; then
	west build -p -b nrf5340_dk_nrf5340_cpunet -- -DOVERLAY_CONFIG=mpack.conf
elif [ "$s" = "cmp" ]; then
	west build -p -b nrf5340_dk_nrf5340_cpunet -- -DOVERLAY_CONFIG=cmp.conf
else
	west build -p -b nrf5340_dk_nrf5340_cpunet
fi

echo -e "\e[1m[-] Flashing NET core\e[0m"
west flash

echo -e "\e[1m[=] Done, serialization using $a\e[0m"
