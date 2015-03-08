echo "Finding Jetson...."
sudo arp-scan --interface=eth0 --localnet | grep NVIDIA
exit 0
