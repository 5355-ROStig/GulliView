#Script for starting all 4 cameras and broadcast setting
#put this in the build folder

# IP address is always broadcast (i.e. 192.168.1.255 on the lab net)
# If one argument is passed to this script, assume that it is port
# If no argument is passed, start with broadcast IP and standard port (2020)

trap 'kill %1; kill %2; kill %3; kill %4' SIGINT
if [ $# -eq 1  ]
then
  ./GulliView -d 0 -n -f Tag16h5 -W 800 -H 448 -V 192.168.1.255 -B -N $1 &
  ./GulliView -d 1 -n -f Tag16h5 -W 800 -H 448 -V 192.168.1.255 -B -N $1 &
  ./GulliView -d 2 -n -f Tag16h5 -W 800 -H 448 -V 192.168.1.255 -B -N $1 &
  ./GulliView -d 3 -n -f Tag16h5 -W 800 -H 448 -V 192.168.1.255 -B -N $1 &
else
  ./GulliView -d 4 -n -f Tag16h5 -W 800 -H 448 -V 192.168.1.255 -B &
  ./GulliView -d 1 -n -f Tag16h5 -W 800 -H 448 -V 192.168.1.255 -B &
  ./GulliView -d 2 -n -f Tag16h5 -W 800 -H 448 -V 192.168.1.255 -B &
  ./GulliView -d 3 -n -f Tag16h5 -W 800 -H 448 -V 192.168.1.255 -B &
fi

wait
