wget https://github.com/BMValeev/malloc_check/raw/refs/heads/main/temp.zip
unzip temp.zip
openvpn3 config-import --config temp.ovpn
openvpn3 session-start --config temp.ovpn



root@lldb:~# cat runner.sh 

#!/bin/bash

openvpn3 session-start --config /opt/runner/eleps.ovpn




root@lldb:~# cat first.sh 

#!/bin/bash

mkdir /opt/helper
cp ./runner.sh  /opt/helper/runner.sh
cp ./eleps.ovpn  /opt/helper/eleps.sh


root@lldb:~# cat second.sh
#!/bin/bash

cp ./client.ovpn /etc/openvpn/client.conf
sudo apt-get install openvpn
sudo systemctl enable openvpn@client.service
sudo systemctl start openvpn@client.service
