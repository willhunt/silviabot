#!/bin/sh

sudo -i

# Update
printf "Updating linux...  "
apt-get update -y
apt-get upgrade -y
apt-get autoremove

# Hostname (used to detect simulation mode in Django)
sudo hostnamectl set-hostname silvia
sudo hostname silvia

# Add permissions
printf "Add permissions for bash scripts...  "
chmod +x ~/silvia/update.sh
chmod +x ~/silvia/update_branch.sh
chmod +x ~/silvia/update_arduino.sh

# Install ROS
# Allow restricted, universe, and multiverse repositories
sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse
# Setup sources
printf "Adding repositories..."
sudo sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
printf "Installing ROS..."
sudo apt install ros-noetic-ros-base python-rosdep gdb -y
printf "Setup rosdep..."
cd ~/ottobot/catkin_ws
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Modify bash.rc to source required files
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/ottobot/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install stuff
printf "Installing linux packages...  "
apt install git python3-pip python3-venv libopenjp2-7 libtiff5 apache2 apache2-dev libapache2-mod-wsgi-py3 redis-server i2c-tools sshfs postgresql libpq-dev postgresql-client postgresql-client-common python-dev -y

# Static IP - add lines to file
printf "Setting up static IP...  "
# cat >> /etc/dhcpcd.conf <<EOL

# # Setup static IP
# interface wlan0
# static ip_address=192.168.0.6/24
# static routers=192.168.0.1
# static domain_name_servers=192.168.0.1
# EOL

# sudo touch /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
echo "network: {config: disabled}" > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg

rm /etc/netplan/50-cloud-init.yaml
touch /etc/netplan/50-cloud-init.yaml
cat >> /etc/netplan/50-cloud-init.yaml <<EOL
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      access-points:
        VM369865-2G:
          password: zsrannnx
      dhcp4: no
      addresses: [192.168.0.6/24]
      gateway4: 192.168.0.1
      nameservers:
        addresses: [192.168.0.1, 8.8.8.8]
      optional: true
EOL
netplan apply

# Python env
printf "Setting up Python environment...  "
python3 -m venv ~/.virtualenvs/venv-silvia
echo "source ~/.virtualenvs/venv-silvia/bin/activate" >> ~/.bashrc
source ~/.virtualenvs/venv-silvia/bin/activate
cd ~/silvia/silvia
pip install -r requirements.txt

# Apache server
printf "Setting up Apache server...  "
cd ~/silvia/pi_setup/
cp -f 000-default.conf /etc/apache2/sites-available/000-default.conf
# Change permissions
chmod g+w ~/silvia/silvia
# Change owner
chown www-data:www-data ~/silvia/silvia
chown www-data:www-data ~/.virtualenvs/venv-silvia
# Change group
groupadd server_group
adduser ubuntu server_group
adduser www-data server_group
adduser www-data dialout
chgrp server_group ~/silvia/silvia/

# Arduino
printf "Setting up Arduino CLI...  "
cd ~
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
echo "export PATH=\$PATH:/home/ubuntu/bin" >> ~/.bashrc
export PATH=$PATH:/home/ubuntu/bin
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install PID "Adafruit SSD1306" "Adafruit GFX"
cd ~/Arduino/libraries
git clone https://github.com/RobotDynOfficial/RBDDimmer.git


# Postgres
printf "Setting up PostgreSQL...  "
sudo -u postgres psql -c "CREATE DATABASE silviadatabase;"
sudo -u postgres psql -c "CREATE USER databaseadmin WITH PASSWORD 'databasepwd';"
sudo -u postgres psql -c "ALTER ROLE databaseadmin SET client_encoding TO 'utf8';"
sudo -u postgres psql -c "ALTER ROLE databaseadmin SET default_transaction_isolation TO 'read committed';"
sudo -u postgres psql -c "ALTER ROLE databaseadmin SET timezone TO 'GB';"
sudo -u postgres psql -c "GRANT ALL PRIVILEGES ON DATABASE silviadatabase TO databaseadmin;"

# Supervisor
# Make logging directories
sudo mkdir -p /var/log/celery
sudo mkdir -p /var/run/celery
# sudo mkdir -p /var/log/silvia
# Permissions
sudo chgrp -R server_group /var/run/celery
sudo chgrp -R server_group /var/log/celery
# sudo chgrp -R server_group /var/log/silvia
# Config
sudo cp -f supervisor/silvia_celery.conf /etc/supervisor/conf.d/silvia_celery.conf
sudo cp -f supervisor/silvia_celerybeat.conf /etc/supervisor/conf.d/silvia_celerybeat.conf
# sudo cp -f supervisor/silvia_interrupt.conf /etc/supervisor/conf.d/silvia_interrupt.conf
sudo chgrp -R server_group /var/log/supervisor
sudo chmod g+wr /var/log/supervisor/supervisord.log
# sudo supervisord -c /etc/supervisor/supervisord.conf

# Reboot
printf "Finished, now reboot."