#!/bin/bash

# Setup McGill Robotics AUV workspace

# Fail on first error
set -e

# Ask for sudo power
sudo -v

# Update Git repo
echo "Pulling latest chages..."
git pull && git submodule update --init --recursive
echo

# ROS package dependencies
if [[ -x "$(command -v rosdep)" ]]; then
  echo "Installing ROS package dependencies..."
  rosdep update
  pushd catkin_ws
  catkin clean -y
  rosdep install -r -y --from-paths src --rosdistro kinetic --ignore-src \
    --skip-keys="arduino-core"
  popd
  echo
fi

# Python package dependencies
echo "Installing Python package dependencies..."
sudo pip install scipy scikit-learn
echo

# Increase USBFS buffer size
if [[ $(uname -m) == "x86_64" ]]; then
  if [[ -z $(grep 'usbcore.usbfs_memory_mb=1024' /etc/default/grub) ]]; then
    echo "Increasing USB3 buffer limit..."
    REBOOT="true"
    sudo sed -e 's/"quiet"/"quiet usbcore.usbfs_memory_mb=1024"/' \
      -e 's/"quiet splash"/"quiet splash usbcore.usbfs_memory_mb=1024"/' \
      -i /etc/default/grub
    sudo update-grub
  fi
fi

# PointGrey camera drivers
echo "Installing PointGrey drivers..."
if [[ $(uname -m) == "x86_64" ]]; then
  sudo apt-get install -y libraw1394-11 libgtkmm-2.4-1v5 libglademm-2.4-1v5 \
    libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 \
    libusb-1.0-0
  if [[ -z "$(dpkg -l | grep flycap)" ]]; then
    pushd drivers/flycapture2-2.10.3.237-amd64
    sudo sh install_flycapture.sh
    popd
    sudo udevadm trigger
  fi
fi


# Symlink udev rules
if [[ ! -e "/etc/udev/rules.d/9-auv.rules" ]]; then
  echo "Symlinking udev rules..."
  sudo ln -s $(dirname $(readlink -f $0))/9-auv.rules \
    /etc/udev/rules.d/

  echo "Reload udev rules..."
  sudo udevadm control --reload
  sudo udevadm trigger
  echo
fi

# Setup Arduino IDE on x64
if [[ "$(uname -m)" == "x86_64" ]]; then
  echo "Installing Arduino IDE..."
  if [[ ! -d /opt/arduino ]]; then
    # Remove apt-get old version
    sudo apt-get purge -y arduino

    echo "Installing Arduino IDE..."
    # Download and extract
    arduino_tar=arduino-1.6.5-r5-linux64.tar.xz
    pushd ~/
    wget -O ${arduino_tar} https://downloads.arduino.cc/${arduino_tar}
    tar -xf ${arduino_tar} arduino
    rm -f ${arduino_tar}

    # Move to /opt
    sudo mv arduino /opt/arduino

    # Install .desktop file for desktop machines
    if [[ $(command -v desktop-file-install) ]]; then
      echo '[Desktop Entry]' >> arduino.desktop
      echo 'Type=Application' >> arduino.desktop
      echo 'Name=Arduino IDE' >> arduino.desktop
      echo 'GenericName=Arduino IDE' >> arduino.desktop
      echo 'Comment=Open-source electronics prototyping platform' \
        >> arduino.desktop
      echo 'Exec=/usr/bin/arduino' >> arduino.desktop
      echo 'Icon=/opt/arduino/lib/arduino_icon.ico' >> arduino.desktop
      echo 'Terminal=false' >> arduino.desktop
      echo 'Categories=Development;IDE;Electronics;' >> arduino.desktop
      echo 'MimeType=text/x-arduino;' >> arduino.desktop
      echo 'Keywords=embedded electronics;electronics;avr;microcontroller;' \
        >> arduino.desktop
      echo 'StartupWMClass=processing-app-Base' >> arduino.desktop
      chmod +x arduino.desktop
      sudo desktop-file-install arduino.desktop
      rm -f arduino.desktop
    fi

    # Symlink command-line executable
    if [[ ! -e /usr/bin/arduino ]]; then
      sudo ln -s /opt/arduino/arduino /usr/bin/arduino
    fi
    popd
    echo
  fi
fi

# Add user tp dialout to get access to devices
echo "Adding user to groups..."
sudo usermod -aG dialout robotics
echo

echo "Setup complete."
