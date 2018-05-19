#!/bin/bash

# Setup McGill Robotics AUV workspace

# Fail on first error
set -e

header=$(tput setab 1; tput setaf 0)
section=$(tput bold; tput setaf 2)
warning=$(tput bold; tput setaf 1)
reset=$(tput sgr0)

echo "${header}Welcome to McGill Robotics setup AUV setup script!${reset}"

if [[ "$(whoami)" == "root" ]]; then
  echo "${warning}Please DO NOT run this as root!${reset}"
  exit -1
fi

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"

# Ask for sudo power
sudo -v

# Update Git repo
echo "${section}Update git repository...${reset}"
  git submodule sync --recursive
  git submodule update --init --recursive --force
echo

echo "${section}Update ROS dependencies${reset}"
# ROS package dependencies
if [[ -x "$(command -v rosdep)" ]]; then
  rosdep update
  pushd catkin_ws
  rm -rf build devel logs
  if [[ "$(lsb_release -cs)" == "xenial" ]]; then
    echo "Installing dependancies for Ubuntu 16.04..."
    rosdep install -r -y --from-paths src --rosdistro kinetic --ignore-src \
    --skip-keys="arduino-core"
  elif [[ "$(lsb_release -cs)" == "trusty" ]]; then
    echo "Installing dependancies for Ubuntu 14.04..."
    rosdep install -r -y --from-paths src --rosdistro jade --ignore-src \
    --skip-keys="arduino-core"
  fi
  popd
  echo
fi

# Increase USBFS buffer size
echo "${section}Increase USBFS buffer size...${reset}"
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

# Symlink udev rules
if [[ ! -f "/etc/udev/rules.d/9-auv.rules" ]]; then
  echo "${section}Symlinking udev rules...${reset}"
  sudo ln -sf "$(dirname "$(readlink -f "$0")")"/config/udev/9-auv.rules \
    /etc/udev/rules.d/

  echo "Reload udev rules..."
  sudo udevadm control --reload
  sudo udevadm trigger
  echo
fi

# Setup Arduino IDE on x64
if [[ "$(uname -m)" == "x86_64" ]]; then
  if [[ ! -d /opt/arduino ]]; then
    echo "${sestion}Installing Arduino IDE...${reset}"
    # Remove apt-get old version
    sudo apt-get purge -y arduino

    # Download and extract
    arduino_ver=arduino-1.6.5-r5
    arduino_tar=${arduino_ver}-linux64.tar.xz
    pushd ~/
    wget -O ${arduino_tar} https://downloads.arduino.cc/${arduino_tar}
    tar -xf ${arduino_tar}
    rm -f ${arduino_tar}

    # Move to /opt
    sudo mv ${arduino_ver} /opt/arduino

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

# Copy st-flash for hydrophones
if [[ ! -e /usr/bin/st-flash ]]; then
  echo "${section}Add st-flash to /usr/bin/...${reset}"
  st_flash_dir="$(dirname "$(readlink -f "$0")")"/catkin_ws/src/nucleo/drivers
  sudo cp "${st_flash_dir}/st-flash" /usr/bin/st-flash
fi

function _symlink {
  echo 'Symlinking '"${1}"' to '"${2}"'...'
  if [[ "${3}" == "root" ]]; then
    sudo ln -s "${1}" "${2}"
  else
    ln -s "${1}" "${2}"
  fi
}

function symlink_file  {
  if [[ ! -e "${2}" ]]; then
    if [[ -L "${2}" ]]; then
      echo 'Removing broken link '"${2}"'...'
      sudo rm -f "${2}"
    fi
    _symlink "${@}"
  fi
}

# Symlink tmuxinator config
if [[ ! -d "${HOME}/.tmuxinator" ]]; then
  mkdir -p "${HOME}/.tmuxinator"
fi

for file in $(ls config/mux); do
  dest="${HOME}/.tmuxinator/${file}"
  orig="${DIR}/config/mux/${file}"
  symlink_file "${orig}" "${dest}"
done

# Add auv to hosts file
if [[ ! $(cat /etc/hosts | grep auv) ]]; then
  sudo sed -i "1 i 10.10.10.10\tauv" /etc/hosts
fi

# Add user to dialout to get access to devices
echo "${section}Adding user to groups...${reset}"
sudo usermod -aG dialout $(whoami)
echo

echo "Setup complete."
