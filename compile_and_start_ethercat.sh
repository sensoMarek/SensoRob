#!/bin/bash

# Function to remove previously installed EtherCAT
remove_ethercat() {
    if [ -d "/usr/local/etherlab" ]; then
        rm -rf /usr/local/etherlab
    fi
    if [ -L "/usr/bin/ethercat" ]; then
        rm /usr/bin/ethercat
    fi
    if [ -L "/etc/init.d/ethercat" ]; then
        rm /etc/init.d/ethercat
    fi
    if [ -f "/etc/sysconfig/ethercat" ]; then
        rm /etc/sysconfig/ethercat
    fi
    if [ -f "/etc/udev/rules.d/99-EtherCAT.rules" ]; then
        rm /etc/udev/rules.d/99-EtherCAT.rules
    fi
}

# Function to clone and compile Etherlab
compile_etherlab() {
    git clone https://gitlab.com/etherlab.org/ethercat.git /ethercat
    cd /ethercat
    git checkout stable-1.5
    ./bootstrap
    ./configure --prefix=/usr/local/etherlab --disable-8139too --disable-eoe --enable-generic --with-linux-dir=/usr/src/linux-headers-$(uname -r)
    make all modules
    make modules_install install
    depmod

    # Configure system
    ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
    ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
    mkdir -p /etc/sysconfig
    cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat

    # Create a new udev rule
    echo 'KERNEL=="EtherCAT[0-9]*", MODE="0666"' > /etc/udev/rules.d/99-EtherCAT.rules
}

# Function to configure the network adapter for EtherCAT
configure_ethercat() {
    MAC_ADDRESS=$(ifconfig | grep -A 1 'enp' | grep ether | awk '{print $2}')
    echo -e "MASTER0_DEVICE=\"$MAC_ADDRESS\"\nDEVICE_MODULES=\"generic\"" > /etc/sysconfig/ethercat
}

# Function to start the EtherCAT master
start_ethercat() {
    /etc/init.d/ethercat start
}

# Main script execution
main() {
    remove_ethercat
    compile_etherlab
    configure_ethercat
    start_ethercat
}

# Run the main function
main
