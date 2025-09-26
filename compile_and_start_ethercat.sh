#!/bin/bash

# Function to remove previously installed EtherCAT
remove_ethercat() {
    echo "Removing previously installed EtherCAT..."
    if [ -d "/usr/local/etherlab" ]; then
        rm -rf /usr/local/etherlab
    fi
    if [ -L "/usr/bin/ethercat" ]; then
        rm -f /usr/bin/ethercat
    fi
    if [ -L "/etc/init.d/ethercat" ]; then
        rm -f /etc/init.d/ethercat
    fi
    if [ -f "/etc/sysconfig/ethercat" ]; then
        rm -f /etc/sysconfig/ethercat
    fi
    if [ -f "/etc/udev/rules.d/99-EtherCAT.rules" ]; then
        rm -f /etc/udev/rules.d/99-EtherCAT.rules
    fi
}

# Function to clone and compile Etherlab
compile_etherlab() {
    echo "Compiling Etherlab..."
    git clone https://gitlab.com/etherlab.org/ethercat.git /ethercat
    cd /ethercat || exit
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
    echo "Searching for a physical network adapter for EtherCAT..."
    
    # Find the first physical network interface that isn't loopback
    INTERFACE_NAME=""
    for iface in /sys/class/net/*; do
        # Get the base name of the interface (e.g., "eno1")
        iface_name=$(basename "$iface")
        
        # Skip the loopback interface
        if [ "$iface_name" = "lo" ]; then
            continue
        fi
        
        # Check if it's a physical device by seeing if the 'device' symlink exists
        # Virtual devices (bridges, tunnels, etc.) will not have this.
        if [ -d "$iface/device" ]; then
            INTERFACE_NAME="$iface_name"
            break # Found one, so we stop
        fi
    done

    if [ -n "$INTERFACE_NAME" ]; then
        # Read the MAC address directly from the file system
        MAC_ADDRESS=$(cat "/sys/class/net/$INTERFACE_NAME/address")
        
        if [ -n "$MAC_ADDRESS" ]; then
            echo "Found interface: $INTERFACE_NAME with MAC: $MAC_ADDRESS"
            echo "Configuring /etc/sysconfig/ethercat..."
            # Use printf for safer, more predictable output
            printf "MASTER0_DEVICE=\"%s\"\nDEVICE_MODULES=\"generic\"\n" "$MAC_ADDRESS" > /etc/sysconfig/ethercat
            # To test without writing to the system file, uncomment the line below
            # printf "MASTER0_DEVICE=\"%s\"\nDEVICE_MODULES=\"generic\"\n" "$MAC_ADDRESS"
        else
            echo "Could not read MAC address for interface $INTERFACE_NAME. Exiting."
            exit 1
        fi
    else
        echo "No suitable physical network interface found for EtherCAT. Exiting."
        exit 1
    fi
}

# Function to start the EtherCAT master
start_ethercat() {
    echo "Starting EtherCAT master..."
    /etc/init.d/ethercat start

    # Set permissions for /dev/EtherCAT0
    echo "Setting permissions for /dev/EtherCAT0..."
    sudo chmod 777 /dev/EtherCAT0
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
