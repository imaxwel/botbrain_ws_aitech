#!/bin/bash
# Check if dialog is installed, if not install it
if ! command -v dialog &> /dev/null; then
    echo "Dialog is not installed. Installing dialog..."

    # Detect the operating system and install dialog
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$ID
    elif [ "$(uname)" = "Darwin" ]; then
        OS="macos"
    else
        echo "Unable to detect operating system."
        exit 1
    fi

    case $OS in
        ubuntu|debian)
            sudo apt-get update
            sudo apt-get install -y dialog
            ;;
        fedora|rhel|centos)
            sudo dnf install -y dialog || sudo yum install -y dialog
            ;;
        arch|manjaro)
            sudo pacman -S --noconfirm dialog
            ;;
        macos)
            if command -v brew &> /dev/null; then
                brew install dialog
            else
                echo "Homebrew is not installed. Please install Homebrew first: https://brew.sh"
                exit 1
            fi
            ;;
        *)
            echo "Unsupported operating system: $OS"
            echo "Please install 'dialog' manually and run this script again."
            exit 1
            ;;
    esac

    # Verify Setup
    if ! command -v dialog &> /dev/null; then
        echo "Failed to install dialog. Please install it manually."
        exit 1
    fi

    echo "Dialog installed successfully!"
fi

# Check if sshpass is installed, if not install it
if ! command -v sshpass &> /dev/null; then
    echo "sshpass is not installed. Installing dialog..."

    # Detect the operating system and install dialog
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$ID
    elif [ "$(uname)" = "Darwin" ]; then
        OS="macos"
    else
        echo "Unable to detect operating system."
        exit 1
    fi

    case $OS in
        ubuntu|debian)
            sudo apt-get update
            sudo apt-get install -y sshpass
            ;;
        fedora|rhel|centos)
            sudo dnf install -y sshpass || sudo yum install -y sshpass
            ;;
        arch|manjaro)
            sudo pacman -S --noconfirm sshpass
            ;;
        macos)
            if command -v brew &> /dev/null; then
                brew install sshpass
            else
                echo "Homebrew is not installed. Please install Homebrew first: https://brew.sh"
                exit 1
            fi
            ;;
        *)
            echo "Unsupported operating system: $OS"
            echo "Please install 'sshpass' manually and run this script again."
            exit 1
            ;;
    esac

    # Verify Setup
    if ! command -v sshpass &> /dev/null; then
        echo "Failed to install sshpass. Please install it manually."
        exit 1
    fi

    echo "sshpass installed successfully!"
fi


# Set purple color scheme for dialog with different tones
export DIALOGRC="/tmp/dialogrc_custom"
cat > "$DIALOGRC" << 'EOF'
# Dialog color configuration
use_colors = ON
screen_color = (WHITE,BLACK,OFF)
shadow_color = (BLACK,BLACK,ON)
dialog_color = (WHITE,MAGENTA,OFF)
title_color = (WHITE,MAGENTA,ON)
border_color = (WHITE,MAGENTA,OFF)
border2_color = (WHITE,MAGENTA,OFF)
button_active_color = (BLACK,WHITE,ON)
button_inactive_color = (WHITE,MAGENTA,OFF)
button_key_active_color = (BLACK,WHITE,ON)
button_key_inactive_color = (WHITE,MAGENTA,OFF)
button_label_active_color = (BLACK,WHITE,ON)
button_label_inactive_color = (WHITE,MAGENTA,OFF)
inputbox_color = (WHITE,MAGENTA,OFF)
inputbox_border_color = (WHITE,MAGENTA,OFF)
inputbox_border2_color = (WHITE,MAGENTA,OFF)
searchbox_color = (WHITE,MAGENTA,OFF)
searchbox_title_color = (WHITE,MAGENTA,ON)
searchbox_border_color = (WHITE,MAGENTA,OFF)
searchbox_border2_color = (WHITE,MAGENTA,OFF)
position_indicator_color = (WHITE,MAGENTA,ON)
menubox_color = (WHITE,MAGENTA,OFF)
menubox_border_color = (WHITE,MAGENTA,OFF)
menubox_border2_color = (WHITE,MAGENTA,OFF)
item_color = (WHITE,MAGENTA,OFF)
item_selected_color = (BLACK,WHITE,ON)
tag_color = (WHITE,MAGENTA,OFF)
tag_selected_color = (BLACK,WHITE,ON)
tag_key_color = (WHITE,MAGENTA,OFF)
tag_key_selected_color = (BLACK,WHITE,ON)
check_color = (WHITE,MAGENTA,OFF)
check_selected_color = (BLACK,WHITE,ON)
uarrow_color = (WHITE,MAGENTA,ON)
darrow_color = (WHITE,MAGENTA,ON)
EOF

# Initial welcome page
dialog --title "BotBrain G1 Wifi Util" \
       --msgbox "\nWelcome to the BotBrain G1 Wifi Utility\n\nThis utility will help you to turn on the wifi radio of your G1 robot and connect to your network.\n\nPress OK to continue or ESC to exit." 12 70

# Check if user cancelled
if [ $? -ne 0 ]; then
    clear
    echo "Setup cancelled by user."
    exit 1
fi

# Get connection details
# IP
HOST=$(dialog --inputbox "Enter G1 robot IP:" 10 70 "192.168.123.164" 2>&1 >/dev/tty)
if [ $? -ne 0 ]; then
    clear
    echo "Setup cancelled by user."
    exit 1
fi

# SSH Credentials
TEMP_FORM=$(mktemp)
dialog --title "SSH Credentials" \
        --form "\nEnter your SSH credentials:\n" 14 100 2 \
        "User:" 1 1 "" 1 20 70 0 \
        "Password:" 2 1 "" 2 20 70 300 \
        2> "$TEMP_FORM"

if [ $? -ne 0 ]; then
    rm -f "$TEMP_FORM"
    clear
    echo "Installation cancelled by user."
    exit 1
fi

# Read the form values
USER=$(sed -n '1p' "$TEMP_FORM")
SSH_PASS=$(sed -n '2p' "$TEMP_FORM")
rm -f "$TEMP_FORM"

echo "USER: $USER"
echo "SSH_PASS: $SSH_PASS"

# WiFi Credentials
TEMP_FORM=$(mktemp)
dialog --title "WiFi Credentials" \
        --form "\nEnter your WiFi credentials:\n" 14 100 2 \
        "SSID:" 1 1 "" 1 20 70 0 \
        "Password:" 2 1 "" 2 20 70 300 \
        2> "$TEMP_FORM"

if [ $? -ne 0 ]; then
    rm -f "$TEMP_FORM"
    clear
    echo "Installation cancelled by user."
    exit 1
fi

# Read the form values
SSID=$(sed -n '1p' "$TEMP_FORM")
PASS=$(sed -n '2p' "$TEMP_FORM")
rm -f "$TEMP_FORM"

echo "SSID: $SSID"
echo "PASS: $PASS"

# Confirm
dialog --yesno "Connect to $HOST and activate WiFi on '$SSID'?" 15 70
if [ $? -ne 0 ]; then
    dialog --msgbox "Cancelled." 5 70
    exit 0
fi

# Show progress
dialog --infobox "Connecting to G1 and activating WiFi..." 13 70

# Execute commands on the board
RESULT=$(sshpass -p "$SSH_PASS" ssh -T -o StrictHostKeyChecking=no "$USER@$HOST" 2>/dev/null << EOF
    # 0. Cache sudo credentials upfront
    echo '$SSH_PASS' | sudo -S -v

    # 1. Unblock WiFi via rfkill
    sudo -S rfkill unblock wlan

    # 2. Enable WiFi radio
    sudo -S nmcli radio wifi on

    # 3. Wait until the target WiFi is visible
    for i in \$(seq 1 15); do
        FOUND=\$(nmcli dev wifi list | grep -w "$SSID")
        if [ -n "\$FOUND" ]; then
            echo "Network $SSID found!"
            break
        fi
        echo "Scanning for $SSID... (\$i/15)"
        sleep 1
    done

    if [ -z "\$FOUND" ]; then
        echo "ERROR: Network $SSID not found after timeout"
        exit 1
    fi

    # 4. Connect to the network
    sudo -S nmcli device wifi connect $SSID password '$PASS'

    # 5. Wait for IP assignment on wlan0
    for i in \$(seq 1 15); do
        WLAN_IP=\$(ip addr show wlan0 | grep 'inet ' | awk '{print \$2}' | cut -d'/' -f1)
        if [ -n "\$WLAN_IP" ]; then
            echo "IP assigned: \$WLAN_IP"
            break
        fi
        echo "Waiting for IP assignment... (\$i/15)"
        sleep 1
    done

    if [ -z "\$WLAN_IP" ]; then
        echo "ERROR: No IP assigned to wlan0 after timeout"
        exit 1
    fi

    # 6. Get wlan0 gateway dynamically
    WLAN_GW="\${WLAN_IP%.*}.1"

    # 7. Modify WiFi priority so the robot can have internet
    sudo ip route add default via "\$WLAN_GW" dev wlan0 metric 50

    # 8. Set permanently this modifications of Ethernet priority
    CONN_NAME=\$(nmcli -t -f NAME,DEVICE connection show --active | grep wlan0 | cut -d':' -f1) 
    sudo nmcli connection modify "\$CONN_NAME" ipv4.route-metric 50
    sudo nmcli connection modify "\$CONN_NAME" ipv4.never-default false


    # 9. Get the IP address on wlan interface
    ip addr show wlan0 | grep 'inet ' | awk '{print \$2}' | cut -d'/' -f1

    # 10. Exit from ssh
    exit
EOF
)

# Extract the last line as the IP
WIFI_IP=$(echo "$RESULT" | tail -n 1)

# Show result
if [ -n "$WIFI_IP" ]; then
    dialog --msgbox "WiFi activated successfully!\n\nBoard WiFi IP: $WIFI_IP" 8 70
else
    dialog --msgbox "WiFi command ran but could not retrieve IP.\n\nFull output:\n$RESULT" 15 70
fi

clear
echo "Board WiFi IP: $WIFI_IP"