#!/bin/bash
# Sanity check: $REVPI_A_HOST (ros2_env.sh) phải khớp <address> trong
# fastdds_peers.xml. FastDDS không support env-var trong locator address
# nên file XML phải sửa tay khi đổi IP — script này nhắc nếu lệch.
set -u

WS="${WS:-/home/pi/ros2_ws}"
ENV_FILE="$WS/ros2_env.sh"
XML_FILE="$WS/fastdds_peers.xml"

# shellcheck source=/dev/null
source "$ENV_FILE"

if [ -z "${REVPI_A_HOST:-}" ]; then
    echo "❌ REVPI_A_HOST chưa được set trong $ENV_FILE"
    exit 1
fi

# Lấy mọi IP RevPi A trong XML (loại bỏ IP Pi 5 = 192.168.27.247)
xml_ips=$(grep -oE '<address>192\.168\.27\.[0-9]+</address>' "$XML_FILE" \
            | sed -E 's/<\/?address>//g' \
            | grep -v '^192\.168\.27\.247$' \
            | sort -u)

if [ -z "$xml_ips" ]; then
    echo "⚠️  Không tìm thấy IP RevPi A trong $XML_FILE"
    exit 1
fi

mismatch=0
for ip in $xml_ips; do
    if [ "$ip" != "$REVPI_A_HOST" ]; then
        echo "❌ Mismatch: XML có $ip nhưng REVPI_A_HOST=$REVPI_A_HOST"
        mismatch=1
    fi
done

if [ $mismatch -eq 0 ]; then
    echo "✅ REVPI_A_HOST=$REVPI_A_HOST khớp với fastdds_peers.xml"
    exit 0
fi

echo ""
echo "→ Sửa tay 2 dòng <address> trong $XML_FILE thành $REVPI_A_HOST"
echo "→ Rồi chạy: bash deploy_revpi.sh   (sync lên RevPi A)"
exit 1
