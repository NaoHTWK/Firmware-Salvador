#!/usr/bin/env bash

set -e

ip=$(./deploy-helpers/local_ipv4.py --no-fail)

IMAGE="cturra/ntp"
CONTAINER="ntp"
CONFIG_PATH="$(pwd)/chrony.conf"

echo "📦 Checking for Docker image $IMAGE..."
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "⬇️ Pulling $IMAGE..."
    docker pull "$IMAGE"
else
    echo "✅ Image $IMAGE already present."
fi

if [[ -z "$ip" || ! "$ip" =~ ^10\.0\.([0-9]{1,3})\.([0-9]{1,3})$ ]]; then
    echo "❌ IP is empty or not a 10.0.xx.xx address: '$ip'"
    exit 1
fi

echo "🚀 Starting temporary LAN NTP container..."
docker run --rm --name "$CONTAINER" \
    --detach \
    --publish=123:123/udp \
    -v "$CONFIG_PATH":/etc/chrony/chrony.conf:ro \
    "$IMAGE"

echo ""
echo "✅ NTP server is running."
echo "➡ Use this IP for clients: $ip"
echo "🔁 Press Ctrl+C to stop."

trap 'echo -e "\n🛑 Stopping container..."; docker stop "$CONTAINER"; exit 0' INT

while true; do
    sleep 10
done
