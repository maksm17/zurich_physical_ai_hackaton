#!/usr/bin/env bash
set -euo pipefail
HOST=${1:?usage: deploy.sh user@host [--no-up]}
REMOTE_DIR=${REMOTE_DIR:-zurich}
SSH="ssh ${SSH_KEY:+-i $SSH_KEY}"
cd "$(dirname "$0")/.."

rsync -az --delete -e "$SSH" \
  --exclude .git --exclude 'ws/build' --exclude 'ws/install' --exclude 'ws/log' \
  --exclude '__pycache__' --exclude '/zurich' --exclude '/zurich.pub' \
  --exclude '/zurich_key' --exclude '/zurich_key.pub' \
  ./ "$HOST:$REMOTE_DIR/"

if [ "${2:-}" = "--no-up" ]; then
  $SSH "$HOST" "cd $REMOTE_DIR && docker compose build"
elif [ "${2:-}" = "--restart" ]; then
  $SSH "$HOST" "cd $REMOTE_DIR && docker compose restart && docker compose logs --tail 30"
else
  $SSH "$HOST" "cd $REMOTE_DIR && docker compose up -d --build && docker compose logs --tail 30"
fi
