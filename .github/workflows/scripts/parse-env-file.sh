#!/usr/env bash

GITHUB_ENV=$1

echo "Parsing environment variables..."
if [ ! -f .env ]; then
  echo "No .env file found. Exiting."
  return 1
fi

while IFS='=' read -r key value; do
    if [[ -n "$key" ]] && [[ ! "$key" =~ ^\s*# ]]; then
        # Remove surrounding quotes if they exist
        echo "Setting environment variable: $key = $value"
        echo "$key=$value" >> "$GITHUB_ENV"
    fi
done < .env
