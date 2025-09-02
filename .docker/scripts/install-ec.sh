#!/usr/bin/env bash

apt-get update && apt-get install -y wget tar && \
cd /tmp && \
wget https://github.com/editorconfig-checker/editorconfig-checker/releases/download/v3.4.0/ec-linux-amd64.tar.gz && \
tar -xzf ec-linux-amd64.tar.gz && \
chmod +x ./bin/ec-linux-amd64 && \
sudo mv ./bin/ec-linux-amd64 /usr/local/bin/ec-linux-amd64 && \
sudo ln -sf /usr/local/bin/ec-linux-amd64 /usr/local/bin/ec
