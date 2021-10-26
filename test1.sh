#! /bin/bash
sudo perl -pi -e 's/\# deb-src http://security.ubuntu.com/ubuntu focal-security main restricted/deb-src http://security.ubuntu.com/ubuntu focal-security main restricted/g' //etc/apt/sources.list
