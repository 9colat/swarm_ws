#! /bin/bash

nr=$1
echo "log$nr.txt"

scp ~/test_data/log$nr.txt nicoleg@192.168.0.9:/home/nicoleg/Documents
