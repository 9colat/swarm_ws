#! /bin/bash
cd
echo "hey"
arduino-cli config init
cd .arduino15
perl -pi -e 's/always_export_binaries: false/always_export_binaries: true/g' arduino-cli.yaml
echo "whats wrong with your face?"

kill $PPID
