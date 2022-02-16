#! /bin/bash
echo "hey"


echo $1
echo $#
if [ $# -eq 0 ]
  then
    echo "No arguments supplied"
    name="not set"
fi
if [ $# -gt 0 ]
  then
    if [$1 -eq 1]
      then
        name="motor"
    fi
    if [$1 -eq 2]
      then
        name="not_motor"
    fi
fi

echo $name
