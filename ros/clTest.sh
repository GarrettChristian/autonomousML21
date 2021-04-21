#!/bin/bash

fake_pose_flag=''

print_usage() {
  printf "Usage: -p activates fake_pose\n"
}

while getopts 'p' flag; do
  case "${flag}" in
    p) fake_pose_flag='true' ;;
    *) print_usage
       exit 1 ;;
  esac
done

if [ -z "$fake_pose_flag" ]
then
    echo "starting pose tracking"
else
    echo "starting fake pose tracking"
fi

echo "done"

