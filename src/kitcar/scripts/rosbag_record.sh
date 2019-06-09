#!/bin/bash

check_bag()
{
    echo "";
    echo "";
    rosrun kitcar bag_check.py "$1";
}

write_rosbag_info_and_diff()
{
	echo "rosbag_name=$1" >rosbag_info.txt;
	echo "time=$now">>rosbag_info.txt;
	echo "CAR_NAME=$CAR_NAME" >>rosbag_info.txt;
	echo "">>rosbag_info.txt;
	echo "kitcar_ros repository branch:">>rosbag_info.txt;
	(cd $KITCAR_REPO_PATH/kitcar-ros && git symbolic-ref HEAD --short) >>rosbag_info.txt;
	echo "">>rosbag_info.txt;
	(cd $KITCAR_REPO_PATH/kitcar-ros && git rev-list --format=medium --max-count=1 HEAD) >>rosbag_info.txt;
	echo "">>rosbag_info.txt;
	echo "">>rosbag_info.txt;

	(cd $KITCAR_REPO_PATH/kitcar-ros && git diff HEAD --) >ros_diff
}

write_camera_car_specs()
{
	#extract alternate yaml-file for camera and car_specs, which can be used for parameter tuning
	awk 'BEGIN{RS="(^|\n)[a-z_]+:"; print_next_line=0; ORS="";}
	print_next_line {print; print_next_line=0;}
	RT~/car_specs|camera/ {print RT; print_next_line=1;}
	END{print "\n";}' "params_$1.yaml" > "camera_car_specs_$1.yaml"
}

if [ -z "$KITCAR_REPO_PATH" ]
then
    KITCAR_REPO_PATH=$HOME;
fi

if [ $# -eq 0 ] ||  [ "$1" = '-h' ] || [ "$1" = '--help' ]
then
	tput setaf 2; echo 'Usage: sh rosbag_record.sh -o PREFIX TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'
	echo "or"
	echo 'Usage: sh rosbag_record.sh -O NAME TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'
	echo "or"
	echo 'Usage: sh rosbag_record.sh TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'; tput sgr0
	echo ""
	echo "Create folder, dump parameters and record rosbag in this directory."
	echo ""
	echo 'ROSBAG_RECORD_OPTIONS:'
	rosbag record -h | sed '1,3 d'
	exit 0
elif [ "$1" = '-o' ]
then
	rosbag_dir=$2
	rosbag_prefix=$(echo "$rosbag_dir" | awk -F/ '{ if($NF!=""){ folder=$NF} else if(NF>1){ folder=$(NF-1)};  print folder; }')
	rosbag_dir=$(echo "$rosbag_dir" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
	rosbag_prefix="$rosbag_prefix""_"
	now=$(date +"%Y-%m-%d-%H-%M-%S")

	mkdir "$rosbag_dir""_""$now" &&
	(
		cd "$rosbag_dir""_""$now" && { 
		write_rosbag_info_and_diff "$rosbag_dir$now"

		rosparam dump "params_$rosbag_prefix$now.yaml";
		write_camera_car_specs "$rosbag_prefix$now"
		shift;
		shift;

    # shellcheck disable=SC2064
		trap "{ check_bag $rosbag_prefix$now.bag; }" INT
		rosbag record -O "$rosbag_prefix$now" "$@";}
	)

elif [ $1 = '-O' ]
then
	rosbag_dir=$2
	rosbagname=$(echo $rosbag_dir | awk -F/ '{ if($NF!=""){ folder=$NF} else if(NF>1){ folder=$(NF-1)};  print folder; }')
	rosbag_dir=$(echo "$rosbag_dir" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
	now=$(date +"%Y-%m-%d-%H-%M-%S")

	mkdir "$rosbag_dir" &&

	(
		cd "$rosbag_dir" && { 
		write_rosbag_info_and_diff "$rosbag_dir"

		rosparam dump "params_$rosbagname.yaml";
		write_camera_car_specs "$rosbagname"
		shift;
		shift;

    # shellcheck disable=SC2064
		trap "{ check_bag $rosbagname.bag ; }"  INT
		rosbag record -O "$rosbagname" "$@";}
	)
	
else	
	for var in "$@"
	do
		if [ "$var" = '-o' ] || [ "$var" = '-O' ]
		then
			tput bold; tput setaf 1; echo "The name or prefix option has to be the first argument."
			echo ""
			tput setaf 2; echo 'Usage: sh rosbag_record.sh -o PREFIX TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'
			echo "or"
			echo 'Usage: sh rosbag_record.sh -O NAME TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'
			echo "or"
			echo 'Usage: sh rosbag_record.sh TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'; tput sgr0
			echo ""
			echo "Create folder, dump parameters and record rosbag in this directory."
			echo ""
			echo 'ROSBAG_RECORD_OPTIONS:'
			rosbag record -h | sed '1,3 d'
			exit 1
		fi
	done

	now=$(date +"%Y-%m-%d-%H-%M-%S")


	mkdir "$now" &&

	(
		cd $now && { 
		echo "rosbag_name=$now" >rosbag_info.txt;
		write_rosbag_info_and_diff "$now"

		rosparam dump params_$now".yaml";
		write_camera_car_specs "$now"

    # shellcheck disable=SC2064
		trap "{ check_bag $now.bag; }" INT
		rosbag record -O $now "$@";}
	)
fi

