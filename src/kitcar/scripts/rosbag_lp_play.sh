#!/bin/sh
if [ $# -eq 0 ] 
then
	tput setaf 2; echo "Usage: sh rosbag_play.sh ROSBAG_DIRECTORY [ROSBAG_PLAY_OPTIONS]"
	echo "or"
	echo "Usage: sh rosbag_play.sh ROSBAG_FILE [ROSBAG_PLAY_OPTIONS]"; tput sgr0
	echo ""
	echo "Load parameters and play rosbag created by rosbag_record.sh."
	echo ""
	tput setaf 2; echo "ROSBAG_DIRECTORY: the folder containing the .bag file and the .yaml file created by rosbag_record.sh"
	echo "ROSBAG_FILE: the .bag file. Use the ROSBAG_DIRECTORY if the rosbag was created by rosbag_record.sh" ; tput sgr0
	echo ""
	echo "ROSBAG_PLAY_OPTIONS:"
	rosbag play -h | sed '1,4 d'
	exit 0
elif [ "$1" = '-h' ] || [ "$1" = '--help' ]
then
	tput setaf 2; echo "Usage: sh rosbag_play.sh ROSBAG_DIRECTORY [ROSBAG_PLAY_OPTIONS]"
	echo "or"
	echo "Usage: sh rosbag_play.sh ROSBAG_FILE [ROSBAG_PLAY_OPTIONS]"; tput sgr0
	echo ""
	echo "Load parameters and play rosbag created by rosbag_record.sh."
	echo ""
	tput setaf 2; echo "ROSBAG_DIRECTORY: the folder containing the .bag file and the .yaml file created by rosbag_record.sh"
	echo "ROSBAG_FILE: the .bag file. Use the ROSBAG_DIRECTORY if the rosbag was created by rosbag_record.sh" ; tput sgr0
	echo ""
	echo "ROSBAG_PLAY_OPTIONS:"
	rosbag play -h | sed '1,4 d'
	exit 0
elif expr "$1" : "-.*" > /dev/null
then
	tput bold; tput setaf 1; echo "The ROSBAG_DIRECTORY has to be the first argument"; tput sgr0
	echo ""
	echo "Load parameters and play rosbag created by rosbag_record.sh."
	echo ""
	tput setaf 2; echo "Usage: sh rosbag_play.sh ROSBAG_DIRECTORY [ROSBAG_PLAY_OPTIONS]"
	echo "or"
	echo "Usage: sh rosbag_play.sh ROSBAG_FILE [ROSBAG_PLAY_OPTIONS]"; tput sgr0
	echo ""
	tput setaf 2; echo "ROSBAG_DIRECTORY: the folder containing the .bag file and the .yaml file created by rosbag_record.sh"
	echo "ROSBAG_FILE: the .bag file. Use the ROSBAG_DIRECTORY if the rosbag was created by rosbag_record.sh" ; tput sgr0
	echo ""
	echo "ROSBAG_PLAY_OPTIONS:"
	rosbag play -h | sed '1,4 d'
	exit 1
fi

directory="$1"
IFS="$(printf '\n\t')"
directory=$(echo "$directory" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
shift

if [ -d "$directory" ]
then
	if expr "$directory" : '.*/$' > /dev/null
	then
		:
	else
		directory="$directory/"
	fi
	
	number_of_bags=$(find "$directory" -maxdepth 1 -name "*.bag" | wc -l)
	number_of_yamls=$(find "$directory" -maxdepth 1 -name "*.yaml" | wc -l)
	if [ "$number_of_bags" -ne 1 ] 
	then
		while true; do
			echo "The directory contains $number_of_bags .bag files. Do you want to play all (a) or choose one (c) rosbag? q to quit:"    			
			read  yn
   			 case $yn in
       			 	[Aa]* ) 
					for f in "$directory"*.bag
							do
								rosbagdirectory=$f
							done; break;;
        			[Cc]* ) 
					{
					while true; do
						echo ""
						tput setaf 6; find "$directory" -maxdepth 1 -name "*.bag"; tput sgr0
						echo "The directory contains $number_of_bags .bag files. Please specify which file to use. q to quit:"     						
						read bag
						bag=$(echo "$bag" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
   						if [ -f "$directory$bag" ]
						then
							rosbagdirectory="$directory$bag"
							break
						elif expr "$bag" : '[Qq]*' > /dev/null
						then
							exit 0
						elif [ $(find "$directory" -maxdepth 1 -name "$bag*.bag" | wc -l) -eq 1 ]
						then
							#rosbagdirectory="$directory$bag"*.bag
							for f in "$directory$bag"*.bag
							do
								rosbagdirectory=$f
							done
							break
						elif [ $(find "$directory" -maxdepth 1 -name "$bag*.bag" | wc -l) -lt 1 ]
						then
							tput setaf 1; echo "no matching bag files"; tput sgr0
						elif [ $(find "$directory" -maxdepth 1 -name "$bag*.bag" | wc -l) -gt 1 ]
						then
							tput setaf 1; echo "multiple matching bag files"; tput sgr0
						fi
					done
					break
					};
					;;
				[Qq]* ) exit 0;;
        			* ) tput setaf 1; echo "a to play all rosbags, c to choose or q to quit"; tput sgr0;;
   			 esac
		done
	else
		for f in $directory*.bag
		do
			rosbagdirectory=$f
		done
	fi

	if [ "$number_of_yamls" -ne 1 ] 
	then
		{
		read_params=true
		while true; do
			echo ""
			tput setaf 6; find "$directory" -maxdepth 1 -name "*.yaml"; tput sgr0
			echo "The directory contains $number_of_yamls .yaml parameter files. Please specify which file to use. n to load no parameters. q to quit:"    			
			read yaml
			yaml=$(echo "$yaml" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
   			if [ -f "$directory$yaml" ]
			then
				break
			elif expr "$yaml" : '[Qq]*' > /dev/null
			then
				exit 0
			elif expr "$yaml" : '[Nn]$' \| $yaml : '[Nn][Oo]$' > /dev/null
			then
				read_params=false
				break
			elif [ $(find "$directory" -maxdepth 1 -name "$yaml*.yaml" | wc -l) -eq 1 ]
			then
				for f in "$yaml"*.yaml
							do
								yaml=$f
							done
				break
			elif [ $(find "$directory" -maxdepth 1 -name "$yaml*.yaml" | wc -l) -lt 1 ]
			then
				tput setaf 1; echo "no matching yaml files"; tput sgr0
			elif [ $(find "$directory" -maxdepth 1 -name "$yaml*.yaml" | wc -l) -gt 1 ]
			then
				tput setaf 1; echo "multiple matching yaml files"; tput sgr0
			fi
		done
		}
		if [ $read_params = true ]
		then
			rosparam load "$directory$yaml" && tput setaf 2; echo "parameters loaded"; tput sgr0
		fi
	else
		rosparam load "$directory"*.yaml && tput setaf 2; echo "parameters loaded"; tput sgr0
	fi
	
	rosbag play -l --pause "$@" "$rosbagdirectory"
elif [ -f "$directory" ]
then
	if expr "$directory" : '.*\.bag$' > /dev/null
	then
		tput setaf 1; echo "$directory is a rosbag file, not a folder containing a rosbag and parameter file"; tput sgr0
		echo "Use the ROSBAG_DIRECTORY option if the rosbag was created by rosbag_record.sh"
		echo ""
		{
		read_params=true
		while true; do
			echo "Do you want to specify a .yaml parameter file(y/n)? q to quit:"			
			read yn
			case "$yn" in
       			 	[Yy]* ) break;;
        			[Nn]* ) read_params=false; break;;
				[Qq]* ) exit 0;;
				* ) tput setaf 1; echo "y to specify a parameter file, n to play rosbag without parameters or q to quit"; tput sgr0;;
			esac
		done
		if [ $read_params = true ]
		then
			while true; do
			echo ""
			echo "Please enter the .yaml path. q to quit:"    			
			read yaml
			yaml=$(echo "$yaml" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
   			if [ -f "$yaml" ]
			then
				if expr "$yaml" : '.*\.yaml$' > /dev/null
				then
					break
				else
					tput bold; tput setaf 1; echo "$yaml is not a yaml file"; tput sgr0
				fi
			elif expr "$yaml" : '[Qq]*' > /dev/null
			then
				exit 0
			else
				tput setaf 1; echo "$yaml is not a file"; tput sgr0
			fi
			done
			rosparam load "$yaml" && tput setaf 2; echo "parameters loaded"; tput sgr0
		else
			tput bold; tput setaf 1; echo "NO PARAMETERS LOADED"; tput sgr0
		fi
		}
		rosbag play -l --pause "$@" "$directory"
	else
		tput bold; tput setaf 1; echo "$directory is neither a folder nor a rosbag-file"; tput sgr0
		echo ""
		tput setaf 2; echo "Usage: sh rosbag_play.sh ROSBAG_DIRECTORY [ROSBAG_PLAY_OPTIONS]"
		echo "or"
		echo "Usage: sh rosbag_play.sh ROSBAG_FILE [ROSBAG_PLAY_OPTIONS]"; tput sgr0
		echo ""
		tput setaf 2; echo "ROSBAG_DIRECTORY: the folder containing the .bag file and the .yaml file created by rosbag_record.sh"
		echo "ROSBAG_FILE: the .bag file. Use the ROSBAG_DIRECTORY if the rosbag was created by rosbag_record.sh" ; tput sgr0
	fi

else
	tput bold; tput setaf 1; echo "$directory is not a valid directory or file"; tput sgr0
	echo ""
	tput setaf 2; echo "Usage: sh rosbag_play.sh ROSBAG_DIRECTORY [ROSBAG_PLAY_OPTIONS]"
	echo "or"
	echo "Usage: sh rosbag_play.sh ROSBAG_FILE [ROSBAG_PLAY_OPTIONS]"; tput sgr0
	echo ""
	tput setaf 2; echo "ROSBAG_DIRECTORY: the folder containing the .bag file and the .yaml file created by rosbag_record.sh"
	echo "ROSBAG_FILE: the .bag file. Use the ROSBAG_DIRECTORY if the rosbag was created by rosbag_record.sh" ; tput sgr0
fi


