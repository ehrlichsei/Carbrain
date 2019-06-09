#!/bin/bash
stty icanon #rosbag play sometimes doesn't reset icanon => without this line the script might loop indefinitely if it is started multiple times in the same terminal because read returns instantly if icanon is not set 
BIFS="$IFS"
if [ $# -eq 0 ] 
then
	tput setaf 2; echo "Usage: sh rosbag_play.sh ROSBAG_DIRECTORY [ROSBAG_PLAY_OPTIONS] [-n|--no-params]"
	echo "or"
	echo "Usage: sh rosbag_play.sh ROSBAG_FILE [ROSBAG_PLAY_OPTIONS] [-n|--no-params]"; tput sgr0
	echo ""
	echo "Load parameters and play rosbag created by rosbag_record.sh."
	echo ""
	echo "-n, --no-params		skip parameter loading"
	echo ""
	tput setaf 2; echo "ROSBAG_DIRECTORY: the folder containing the .bag file and the .yaml file created by rosbag_record.sh"
	echo "ROSBAG_FILE: the .bag file. Use the ROSBAG_DIRECTORY if the rosbag was created by rosbag_record.sh" ; tput sgr0
	echo ""
	echo "ROSBAG_PLAY_OPTIONS:"
	rosbag play -h | sed '1,4 d'
	exit 0
elif [ "$1" = '-h' ] || [ "$1" = '--help' ]
then
	tput setaf 2; echo "Usage: sh rosbag_play.sh ROSBAG_DIRECTORY [ROSBAG_PLAY_OPTIONS] [-n|--no-params]"
	echo "or"
	echo "Usage: sh rosbag_play.sh ROSBAG_FILE [ROSBAG_PLAY_OPTIONS] [-n|--no-params]"; tput sgr0
	echo ""
	echo "Load parameters and play rosbag created by rosbag_record.sh."
	echo ""
	echo "-n, --no-params		skip parameter loading"
	echo ""
	tput setaf 2; echo "ROSBAG_DIRECTORY: the folder containing the .bag file and the .yaml file created by rosbag_record.sh"
	echo "ROSBAG_FILE: the .bag file. Use the ROSBAG_DIRECTORY if the rosbag was created by rosbag_record.sh" ; tput sgr0
	echo ""
	echo "ROSBAG_PLAY_OPTIONS:"
	rosbag play -h | sed '1,4 d'
	exit 0
fi

no_params=false
POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -n|--no-params)
    no_params=true
    tput setaf 1; echo "no parameters loaded"; tput sgr0
    shift # past argument
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters


if expr "$1" : "-.*" > /dev/null
then
	tput bold; tput setaf 1; echo "The ROSBAG_DIRECTORY has to be the first argument"; tput sgr0
	echo ""
	echo "Load parameters and play rosbag created by rosbag_record.sh."
	echo ""
	echo "-n, --no-params	 	skip parameter loading"
	echo ""
	tput setaf 2; echo "Usage: sh rosbag_play.sh ROSBAG_DIRECTORY [ROSBAG_PLAY_OPTIONS] [-n|--no-params]"
	echo "or"
	echo "Usage: sh rosbag_play.sh ROSBAG_FILE [ROSBAG_PLAY_OPTIONS] [-n|--no-params]"; tput sgr0
	echo ""
	tput setaf 2; echo "ROSBAG_DIRECTORY: the folder containing the .bag file and the .yaml file created by rosbag_record.sh"
	echo "ROSBAG_FILE: the .bag file. Use the ROSBAG_DIRECTORY if the rosbag was created by rosbag_record.sh" ; tput sgr0
	echo ""
	echo "ROSBAG_PLAY_OPTIONS:"
	rosbag play -h | sed '1,4 d'
	exit 1
fi

directory="$1"
IFS=$(printf '\n\t')
directory=$(echo "$directory" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
shift

do_eval=false

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
			read yn
   			 case $yn in
       			 	[Aa]* )
						rosbagdirectory=$(find "$directory" -maxdepth 1 -type f -name "*.bag" -printf "\"%p\" ")
						if [ $(expr "$rosbagdirectory" : "[0-9a-zA-Z_/~. ÄäÜüÖöß()\"-]*$") -eq 0 ] 
						then
							tput setaf 1; echo "path contains invalid symbols: cannot start rosbags with one command"; tput sgr0
							tput setaf 1; echo "you can play the rosbags one at a time (c)"; tput sgr0
						else
							do_eval=true
							break
						fi
						;;
        			[Cc]* ) 
					{
					while true; do
						echo ""
						files=$(find "$directory" -maxdepth 1 -type f -name "*.bag" -printf "%f\n")
						OIFS="$IFS"
IFS='
'
#the newline above is important
						number=0

						for file in $files  
						do
                  number=$((number + 1))
     							tput setaf 6; printf "%d: %s\n" $number $file; tput sgr0
						done
						IFS="$OIFS"
						echo "The directory contains $number_of_bags .bag files. Please enter the number of the file to use. n to load no parameters. q to quit:"	
						read bag
						bag=$(echo "$bag" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
   						if [ -f "$directory$bag" ]
						then
							rosbagdirectory="$directory$bag"
							break
						elif expr "$bag" : '[Qq]*' > /dev/null
						then
							exit 0
						elif expr $bag : '[0-9]*' > /dev/null
						then
						number=0
						for f in $files
							do
                number=$((number + 1)) > /dev/null
								if [ "$bag" -eq "$number" ]
								then
								rosbagdirectory="$directory$f"
								break 2
								fi
							done
						tput setaf 1; printf "please enter a number between 1 and %d\n" $number_of_bags; tput sgr0
						else
						tput setaf 1; printf "please enter a filename, the corrosponding number, n to load no parameters or q to quit "; tput sgr0
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
			rosbagdirectory="$f"
		done
	fi

	if [ $no_params = false ]
	then
		if [ "$number_of_yamls" -ne 1 ] 
		then
			{
			read_params=true
			while true; do
				echo ""
				#tput setaf 6; find "$directory" -maxdepth 1 -name "*.yaml" -printf "%f\n"; tput sgr0
				files=$(find "$directory" -maxdepth 1 -type f -name "*.yaml" -printf "%f\n")
				OIFS="$IFS"
IFS='
'
#the newline above is important
				number=0

				for file in $files  
				do
				number=$((number + 1))
						tput setaf 6; printf "%d: %s\n" $number $file; tput sgr0
				done
				IFS="$OIFS"


				echo "The directory contains $number_of_yamls .yaml parameter files. Please enter the number of the file to use. n to load no parameters. q to quit:"
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
				elif expr $yaml : '[0-9]*' > /dev/null
				then
					number=0
					for f in $files
								do
					number=$((number + 1)) > /dev/null
									if [ "$yaml" -eq "$number" ]
									then
									yaml="$f"
									break 2
									fi
					done
					tput setaf 1; printf "please enter a number between 1 and %d\n" $number_of_yamls; tput sgr0
				else
					tput setaf 1; printf "please enter a filename, the corrosponding number, n to load no parameters or q to quit "; tput sgr0
				fi
			done
			}
			if [ $read_params = true ]
			then
				echo "loading parameters: this might take a few seconds"
				rosparam load "$directory$yaml" && tput setaf 2; echo "parameters loaded"; tput sgr0
			fi
		else
			echo "loading parameters: this might take a few seconds"
			rosparam load "$directory"*.yaml && tput setaf 2; echo "parameters loaded"; tput sgr0
		fi
	fi
	IFS=$BIFS
	if [ $do_eval = true ]
	then
		eval rosbag play "$@" $rosbagdirectory
	else
		rosbag play "$@" "$rosbagdirectory"
	fi
elif [ -f "$directory" ]
then
	if expr "$directory" : '.*\.bag$' > /dev/null
	then
		if [ $no_params = false ]
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
				echo "loading parameters: this might take a few seconds"
				rosparam load "$yaml" && tput setaf 2; echo "parameters loaded"; tput sgr0
			else
				tput bold; tput setaf 1; echo "NO PARAMETERS LOADED"; tput sgr0
			fi
			}
		fi
		IFS=$BIFS
		rosbag play "$@" $directory
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

