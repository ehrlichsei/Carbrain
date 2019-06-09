#!/bin/bash

trap ctrl_c INT
function ctrl_c() {
   echo "Interrupting process"
   killall perf || exit -1
}

if [ "$FLAMEGRAPH" == "" ] ; then
  echo "FLAMEGRAPH env is not set! Please git clone https://github.com/brendangregg/FlameGraph somewhere and set the path!"
  exit -1
fi

echo "Please make sure you run a debug build!"
echo "setting permissions"
sudo chmod a+r /proc/kallsyms
sudo -- sh -c "echo 0 > /proc/sys/kernel/kptr_restrict"
sudo -- sh -c "echo -1 > /proc/sys/kernel/perf_event_paranoid"

echo "Recording stack traces for $@ seconds."

perf record -F 99 -a -g -- sleep $@
perf script | $FLAMEGRAPH/stackcollapse-perf.pl > out.perf-folded
$FLAMEGRAPH/flamegraph.pl out.perf-folded > flamegraph.svg

echo "Result saved in flamegraph.svg"

