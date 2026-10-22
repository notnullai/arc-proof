#!/bin/bash
# one run_scenario run on an idle host: one.sh <out> <tag> <args...>
set -u
cd /d/arc-g13
OUT=$1; T=$2; shift 2; mkdir -p $OUT
while true; do
  busy=$(powershell -NoProfile -Command "(Get-CimInstance Win32_Process -Filter \"Name like 'python%'\" | Where-Object { \$_.CommandLine -match 'run_scenario' } | Measure-Object).Count")
  [ "${busy//[$'\r\n ']/}" = "0" ] && docker rm -f px4gz >/dev/null 2>&1
  cpu=$(powershell -NoProfile -Command "[int]((Get-Counter '\Processor(_Total)\% Processor Time' -SampleInterval 3 -MaxSamples 4).CounterSamples | Measure-Object CookedValue -Average).Average")
  if [ "${busy//[$'\r\n ']/}" = "0" ] && [ "${cpu//[$'\r\n ']/}" -lt 25 ]; then break; fi
  sleep 30
done
echo "=== $T $(date +%T)"
python tools/px4/run_scenario.py --out $OUT --tag $T "$@" > $OUT/$T.log 2>&1
grep -o "HANDOVER.\{0,400\}" $OUT/$T.log
